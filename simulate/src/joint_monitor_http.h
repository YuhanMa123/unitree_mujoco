#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <functional>
#include <vector>
#include "joint_monitor_state.h"

namespace unitree_mujoco {
// A bounded, nonblocking event loop: a slow browser cannot block DDS callbacks
// or other clients. Only the latest snapshot is queued for each SSE connection.
class JointMonitorHttp {
    struct Client {
        int fd;
        std::string request, output;
        size_t sent=0;
        bool stream=false, close=false;
        JointMonitorState::Time accepted=JointMonitorState::Clock::now(), last{};
    };
    int listener_=-1;
    std::vector<Client> clients_;
    std::string page_;
    JointMonitorState& state_;
    double period_;
    static void Nonblocking(int fd) {
        if(fcntl(fd,F_SETFL,fcntl(fd,F_GETFL,0)|O_NONBLOCK)<0)throw std::runtime_error("Cannot set HTTP socket nonblocking");
        fcntl(fd,F_SETFD,FD_CLOEXEC);
    }
    static std::string Response(const std::string& body,const std::string& type,const char* status="200 OK") {
        return std::string("HTTP/1.1 ")+status+"\r\nContent-Type: "+type+
            "\r\nCache-Control: no-store\r\nConnection: close\r\nContent-Length: "+std::to_string(body.size())+"\r\n\r\n"+body;
    }
    void Request(Client& c) {
        const auto line=c.request.substr(0,c.request.find("\r\n"));
        const auto start=line.find(' '),end=start==std::string::npos?start:line.find(' ',start+1);
        if(line.substr(0,start)!="GET"||end==std::string::npos) {
            c.output=Response("Method not allowed\n","text/plain","405 Method Not Allowed");c.close=true;return;
        }
        auto path=line.substr(start+1,end-start-1);path=path.substr(0,path.find('?'));
        if(path=="/api/stream") {
            c.output="HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nCache-Control: no-store\r\nX-Accel-Buffering: no\r\nConnection: close\r\n\r\n";
            c.stream=true;
        } else {
            c.close=true;
            if(path=="/"||path=="/index.html")c.output=Response(page_,"text/html; charset=utf-8");
            else if(path=="/api/joints")c.output=Response(state_.Snapshot(),"application/json; charset=utf-8");
            else c.output=Response("Not found\n","text/plain","404 Not Found");
        }
        c.request.clear();
    }
public:
    JointMonitorHttp(JointMonitorState& state,const std::string& host,int port,double rate,std::string page)
        :page_(std::move(page)),state_(state),period_(1/rate) {
        sockaddr_in address{};address.sin_family=AF_INET;address.sin_port=htons(port);
        if(inet_pton(AF_INET,host.c_str(),&address.sin_addr)!=1)throw std::runtime_error("--host must be an IPv4 address");
        listener_=socket(AF_INET,SOCK_STREAM,0);
        if(listener_<0)throw std::runtime_error("Cannot create HTTP socket: "+std::string(strerror(errno)));
        try {
            int yes=1;setsockopt(listener_,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes));Nonblocking(listener_);
            if(bind(listener_,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0||listen(listener_,32)<0)
                throw std::runtime_error("Cannot listen on "+host+":"+std::to_string(port)+": "+strerror(errno));
        }catch(...){::close(listener_);listener_=-1;throw;}
    }
    ~JointMonitorHttp(){for(const auto& c:clients_)::close(c.fd);if(listener_>=0)::close(listener_);}
    JointMonitorHttp(const JointMonitorHttp&)=delete;
    JointMonitorHttp& operator=(const JointMonitorHttp&)=delete;
    void Run(const volatile std::sig_atomic_t& stopping) {
        while(!stopping) {
            std::vector<pollfd> events{{listener_,POLLIN,0}};
            auto now=JointMonitorState::Clock::now();
            for(auto& c:clients_) {
                if(c.stream&&c.output.empty()&&std::chrono::duration<double>(now-c.last).count()>=period_) {
                    c.output="data: "+state_.Snapshot(now)+"\n\n";c.last=now;
                }
                events.push_back({c.fd,static_cast<short>(c.output.empty()?POLLIN:POLLOUT),0});
            }
            if(poll(events.data(),events.size(),10)<0){if(errno==EINTR)continue;throw std::runtime_error("HTTP poll failed");}
            // Existing clients first: accept must not invalidate the poll index mapping.
            for(size_t i=0;i<clients_.size();++i) {
                auto& c=clients_[i];const short event=events[i+1].revents;
                bool remove=event&(POLLERR|POLLHUP|POLLNVAL);
                if(!remove&&(event&POLLIN)) {
                    char bytes[4096];auto n=recv(c.fd,bytes,sizeof(bytes),0);
                    if(n==0||(n<0&&errno!=EAGAIN&&errno!=EWOULDBLOCK&&errno!=EINTR))remove=true;
                    else if(n>0&&!c.stream) {
                        c.request.append(bytes,n);
                        if(c.request.size()>8192)remove=true;
                        else if(c.request.find("\r\n\r\n")!=std::string::npos)Request(c);
                    }
                }
                if(!remove&&(event&POLLOUT)&&!c.output.empty()) {
                    auto n=send(c.fd,c.output.data()+c.sent,c.output.size()-c.sent,MSG_NOSIGNAL);
                    if(n<0&&errno!=EAGAIN&&errno!=EWOULDBLOCK&&errno!=EINTR)remove=true;
                    if(n>0)c.sent+=n;
                    if(c.sent==c.output.size()){c.output.clear();c.sent=0;if(c.close)remove=true;}
                }
                // Bound slow/incomplete HTTP requests; SSE is intentionally persistent.
                if(!c.stream&&now-c.accepted>std::chrono::seconds(5))remove=true;
                if(remove){::close(c.fd);c.fd=-1;}
            }
            clients_.erase(std::remove_if(clients_.begin(),clients_.end(),[](const Client& c){return c.fd<0;}),clients_.end());
            if(events[0].revents&POLLIN) {
                int fd=accept(listener_,nullptr,nullptr);
                if(fd>=0) {
                    if(clients_.size()>=32)::close(fd);
                    else {try{Nonblocking(fd);clients_.push_back(Client{fd});}catch(...){::close(fd);throw;}}
                }
            }
        }
    }
};
} // namespace unitree_mujoco
