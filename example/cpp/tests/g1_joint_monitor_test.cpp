#include "joint_monitor_state.h"
#include <unitree/robot/channel/channel_publisher.hpp>
#include <yaml-cpp/yaml.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cerrno>
#include <csignal>
#include <iostream>
#include <limits>
#include <thread>

using Monitor=unitree_mujoco::JointMonitorState;
using namespace std::chrono_literals;
namespace {
void Require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
void Near(double value,double expected){Require(std::abs(value-expected)<1e-5,"Unexpected numeric value");}
struct Socket {
    int fd=-1;
    ~Socket(){if(fd>=0)close(fd);}
    Socket()=default;Socket(const Socket&)=delete;Socket& operator=(const Socket&)=delete;
};
struct Process {
    pid_t pid=-1;
    ~Process(){if(pid>0){kill(pid,SIGKILL);waitpid(pid,nullptr,0);}}
    void Stop() {
        kill(pid,SIGINT);
        for(int i=0;i<200;++i){int status=0;
            if(waitpid(pid,&status,WNOHANG)==pid){pid=-1;Require(WIFEXITED(status)&&WEXITSTATUS(status)==0,"Monitor did not exit cleanly");return;}
            std::this_thread::sleep_for(10ms);
        }throw std::runtime_error("Monitor shutdown timed out");
    }
};
int FreePort() {
    Socket s;s.fd=socket(AF_INET,SOCK_STREAM,0);Require(s.fd>=0,"socket failed");
    sockaddr_in addr{};addr.sin_family=AF_INET;addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    Require(bind(s.fd,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))==0,"bind failed");
    socklen_t size=sizeof(addr);getsockname(s.fd,reinterpret_cast<sockaddr*>(&addr),&size);return ntohs(addr.sin_port);
}
void Connect(Socket& s,int port) {
    s.fd=socket(AF_INET,SOCK_STREAM,0);Require(s.fd>=0,"socket failed");
    timeval timeout{2,0};setsockopt(s.fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
    setsockopt(s.fd,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
    sockaddr_in addr{};addr.sin_family=AF_INET;addr.sin_port=htons(port);addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    Require(connect(s.fd,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))==0,"HTTP connect failed");
}
void Send(int fd,const std::string& value) {
    size_t offset=0;while(offset<value.size()){auto n=send(fd,value.data()+offset,value.size()-offset,MSG_NOSIGNAL);
        Require(n>0,"HTTP send failed");offset+=n;}
}
std::string Get(int port,const std::string& path,bool post=false) {
    Socket s;Connect(s,port);Send(s.fd,std::string(post?"POST ":"GET ")+path+" HTTP/1.1\r\nHost: localhost\r\n\r\n");
    std::string result;char bytes[4096];for(;;){auto n=recv(s.fd,bytes,sizeof(bytes),0);if(n==0)break;
        Require(n>0,"HTTP response timed out");result.append(bytes,n);}return result;
}
YAML::Node Json(const std::string& response) {
    Require(response.find("200 OK")!=std::string::npos,"HTTP status not OK");
    return YAML::Load(response.substr(response.find("\r\n\r\n")+4));
}
void StateChecks() {
    const auto start=Monitor::Clock::now();Monitor monitor(50,start);Monitor::Cmd cmd{},target{},enable{};Monitor::State state{};
    state.mode_machine()=5;state.motor_state()[0].q()=.25;state.motor_state()[0].dq()=.5;state.motor_state()[0].tau_est()=2;
    state.imu_state().quaternion()={float(std::cos(.15)),0,float(std::sin(.15)),0};
    cmd.motor_cmd()[0].q()=1;cmd.motor_cmd()[0].kp()=10;target.motor_cmd()[0].q()=2;target.motor_cmd()[0].kp()=20;
    monitor.OnCommand(cmd,start);monitor.OnTarget(target,start);monitor.OnEnable(enable,start);monitor.OnState(state,start);
    auto first=YAML::Load(monitor.Snapshot(start));Require(first["joints"].size()==29,"Wrong joint count");
    auto joint=first["joints"][0];Near(joint["q_target"].as<double>(),2);Near(joint["error"].as<double>(),1.75);
    Require(!joint["enabled"].as<bool>()&&joint["active"].as<bool>(),"Enable/active semantics differ");
    Near(first["imu"]["rpy"][1].as<double>(),.3);
    Require(first["joints"][12]["name"].as<std::string>()=="waist_yaw_joint","Wrong DDS order");
    monitor.OnState(state,start+1ms);Require(YAML::Load(monitor.Snapshot(start))["sample_id"].as<int>()==1,"Rate limit failed");
    monitor.OnCommand(cmd,start+600ms);monitor.OnState(state,start+600ms);
    auto fallback=YAML::Load(monitor.Snapshot(start+610ms));joint=fallback["joints"][0];
    Near(joint["q_target"].as<double>(),1);Require(joint["target_available"].as<bool>(),"Fresh command fallback missing");
    Require(joint["enabled"].IsNull()&&fallback["output_enabled"].IsNull(),"Enable expiry failed");Near(fallback["age_ms"].as<double>(),10);
    monitor.OnState(state,start+1200ms);auto expired=YAML::Load(monitor.Snapshot(start+1200ms));
    Require(!expired["joints"][0]["target_available"].as<bool>()&&!expired["joints"][0]["active"].as<bool>(),"Command expiry failed");
    state.motor_state()[0].q()=std::numeric_limits<float>::quiet_NaN();monitor.OnState(state,start+1300ms);
    Require(YAML::Load(monitor.Snapshot(start+1300ms))["sample_id"].as<int>()==3,"Invalid state accepted");
    state.motor_state()[0].q()=.25;state.mode_machine()=3;monitor.OnState(state,start+1400ms);
    Require(YAML::Load(monitor.Snapshot(start+1400ms))["sample_id"].as<int>()==3,"Non-G1 mode accepted");
}
void Integration(const std::string& executable) {
    const auto port=FreePort();const auto domain=100+getpid()%100;
    const auto port_text=std::to_string(port),domain_text=std::to_string(domain);Process process;
    process.pid=fork();Require(process.pid>=0,"fork failed");
    if(process.pid==0){execl(executable.c_str(),executable.c_str(),"--port",port_text.c_str(),"--domain",domain_text.c_str(),nullptr);_exit(127);}
    bool ready=false;
    for(int i=0;i<200&&!ready;++i){try{Get(port,"/api/joints");ready=true;}catch(...){std::this_thread::sleep_for(20ms);}}
    Require(ready,"Monitor did not start");
    auto empty=Json(Get(port,"/api/joints"));Require(empty["joints"].size()==0&&empty["age_ms"].IsNull(),"Initial state differs");
    auto page=Get(port,"/");Require(page.find("new EventSource('/api/stream')")!=std::string::npos,"Embedded page missing SSE frontend");
    Require(page.find("jointLegend")!=std::string::npos&&page.find("imu-quaternion")!=std::string::npos,"Frontend features missing");
    Require(Get(port,"/missing").find("404 Not Found")!=std::string::npos,"Missing 404");
    Require(Get(port,"/api/joints",true).find("405 Method Not Allowed")!=std::string::npos,"Write request accepted");
    // Keep two SSE clients open while querying normal HTTP endpoints.
    Socket stream1,stream2;Connect(stream1,port);Connect(stream2,port);
    for(int fd:{stream1.fd,stream2.fd})Send(fd,"GET /api/stream HTTP/1.1\r\nHost: localhost\r\n\r\n");
    unitree::robot::ChannelFactory::Instance()->Init(domain,"lo");
    unitree::robot::ChannelPublisher<Monitor::State> states("rt/lowstate");states.InitChannel();
    unitree::robot::ChannelPublisher<Monitor::Cmd> commands("rt/lowcmd"),targets("rt/g1/reset_target"),enables("rt/g1/reset_enable");
    commands.InitChannel();targets.InitChannel();enables.InitChannel();
    Monitor::State state{};Monitor::Cmd cmd{},target{},enable{};
    state.mode_machine()=5;state.motor_state()[0].q()=.25;state.imu_state().quaternion()={1,0,0,0};
    cmd.motor_cmd()[0].q()=1;cmd.motor_cmd()[0].kp()=10;target.motor_cmd()[0].q()=2;
    for(auto& m:enable.motor_cmd())m.mode()=1;
    bool received=false;
    for(int i=0;i<100;++i) {
        commands.Write(cmd);targets.Write(target);enables.Write(enable);states.Write(state);std::this_thread::sleep_for(20ms);
        auto data=Json(Get(port,"/api/joints?fresh=1"));
        if(data["joints"].size()==29&&data["output_enabled"].as<bool>(false)&&data["joints"][0]["q_target"].as<double>()==2){received=true;break;}
    }Require(received,"Actual DDS data not visible over HTTP");
    for(int i=0;i<3;++i){states.Write(state);std::this_thread::sleep_for(30ms);}
    for(int fd:{stream1.fd,stream2.fd}) {
        std::string response;char bytes[4096];int first_sample=-1;bool updated=false;
        while(!updated){auto n=recv(fd,bytes,sizeof(bytes),0);Require(n>0,"SSE timeout");response.append(bytes,n);
            Require(response.size()<1024*1024,"SSE did not deliver updated joint data");
            size_t pos=0;
            while((pos=response.find("data: ",pos))!=std::string::npos) {
                const auto end=response.find("\n\n",pos);if(end==std::string::npos)break;
                const auto data=YAML::Load(response.substr(pos+6,end-pos-6));
                if(data["joints"].size()==29) {
                    Near(data["joints"][0]["q"].as<double>(),.25);
                    const int sample=data["sample_id"].as<int>();
                    if(first_sample<0)first_sample=sample;else if(sample>first_sample)updated=true;
                }
                pos=end+2;
            }
        }
        Require(response.find("text/event-stream")!=std::string::npos,"Wrong SSE MIME");
    }
    // DDS stop must age the last sample; server stays responsive.
    std::this_thread::sleep_for(600ms);auto stale=Json(Get(port,"/api/joints"));
    Require(stale["age_ms"].as<double>()>500,"Data age did not increase");
    states.Write(state);std::this_thread::sleep_for(80ms);auto expired=Json(Get(port,"/api/joints"));
    Require(expired["output_enabled"].IsNull()&&!expired["joints"][0]["target_available"].as<bool>(),"DDS target/enable expiry failed");
    // Exit while persistent SSE sockets are still open.
    process.Stop();
}
}
int main(int argc,char** argv) {
    try{Require(argc==2,"Expected production monitor executable");StateChecks();Integration(argv[1]);
        std::cout<<"PASS monitor: state/target/enable freshness, DDS order, IMU, invalid data, rate limit, actual DDS -> HTTP/JSON, two SSE clients, routes, stale data and clean shutdown\n";return 0;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
