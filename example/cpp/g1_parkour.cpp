// Inference/controller for simulate_python/unitree_mujoco.py; simulation only.
#include "g1_parkour_policy.h"
#include "g1_hardware.h"
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <poll.h>
#include <termios.h>
#include <chrono>
#include <csignal>
#include <cerrno>
#include <iostream>
#include <mutex>
#include <thread>

using Clock=std::chrono::steady_clock;
using Cmd=unitree_hg::msg::dds_::LowCmd_;
using State=unitree_hg::msg::dds_::LowState_;
volatile std::sig_atomic_t stopped=0;
void Stop(int) {stopped=1;}

class Keyboard {
    termios saved_{}; bool restore_=false;
public:
    Keyboard() {
        if(!isatty(0)) return;
        if(tcgetattr(0,&saved_)!=0) throw std::runtime_error("Cannot read terminal settings");
        auto settings=saved_; settings.c_lflag&=~(ICANON|ECHO);settings.c_cc[VMIN]=0;settings.c_cc[VTIME]=0;
        if(tcsetattr(0,TCSANOW,&settings)!=0) throw std::runtime_error("Cannot set terminal settings");
        restore_=true;
    }
    ~Keyboard() {if(restore_) tcsetattr(0,TCSANOW,&saved_);}
    char Read() {
        pollfd fd{0,POLLIN,0};int r=poll(&fd,1,0);
        if(r<0 && errno!=EINTR) throw std::runtime_error("Keyboard poll failed");
        if(r<=0) return 0;
        char key=0;auto n=read(0,&key,1);return n==0?' ':key;
    }
};

class Receiver {
    int fd_=-1;
public:
    explicit Receiver(int port) {
        fd_=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK,0);
        if(fd_<0) throw std::runtime_error("Cannot create observation socket");
        sockaddr_in address{};address.sin_family=AF_INET;address.sin_port=htons(port);
        address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        if(bind(fd_,reinterpret_cast<sockaddr*>(&address),sizeof(address))!=0) {
            close(fd_);throw std::runtime_error("Cannot bind observation port (another controller running?)");
        }
    }
    ~Receiver() {close(fd_);}
    bool Latest(g1_parkour::Observation& result,uint32_t signature) {
        bool updated=false;std::array<unsigned char,65536> packet{};
        // Bounded drain: at 50Hz the queue normally contains zero or one packet.
        for(int i=0;i<64;++i) {
            const auto n=recv(fd_,packet.data(),packet.size(),0);
            if(n<0) {
                if(errno==EAGAIN || errno==EWOULDBLOCK) break;
                if(errno==EINTR) continue;
                throw std::runtime_error("Observation receive failed");
            }
            result=g1_parkour::Decode(packet.data(),n,signature);updated=true;
        }
        return updated;
    }
};

int main(int argc,char** argv) {
    std::signal(SIGINT,Stop); std::signal(SIGTERM,Stop);
    std::string models=G1_PARKOUR_DEFAULT_MODELS, config=G1_PARKOUR_DEFAULT_CONFIG;
    int port=17200,domain=1;double reset_seconds=3;bool check=false;
    try {
        for(int i=1;i<argc;++i) {
            std::string arg=argv[i];
            auto value=[&]() {if(i+1>=argc) throw std::runtime_error("Missing value for "+arg);return std::string(argv[++i]);};
            if(arg=="--model-dir") models=value();else if(arg=="--config") config=value();
            else if(arg=="--port") port=std::stoi(value());else if(arg=="--domain") domain=std::stoi(value());
            else if(arg=="--reset-seconds") reset_seconds=std::stod(value());
            else if(arg=="--check-policy") check=true;
            else if(arg=="--help") {
                std::cout<<"g1_parkour [--model-dir DIR] [--config JSON] [--domain 1] [--port 17200] [--reset-seconds 3] [--check-policy]\nSimulation only, DDS interface lo. Keys: 9 -> p -> r; w/s forward, q/e yaw, x stop velocity, Space exit.\n";return 0;
            } else throw std::runtime_error("Unknown argument: "+arg);
        }
        if(domain<=0 || port<1024 || port>65535 || !std::isfinite(reset_seconds) || reset_seconds<=0)
            throw std::runtime_error("Use positive simulation domain, valid UDP port and positive reset duration");
        g1_parkour::Policy policy(models,config);
        if(check) {
            g1_parkour::Observation obs;obs.q=policy.neutral;obs.gravity={0,0,-1};obs.depth.fill(2.5f);
            auto target=policy.Step(obs,{0,0,0});
            std::cout<<"CPU parkour inference OK; targets (DDS):";
            for(float v:target) std::cout<<' '<<v;
            std::cout<<'\n';return 0;
        }
        Receiver receiver(port);
        unitree::robot::ChannelFactory::Instance()->Init(domain,"lo");
        unitree::robot::ChannelPublisher<Cmd> publisher("rt/lowcmd");publisher.InitChannel();
        unitree::robot::ChannelPublisher<Cmd> targets("rt/g1/reset_target");targets.InitChannel();
        unitree::robot::ChannelPublisher<Cmd> enable("rt/g1/reset_enable");enable.InitChannel();
        std::mutex mutex; State state{};Clock::time_point state_time{};bool have_state=false;
        // Subscriber is declared after captured objects so its destructor runs first.
        unitree::robot::ChannelSubscriber<State> subscriber("rt/lowstate");
        subscriber.InitChannel([&](const void* message) {
            auto incoming=*static_cast<const State*>(message);
            if(incoming.mode_machine()!=0 && incoming.mode_machine()!=5) return;
            for(int i=0;i<29;++i) if(!std::isfinite(incoming.motor_state()[i].q()) || !std::isfinite(incoming.motor_state()[i].dq())) return;
            std::lock_guard<std::mutex> lock(mutex);state=incoming;state_time=Clock::now();have_state=true;
        },1);
        Keyboard keyboard;
        Cmd command{};command.mode_pr()=0;
        bool powered=false,active=false,holding=false,running=false,publishing=false;
        g1_parkour::Observation obs;bool have_obs=false;
        Clock::time_point obs_time{},reset_start{};
        g1_parkour::Joints start{},target{},neutral=policy.DefaultDds();
        std::array<float,3> velocity{};
        uint64_t sequence=0;double sim_time=-1;
        auto next=Clock::now(), waiting_since=next;
        auto send=[&](Cmd& cmd) {cmd.crc()=g1_hardware::Crc(cmd);if(!publisher.Write(cmd)) throw std::runtime_error("DDS command publication failed");};
        auto clear=[&]() {
            for(auto& motor:command.motor_cmd()) motor.kp()=motor.kd()=motor.tau()=motor.dq()=motor.q()=0;
            command.crc()=g1_hardware::Crc(command);
            if(publishing) for(int i=0;i<3;++i) {publisher.Write(command);std::this_thread::sleep_for(std::chrono::milliseconds(2));}
            Cmd status{};status.crc()=g1_hardware::Crc(status);enable.Write(status);
        };
        std::cout<<"Waiting for Python MuJoCo observations on 127.0.0.1:"<<port<<" and DDS domain "<<domain<<".\n9: enable, p: reset, r: run; Space / Ctrl+C: exit.\n"<<std::flush;
        int result=0;
        try {
            while(!stopped) {
                const auto now=Clock::now();
                bool fresh=receiver.Latest(obs,policy.signature);
                if(fresh) {
                    if(have_obs && (obs.sequence<=sequence || obs.time<=sim_time)) {
                        if(active) throw std::runtime_error("Simulator reset/restarted; restart controller");
                    }
                    if(running && obs.time-sim_time>.05) throw std::runtime_error("Observation frames dropped; history timing invalid");
                    have_obs=true;sequence=obs.sequence;sim_time=obs.time;obs_time=now;
                }
                bool valid_state=false;State snapshot;
                {std::lock_guard<std::mutex> lock(mutex);valid_state=have_state && now-state_time<std::chrono::milliseconds(500);snapshot=state;}
                bool ready=valid_state && have_obs && now-obs_time<std::chrono::milliseconds(150);
                if(!ready && active) throw std::runtime_error("Simulator observations or DDS lowstate stale; stopping");
                if(active && snapshot.mode_machine()!=command.mode_machine()) throw std::runtime_error("DDS robot mode changed; stopping");
                if(!ready && now-waiting_since>std::chrono::seconds(10)) throw std::runtime_error("No synchronized simulator input; check PARKOUR_OBSERVATIONS, domain and UDP port");
                if(ready) waiting_since=now;
                const char key=keyboard.Read();if(key==' ') break;
                if(key=='9' && !active && ready) {powered=true;std::cout<<"Output enabled; press p to reset.\n"<<std::flush;}
                if((key=='p'||key=='P') && powered && ready) {
                    for(int i=0;i<29;++i) start[policy.ids[i]]=policy.sign[i]*obs.q[i];
                    reset_start=now;active=true;holding=false;running=false;velocity.fill(0);policy.Reset();
                    command.mode_machine()=snapshot.mode_machine();
                    std::cout<<"Reset started.\n"<<std::flush;
                }
                if((key=='r'||key=='R') && holding && ready && !running) {
                    bool near=true;for(int i=0;i<29;++i) if(std::abs(obs.q[i]-policy.neutral[i])>.35) near=false;
                    if(near) {policy.Reset();running=true;fresh=true;velocity.fill(0);std::cout<<"Parkour policy ON.\n"<<std::flush;}
                    else std::cout<<"Joints not near training pose; keep band enabled and reset.\n"<<std::flush;
                }
                if(running) {
                    switch(key) {
                        case 'w':velocity[0]+=.1f;break;case 's':velocity[0]-=.1f;break;
                        case 'q':velocity[2]+=.1f;break;case 'e':velocity[2]-=.1f;break;
                        case 'x':velocity.fill(0);break;
                    }
                    velocity[0]=std::clamp(velocity[0],0.f,1.f);
                    velocity[1]=0;velocity[2]=std::clamp(velocity[2],-1.f,1.f);
                    if(key=='w'||key=='s'||key=='q'||key=='e'||key=='x')
                        std::cout<<"Velocity: "<<velocity[0]<<", 0, "<<velocity[2]<<'\n'<<std::flush;
                    if(fresh) {
                        if(obs.height<.5 || obs.gravity[2]>-std::cos(1.)) throw std::runtime_error("Torso height/tilt protection triggered");
                        const auto before=Clock::now();target=policy.Step(obs,velocity);
                        if(Clock::now()-before>std::chrono::milliseconds(20)) throw std::runtime_error("Inference exceeded 20ms period");
                    }
                }
                if(active) {
                    if(!running) {
                        double u=std::clamp(std::chrono::duration<double>(now-reset_start).count()/reset_seconds,0.,1.);
                        double blend=u*u*(3-2*u);
                        for(int i=0;i<29;++i) target[i]=start[i]+blend*(neutral[i]-start[i]);
                        if(u==1 && !holding) {holding=true;std::cout<<"Holding training pose; press r for parkour.\n"<<std::flush;}
                    }
                    for(int i=0;i<29;++i) {
                        auto& motor=command.motor_cmd()[policy.ids[i]];motor.mode()=1;
                        motor.q()=target[policy.ids[i]];motor.dq()=motor.tau()=0;motor.kp()=policy.kp[i];motor.kd()=policy.kd[i];
                    }
                    send(command);publishing=true;targets.Write(command);
                }
                Cmd status{};
                for(int i=0;i<29;++i) status.motor_cmd()[i].mode()=powered?1:0;
                status.crc()=g1_hardware::Crc(status);enable.Write(status);
                next+=std::chrono::milliseconds(2);if(next<Clock::now()) next=Clock::now();
                std::this_thread::sleep_until(next);
            }
        } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';result=1;}
        clear();subscriber.CloseChannel();return result;
    } catch(const std::exception& e) {std::cerr<<"g1_parkour: "<<e.what()<<'\n';return 1;}
}
