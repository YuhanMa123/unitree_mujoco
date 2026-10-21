// Shared simulation-only DDS/UDP controller. Include after selecting g1_visual_policy.
// ONNX controller for the C++ or Python MuJoCo parkour observation publisher.
#include "g1_hardware.h"
#include <unitree_mujoco/gamepad.h>
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
        const int buffer_bytes=2*1024*1024;
        setsockopt(fd_,SOL_SOCKET,SO_RCVBUF,&buffer_bytes,sizeof(buffer_bytes));
        sockaddr_in address{};address.sin_family=AF_INET;address.sin_port=htons(port);
        address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        if(bind(fd_,reinterpret_cast<sockaddr*>(&address),sizeof(address))!=0) {
            close(fd_);throw std::runtime_error("Cannot bind observation port (another controller running?)");
        }
    }
    ~Receiver() {close(fd_);}
    std::vector<g1_visual_policy::Observation> Pending(uint32_t signature) {
        std::vector<g1_visual_policy::Observation> result;std::array<unsigned char,65536> packet{};
        // Preserve every queued sample: keeping only the newest destroys history
        // whenever the render/OS scheduler delivers two packets in one tick.
        for(int i=0;i<64;++i) {
            const auto n=recv(fd_,packet.data(),packet.size(),0);
            if(n<0) {
                if(errno==EAGAIN || errno==EWOULDBLOCK) break;
                if(errno==EINTR) continue;
                throw std::runtime_error("Observation receive failed");
            }
            result.push_back(g1_visual_policy::Decode(packet.data(),n,signature));
        }
        return result;
    }
};

class StateSubscriber {
    unitree::robot::ChannelPtr<State> channel_;
public:
    explicit StateSubscriber(const std::function<void(const void*)>& callback)
        : channel_(unitree::robot::ChannelFactory::Instance()->CreateRecvChannel<State>("rt/lowstate",callback,1)) {}
    ~StateSubscriber() {
        // Keep a strong reader reference while closing on the control thread.
        // Dropping the final reference during a DDS callback can deadlock/assert.
        if(channel_) {
            try {
                auto reader=channel_->GetReader()->GetNative();
                reader.close();
            } catch(const std::exception& error) {
                std::cerr<<"DDS subscriber close: "<<error.what()<<'\n';
            }
            channel_.reset();
        }
    }
};

int main(int argc,char** argv) {
    std::signal(SIGINT,Stop); std::signal(SIGTERM,Stop);
    std::string models=G1_VISUAL_DEFAULT_MODELS, config=G1_VISUAL_DEFAULT_CONFIG;
    int port=17200,domain=1;double reset_seconds=3;bool check=false,use_gamepad=true;
    try {
        for(int i=1;i<argc;++i) {
            std::string arg=argv[i];
            auto value=[&]() {if(i+1>=argc) throw std::runtime_error("Missing value for "+arg);return std::string(argv[++i]);};
            if(arg=="--model-dir") models=value();else if(arg=="--config") config=value();
            else if(arg=="--port") port=std::stoi(value());else if(arg=="--domain") domain=std::stoi(value());
            else if(arg=="--reset-seconds") reset_seconds=std::stod(value());
            else if(arg=="--check-policy") check=true;
            else if(arg=="--no-gamepad") use_gamepad=false;
            else if(arg=="--help") {
                std::cout<<argv[0]<<" [--model-dir DIR] [--config JSON] [--domain 1] [--port 17200] [--reset-seconds 3] [--check-policy] [--no-gamepad]\nSimulation only, DDS interface lo. Keyboard: 9 -> p -> wait -> r; Xbox: B -> wait -> Menu. w/s forward, a/d lateral (when supported), q/e yaw, x stop velocity, Space exit.\n";return 0;
            } else throw std::runtime_error("Unknown argument: "+arg);
        }
        if(domain<=0 || port<1024 || port>65535 || !std::isfinite(reset_seconds) || reset_seconds<=0)
            throw std::runtime_error("Use positive simulation domain, valid UDP port and positive reset duration");
        g1_visual_policy::Policy policy(models,config);
        std::cout<<G1_VISUAL_POLICY_NAME<<" config: "<<config<<"\nModels: "<<models<<'\n';
        // Load/validate both networks and warm CPU kernels before starting the timer.
        g1_visual_policy::Observation probe;probe.q=policy.neutral;probe.gravity={0,0,-1};probe.depth.fill(2.5f);
        const auto probe_target=policy.Step(probe,{0,0,0});policy.Reset();
        std::cout<<G1_VISUAL_NETWORK_DESCRIPTION<<"\n";
        if(check) {
            std::cout<<"CPU visual inference OK; targets (DDS):";
            for(float v:probe_target) std::cout<<' '<<v;
            std::cout<<'\n';return 0;
        }
        Receiver receiver(port);
        unitree::robot::ChannelFactory::Instance()->Init(domain,"lo");
        unitree::robot::ChannelPublisher<Cmd> publisher("rt/lowcmd");publisher.InitChannel();
        unitree::robot::ChannelPublisher<Cmd> targets("rt/g1/reset_target");targets.InitChannel();
        unitree::robot::ChannelPublisher<Cmd> enable("rt/g1/reset_enable");enable.InitChannel();
        std::mutex mutex; State state{};Clock::time_point state_time{};bool have_state=false;
        // Subscriber is declared after captured objects so its destructor runs first.
        StateSubscriber subscriber([&](const void* message) {
            auto incoming=*static_cast<const State*>(message);
            if(incoming.mode_machine()!=0 && incoming.mode_machine()!=5) return;
            for(int i=0;i<29;++i) if(!std::isfinite(incoming.motor_state()[i].q()) || !std::isfinite(incoming.motor_state()[i].dq())) return;
            std::lock_guard<std::mutex> lock(mutex);state=incoming;state_time=Clock::now();have_state=true;
        });
        Keyboard keyboard;
        unitree_mujoco::gamepad::Config gamepad_config;
        gamepad_config.enabled=false;
        if(use_gamepad) gamepad_config=unitree_mujoco::gamepad::Config::Load(G1_GAMEPAD_DEFAULT_CONFIG);
        unitree_mujoco::gamepad::Reader gamepad_reader(gamepad_config);
        unitree_mujoco::gamepad::Drive gamepad_drive(gamepad_config);
        bool gamepad_reported=false,gamepad_connected=false,inputs_ready=false;
        Cmd command{};command.mode_pr()=0;
        bool powered=false,active=false,holding=false,running=false,publishing=false;
        g1_visual_policy::Observation obs;bool have_obs=false;
        Clock::time_point obs_time{},reset_start{};
        g1_visual_policy::Joints start{},target{},neutral=policy.DefaultDds();
        std::array<float,3> velocity{};
        uint64_t sequence=0;double sim_time=-1;
        auto next=Clock::now(), waiting_since=next,next_velocity_log=next;
        auto next_policy_log=next;
        std::uint64_t inference_frames=0;
        auto send=[&](Cmd& cmd) {cmd.crc()=g1_hardware::Crc(cmd);if(!publisher.Write(cmd)) throw std::runtime_error("DDS command publication failed");};
        auto clear=[&]() {
            for(auto& motor:command.motor_cmd()) motor.kp()=motor.kd()=motor.tau()=motor.dq()=motor.q()=0;
            command.crc()=g1_hardware::Crc(command);
            if(publishing) for(int i=0;i<3;++i) {publisher.Write(command);std::this_thread::sleep_for(std::chrono::milliseconds(2));}
            Cmd status{};status.crc()=g1_hardware::Crc(status);enable.Write(status);
        };
        std::cout<<"Waiting for MuJoCo visual observations on 127.0.0.1:"<<port<<" and DDS domain "<<domain<<".\nKeyboard: 9 -> p -> wait for Holding training pose -> r; Space / Ctrl+C: exit.\n";
        if(gamepad_config.enabled)
            std::cout<<"Xbox: B enables/resets joints, Menu starts policy, left stick / D-pad velocity, right stick turns, View / R3 stops velocity; A/Y adjusts band without stopping policy.\n";
        std::cout<<std::flush;
        int result=0;
        try {
            while(!stopped) {
                const auto now=Clock::now();
                std::vector<g1_visual_policy::Observation> inference_samples;
                for(const auto& incoming:receiver.Pending(policy.signature)) {
                    if(have_obs && incoming.sequence==sequence && incoming.time==sim_time) continue;
                    const bool reset=have_obs && (incoming.sequence<sequence || incoming.time<sim_time);
                    if(reset) {
                        policy.Reset();velocity.fill(0);gamepad_drive.Clear();
                        inference_samples.clear();
                        if(active && !running) {
                            for(int i=0;i<29;++i) start[policy.ids[i]]=policy.sign[i]*incoming.q[i];
                            reset_start=now;holding=false;
                        }
                        std::cout<<"Simulation reset received; policy "<<(running?"ON":"OFF")
                                 <<"; history and velocity cleared.\n"<<std::flush;
                    }
                    if(running && !reset) {
                        auto frames=g1_visual_policy::RecoverObservationGap(obs,incoming,policy.dt);
                        if(frames.size()>1)
                            std::cout<<"Observation gap recovered: "<<frames.size()-1<<" missing sample(s), "
                                     <<(incoming.time-sim_time)*1000<<"ms; policy remains ON.\n"<<std::flush;
                        inference_samples.insert(inference_samples.end(),frames.begin(),frames.end());
                    } else if(running) inference_samples.push_back(incoming);
                    if(inference_samples.size()>7)
                        throw std::runtime_error("Observation backlog exceeds 120ms recovery window; stopping");
                    obs=incoming;
                    have_obs=true;sequence=obs.sequence;sim_time=obs.time;obs_time=now;
                }
                bool valid_state=false;State snapshot;
                {std::lock_guard<std::mutex> lock(mutex);valid_state=have_state && now-state_time<std::chrono::milliseconds(500);snapshot=state;}
                bool ready=valid_state && have_obs && now-obs_time<std::chrono::milliseconds(150);
                if(!ready && active) throw std::runtime_error("Simulator observations or DDS lowstate stale; stopping");
                if(active && snapshot.mode_machine()!=command.mode_machine()) throw std::runtime_error("DDS robot mode changed; stopping");
                if(!ready && now-waiting_since>std::chrono::seconds(10)) throw std::runtime_error("No synchronized simulator input; enable --parkour (C++) or PARKOUR_OBSERVATIONS (Python), and check domain/UDP port");
                if(ready) waiting_since=now;
                if(ready && !inputs_ready)
                    std::cout<<"Inputs ready (DDS + synchronized observations). Press B, or keyboard 9 then p, to initialize joints.\n"<<std::flush;
                inputs_ready=ready;
                const char key=keyboard.Read();if(key==' ') break;
                const auto pad=gamepad_reader.Poll();
                if(gamepad_config.enabled && (!gamepad_reported || pad.connected!=gamepad_connected)) {
                    if(pad.connected) std::cout<<"Controller gamepad connected: "<<gamepad_reader.Name()<<'\n';
                    else std::cout<<"Controller gamepad unavailable: "<<gamepad_reader.Error()<<"; keyboard available; retrying.\n";
                    std::cout<<std::flush;gamepad_reported=true;gamepad_connected=pad.connected;
                }
                const auto action=gamepad_drive.Update(pad,velocity,running);
                const bool reset_requested=key=='p'||key=='P'||action.reset;
                const bool start_requested=key=='r'||key=='R'||action.start;
                const bool enable_requested=key=='9'||(action.reset&&!powered);
                if(action.stop||action.suspend||action.simulation_reset) velocity.fill(0);
                if(enable_requested) {
                    if(!ready) std::cout<<"Enable rejected: waiting for fresh DDS lowstate and visual observations.\n"<<std::flush;
                    else if(!active) {powered=true;std::cout<<"Output enabled; press p to reset.\n"<<std::flush;}
                }
                if(reset_requested && (!powered||!ready))
                    std::cout<<"Joint reset rejected: "<<(!ready?"waiting for fresh simulator inputs":"press 9 first, or use gamepad B")<<".\n"<<std::flush;
                if(reset_requested && powered && ready) {
                    for(int i=0;i<29;++i) start[policy.ids[i]]=policy.sign[i]*obs.q[i];
                    reset_start=now;active=true;holding=false;running=false;velocity.fill(0);policy.Reset();gamepad_drive.Clear();
                    command.mode_machine()=snapshot.mode_machine();
                    std::cout<<"Reset started ("<<(action.reset?"gamepad B":"keyboard p")<<").\n"<<std::flush;
                }
                if(start_requested && !reset_requested) {
                    if(running) std::cout<<G1_VISUAL_POLICY_NAME<<" policy already ON.\n"<<std::flush;
                    else if(!ready) std::cout<<"Policy start rejected: waiting for fresh DDS lowstate and visual observations.\n"<<std::flush;
                    else if(!powered||!active) std::cout<<"Policy start rejected: initialize joints first. Keyboard 9 -> p -> wait -> r; gamepad B -> wait -> Menu.\n"<<std::flush;
                    else if(!holding) std::cout<<"Policy start rejected: joint reset still running; wait for Holding training pose, then press r / Menu again.\n"<<std::flush;
                    else {
                        float maximum_error=0;int worst=0;
                        for(int i=0;i<29;++i) {
                            const float error=std::abs(obs.q[i]-policy.neutral[i]);
                            if(error>maximum_error) {maximum_error=error;worst=policy.ids[i];}
                        }
                        if(obs.height<.5 || obs.gravity[2]>-std::cos(1.)) {
                            std::cout<<"Policy start rejected: robot is fallen/tilted; state body height="<<obs.height
                                     <<"m, gravity_z="<<obs.gravity[2]<<". Press RB / Backspace to reset simulation, then B -> wait -> Menu.\n"<<std::flush;
                        } else if(maximum_error<=.35f) {
                            policy.Reset();running=true;velocity.fill(0);gamepad_drive.Clear();
                            inference_samples={obs};
                            std::cout<<G1_VISUAL_POLICY_NAME<<" policy ON.\n"<<std::flush;
                        } else std::cout<<"Policy start rejected: DDS joint "<<worst<<" error "<<maximum_error
                                       <<" rad exceeds 0.35; keep band enabled and reset with p / B.\n"<<std::flush;
                    }
                }
                if(running) {
                    const auto old_velocity=velocity;
                    if(action.planar_update) {velocity[0]=action.velocity[0];velocity[1]=action.velocity[1];}
                    if(action.yaw_update) velocity[2]=action.velocity[2];
                    bool changed=true;
                    switch(key) {
                        case 'w':case 'W':velocity[0]+=.1f;break;case 's':case 'S':velocity[0]-=.1f;break;
                        case 'a':case 'A':velocity[1]+=.1f;break;case 'd':case 'D':velocity[1]-=.1f;break;
                        case 'q':case 'Q':velocity[2]+=.1f;break;case 'e':case 'E':velocity[2]-=.1f;break;
                        case 'x':case 'X':velocity.fill(0);break;
                        default:changed=false;
                    }
                    if(changed) gamepad_drive.Clear();
                    velocity[0]=std::clamp(velocity[0],policy.command_min[0],policy.command_max[0]);
                    velocity[1]=std::clamp(velocity[1],policy.command_min[1],policy.command_max[1]);velocity[2]=std::clamp(velocity[2],policy.command_min[2],policy.command_max[2]);
                    if(changed||action.stop||((velocity!=old_velocity)&&now>=next_velocity_log)) {
                        std::cout<<"Velocity: "<<velocity[0]<<", "<<velocity[1]<<", "<<velocity[2]<<'\n'<<std::flush;
                        next_velocity_log=now+std::chrono::milliseconds(100);
                    }
                    const auto batch_started=Clock::now();
                    for(const auto& sample:inference_samples) {
                        if(sample.height<.5 || sample.gravity[2]>-std::cos(1.))
                            throw std::runtime_error("State body height/tilt protection triggered: height="+std::to_string(sample.height)+
                                "m (minimum 0.5m), tilt="+std::to_string(std::acos(std::clamp(-double(sample.gravity[2]),-1.,1.))*180./3.141592653589793)+
                                "deg (maximum 57.3deg), sim_time="+std::to_string(sample.time));
                        const auto before=Clock::now();target=policy.Step(sample,velocity);
                        const double milliseconds=std::chrono::duration<double,std::milli>(Clock::now()-before).count();
                        if(Clock::now()-batch_started>std::chrono::milliseconds(120))
                            throw std::runtime_error("Inference batch exceeded 120ms recovery window; stopping");
                        if(milliseconds>20)
                            std::cout<<"Inference delayed: "<<milliseconds<<"ms; retaining output and processing queued observations.\n"<<std::flush;
                        ++inference_frames;
                        if(now>=next_policy_log) {
                            const auto depth=g1_visual_policy::Policy::Preprocess(obs.depth);
                            const auto range=std::minmax_element(depth.begin(),depth.end());
                            std::cout<<"Policy running: frames="<<inference_frames<<", sim_time="<<obs.time
                                     <<", body_height="<<obs.height<<"m, tilt="
                                     <<std::acos(std::clamp(-double(obs.gravity[2]),-1.,1.))*180./3.141592653589793
                                     <<"deg, depth(normalized)="<<*range.first<<".."<<*range.second
                                     <<", inference="<<milliseconds<<"ms\n"<<std::flush;
                            next_policy_log=now+std::chrono::seconds(2);
                        }
                    }
                }
                if(active) {
                    if(!running) {
                        double u=std::clamp(std::chrono::duration<double>(now-reset_start).count()/reset_seconds,0.,1.);
                        double blend=u*u*(3-2*u);
                        for(int i=0;i<29;++i) target[i]=start[i]+blend*(neutral[i]-start[i]);
                        if(u==1 && !holding) {holding=true;std::cout<<"Holding training pose; press r to run policy.\n"<<std::flush;}
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
        clear();return result;
    } catch(const std::exception& e) {std::cerr<<argv[0]<<": "<<e.what()<<'\n';return 1;}
}
