// Real C++ MuJoCo -> UDP -> g1_parkour ONNX -> DDS -> per-step PD.
// Built only by run_tests.py in its automatically removed temporary project.
#include "unitree_sdk2_bridge.h"
#include "parkour_observation.h"
#include "elastic_band.h"
#include <sys/wait.h>
#include <cstdio>
#include <csignal>
#include <chrono>
#include <thread>

using Clock = std::chrono::steady_clock;
void Require(bool value, const std::string& message) {
    if (!value) throw std::runtime_error(message);
}

class Controller {
    pid_t pid_ = -1;
    int input_ = -1, status_ = 0;
    FILE* log_ = nullptr;
    bool finished_ = false;
public:
    Controller(const std::string& executable, int port, int domain, const std::string& config, const std::string& models) {
        log_ = std::tmpfile();
        Require(log_ != nullptr, "Cannot create temporary controller log");
        int channel[2]; Require(pipe(channel)==0,"Cannot create controller input pipe");
        const auto port_text=std::to_string(port), domain_text=std::to_string(domain);
        pid_=fork();
        if(pid_==0) {
            dup2(channel[0],STDIN_FILENO);dup2(fileno(log_),STDOUT_FILENO);dup2(fileno(log_),STDERR_FILENO);
            close(channel[0]);close(channel[1]);
            std::vector<std::string> arguments{executable,"--port",port_text,"--domain",domain_text,
                "--reset-seconds","0.5","--no-gamepad","--config",config};
            if(!models.empty()) {arguments.push_back("--model-dir");arguments.push_back(models);}
            std::vector<char*> raw;
            for(auto& argument:arguments) raw.push_back(argument.data());
            raw.push_back(nullptr);execv(executable.c_str(),raw.data());
            _exit(127);
        }
        close(channel[0]);input_=channel[1];Require(pid_>0,"Cannot start controller");
    }
    ~Controller() {
        if(input_>=0) close(input_);
        if(pid_>0 && !finished_) {
            kill(pid_,SIGCONT);
            kill(pid_,SIGTERM);
            const auto deadline=Clock::now()+std::chrono::seconds(3);
            while(!Done() && Clock::now()<deadline) std::this_thread::sleep_for(std::chrono::milliseconds(10));
            if(!finished_) {kill(pid_,SIGKILL);waitpid(pid_,&status_,0);}
        }
        if(log_) fclose(log_);
    }
    bool Done() {
        if(!finished_ && waitpid(pid_,&status_,WNOHANG)==pid_) finished_=true;
        return finished_;
    }
    void Key(char key) {Require(write(input_,&key,1)==1,"Controller input failed");}
    void Pause(bool value) {Require(kill(pid_,value?SIGSTOP:SIGCONT)==0,"Cannot inject controller scheduling delay");}
    int ExitCode() {return WIFEXITED(status_)?WEXITSTATUS(status_):-1;}
    std::string Output() {
        fflush(log_);fseek(log_,0,SEEK_SET);
        std::string result;char buffer[4096];std::size_t n;
        while((n=fread(buffer,1,sizeof(buffer),log_))) result.append(buffer,n);
        return result;
    }
};

// Fault injection uses real UDP; production sender and controller are unchanged.
class ObservationRelay {
    int fd_=-1;
    sockaddr_in destination_{};
public:
    int port=0,discarded=0;
    explicit ObservationRelay(int destination) {
        fd_=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK,0);Require(fd_>=0,"Cannot create relay");
        sockaddr_in address{};address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        address.sin_port=htons(destination+1000);
        if(bind(fd_,reinterpret_cast<sockaddr*>(&address),sizeof(address))) {close(fd_);throw std::runtime_error("Cannot bind relay");}
        port=destination+1000;destination_=address;destination_.sin_port=htons(destination);
    }
    ~ObservationRelay() {close(fd_);}
    void Pump(double elapsed) {
        std::array<unsigned char,65536> bytes{};
        for(int i=0;i<64;++i) {
            const auto n=recv(fd_,bytes.data(),bytes.size(),0);
            if(n<0) {if(errno==EAGAIN||errno==EWOULDBLOCK) break;throw std::runtime_error("Relay receive failed");}
            if((elapsed>=5.2 && elapsed<5.24)||(elapsed>=6.2 && elapsed<6.26)) {++discarded;continue;}
            Require(sendto(fd_,bytes.data(),n,0,reinterpret_cast<sockaddr*>(&destination_),sizeof(destination_))==n,"Relay send failed");
        }
    }
};

std::size_t Count(const std::string& text, const std::string& pattern) {
    std::size_t count=0,offset=0;
    while((offset=text.find(pattern,offset))!=std::string::npos) {++count;offset+=pattern.size();}
    return count;
}

int main(int argc,char** argv) {
    std::signal(SIGPIPE,SIG_IGN);
    try {
        Require(argc>=4 && argc<=7,"Expected root, controller, mode, optional config, scene and model directory");
        const std::string root=argv[1],mode=argv[3];
        const std::string config=argc>=5?argv[4]:root+"/example/exported/g1_parkour/g1_torsobase_cfg.json";
        const auto cfg=YAML::LoadFile(config);
        const double physics_dt=cfg["physics_dt"].as<double>();
        const std::string label=cfg["policy_type"] ? "CA PE AMP VAE" : "Parkour";
        Require(mode=="fixed"||mode=="ground"||mode=="stale"||mode=="reset"||mode=="band"||mode=="terrain"||mode=="ground-reset"||mode=="jitter"||mode=="walk","Unknown smoke mode");
        const bool walk=mode=="walk";
        const bool jitter=mode=="jitter";
        const bool ground_reset=mode=="ground-reset";
        const bool ground=mode=="ground"||ground_reset||jitter||walk,stale=mode=="stale";
        const bool reset=mode=="reset",band_mode=mode=="band"||mode=="terrain";
        ElasticBand band;
        const bool terrain=mode=="terrain";
        const int domain=walk?203:jitter?202:ground_reset?201:terrain?200:(band_mode?199:ground?196:(stale?197:(reset?198:195)));
        const int probe=socket(AF_INET,SOCK_DGRAM,0);Require(probe>=0,"Cannot create UDP probe");
        sockaddr_in address{};address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        // Keep observation ports outside the DDS domain participant range.
        address.sin_port=htons(18000+domain);
        Require(bind(probe,reinterpret_cast<sockaddr*>(&address),sizeof(address))==0,"Cannot reserve UDP port");
        socklen_t length=sizeof(address);
        Require(getsockname(probe,reinterpret_cast<sockaddr*>(&address),&length)==0,"Cannot query UDP port");
        const int port=ntohs(address.sin_port);close(probe);
        param::config.load_from_yaml(root+"/simulate/config.yaml");
        param::config.robot="g1";param::config.robot_scene=band_mode?"scene_29dof.xml":"scene_29dof_flat.xml";
        param::config.print_scene_information=0;param::config.use_joystick=0;
        char error[2048]{};
        std::unique_ptr<mjModel,decltype(&mj_deleteModel)> model(
            mj_loadXML(argc>=6 && argv[5][0]?argv[5]:(root+"/unitree_robots/g1/"+param::config.robot_scene.string()).c_str(),nullptr,error,sizeof(error)),mj_deleteModel);
        Require(bool(model),error);model->opt.timestep=physics_dt;
        std::unique_ptr<mjData,decltype(&mj_deleteData)> data(mj_makeData(model.get()),mj_deleteData);
        mj_forward(model.get(),data.get());
        // fork/exec before initializing DDS worker threads in the parent.
        Controller controller(argv[2],port,domain,config,argc==7?argv[6]:"");
        unitree::robot::ChannelFactory::Instance()->Init(domain,"lo");
        G1Bridge bridge(model.get(),data.get());
        std::unique_ptr<ObservationRelay> relay;
        if(jitter) relay=std::make_unique<ObservationRelay>(port);
        unitree_mujoco::ParkourObservationPublisher observations(model.get(),data.get(),
            config,relay?relay->port:port);
        std::array<double,29> neutral{};
        for(int i=0;i<29;++i) {
            const int joint=mj_name2id(model.get(),mjOBJ_JOINT,cfg["joint_names"][i].as<std::string>().c_str());
            for(int a=0;a<29;++a)
                if(model->actuator_trnid[2*a]==joint)
                    neutral[a]=cfg["joint_sign"][i].as<double>()*cfg["default_joint_pos"][i].as<double>();
        }
        std::vector<std::pair<double,char>> events={{1.,'p'},{1.1,'r'},{1.2,'9'},{1.3,'p'},
            {2.6,'r'},{2.8,'w'},{2.9,'q'},{3.1,'x'},{3.3,'p'},{4.5,'r'},{5.,' '}};
        if(ground) events={{1.2,'9'},{1.3,'p'},{2.6,'r'},{10.7,' '}};
        if(walk) events={{1.2,'9'},{1.3,'p'},{2.6,'r'},{3.5,'w'},{3.6,'w'},{3.7,'w'},{3.8,'w'},{3.9,'w'},{9.,'x'},{10.7,' '}};
        if(ground_reset) events={{.8,'#'},{1.2,'9'},{1.3,'p'},{2.6,'r'},{5.,'#'},{8.,'#'},{10.7,' '}};
        if(band_mode) events={{1.2,'9'},{1.3,'p'},{4.,'r'},{16.,' '}};
        if(terrain) events={{1.2,'9'},{1.3,'p'},{4.,'r'},{6.2,'w'},{6.3,'w'},{6.4,'w'},{6.5,'w'},{6.6,'w'},{24.,' '}};
        if(reset) events={{.8,'#'},{1.2,'9'},{1.3,'p'},{2.6,'r'},
            {2.8,'w'},{3.2,'#'},{3.6,'w'},{4.,'#'},{4.4,'q'},{5.,' '}};
        const auto start=Clock::now();auto next=start;
        std::size_t event=0;bool landed=false,policy_changed=false,paused=false,resumed=false;
        double minimum_height=10,maximum_drift=0,ground_duration=0;
        double next_motion_log=3.,sum_vx=0,sum_vy=0,sum_wz=0,motion_duration=0;
        double sum_heading_vx=0,sum_heading_vy=0,sum_world_wz=0;
        double sum_pelvis_pitch=0,sum_torso_pitch=0,sum_waist_pitch=0,sum_waist_target=0;
        const int pelvis_body=mj_name2id(model.get(),mjOBJ_BODY,"pelvis");
        const int torso_body=mj_name2id(model.get(),mjOBJ_BODY,"torso_link");
        const int waist_joint=mj_name2id(model.get(),mjOBJ_JOINT,"waist_pitch_joint");
        auto pitch=[&](int body) {
            const auto* rotation=data->xmat+9*body;
            return std::atan2(-rotation[6],std::hypot(rotation[7],rotation[8]))*180./3.141592653589793;
        };
        const int state_body=mj_name2id(model.get(),mjOBJ_BODY,
            cfg["state_body"]?cfg["state_body"].as<std::string>().c_str():"torso_link");
        Require(state_body>=0,"Missing motion telemetry body");
        try {
            while(!controller.Done() && Clock::now()-start<std::chrono::seconds(28)) {
                const double elapsed=std::chrono::duration<double>(Clock::now()-start).count();
                if(jitter && !paused && elapsed>=4.) {controller.Pause(true);paused=true;}
                if(jitter && !resumed && elapsed>=4.08) {controller.Pause(false);resumed=true;}
                while(event<events.size() && elapsed>=events[event].first) {
                    const char key=events[event++].second;
                    if(key=='#') {unitree_mujoco::ResetSimulation(model.get(),data.get());observations.ResetPose();}
                    else controller.Key(key);
                }
                if(!band_mode && (!ground || elapsed<2.7)) {
                    data->qpos[0]=data->qpos[1]=0;data->qpos[2]=1.2;
                    data->qpos[3]=1;data->qpos[4]=data->qpos[5]=data->qpos[6]=0;
                    for(int i=0;i<6;++i) data->qvel[i]=0;
                } else if(!band_mode && !landed) {
                    data->qpos[2]=.78;for(int i=0;i<6;++i) data->qvel[i]=0;
                    landed=true;
                }
                observations.SetMotorControlActive(bridge.ApplyMotorControl());
                mj_forward(model.get(),data.get());
                if(band_mode) {
                    // Lower from the actual startup suspension, then release after landing.
                    if(elapsed>2.8 && elapsed<6.) band.length_+=.5*physics_dt;
                    if(elapsed>=6.) band.enable_=false;
                    band.Apply(model.get(),data.get());
                }
                bridge.run();
                if(!stale || elapsed<4.7) observations.Update();
                if(relay) relay->Pump(elapsed);
                observations.LimitTorque();
                {
                    std::lock_guard<std::mutex> lock(bridge.lowcmd->mutex_);
                    for(int i=0;i<29;++i) {
                        const auto& motor=bridge.lowcmd->msg_.motor_cmd()[i];
                        if(elapsed<1.3) Require(motor.kp()==0 && motor.kd()==0,"Commands before enable/reset");
                        if(elapsed>(band_mode?4.1:2.7) && elapsed<(band_mode?6.:3.2) && std::abs(motor.q()-neutral[i])>.01) policy_changed=true;
                    }
                }
                mj_step(model.get(),data.get());
                if((ground && landed) || (band_mode && elapsed>6.)) {
                    ground_duration+=model->opt.timestep;
                    minimum_height=std::min(minimum_height,double(data->xpos[3*model->jnt_bodyid[0]+2]));
                    maximum_drift=std::max(maximum_drift,std::hypot(data->qpos[0],data->qpos[1]));
                    mjtNum velocity[6];mj_objectVelocity(model.get(),data.get(),mjOBJ_XBODY,state_body,velocity,1);
                    // IsaacLab root_lin_vel_w is the root body's COM velocity.
                    // BODY + world axes gets that point without using inertial axes.
                    mjtNum world_velocity[6];mj_objectVelocity(model.get(),data.get(),mjOBJ_BODY,state_body,world_velocity,0);
                    const auto* rotation=data->xmat+9*state_body;
                    mju_mulMatTVec3(velocity+3,rotation,world_velocity+3);
                    const double heading=std::atan2(rotation[3],rotation[0]);
                    const double c=std::cos(heading),s=std::sin(heading);
                    const double heading_vx=c*world_velocity[3]+s*world_velocity[4];
                    const double heading_vy=-s*world_velocity[3]+c*world_velocity[4];
                    // Measure the settled command interval, excluding landing and stopping.
                    if(elapsed>=4.5 && elapsed<9.) {
                        sum_vx+=velocity[3]*physics_dt;sum_vy+=velocity[4]*physics_dt;
                        sum_wz+=velocity[2]*physics_dt;motion_duration+=physics_dt;
                        sum_heading_vx+=heading_vx*physics_dt;sum_heading_vy+=heading_vy*physics_dt;
                        sum_world_wz+=world_velocity[2]*physics_dt;
                        sum_pelvis_pitch+=pitch(pelvis_body)*physics_dt;
                        sum_torso_pitch+=pitch(torso_body)*physics_dt;
                        sum_waist_pitch+=data->qpos[model->jnt_qposadr[waist_joint]]*physics_dt;
                        std::lock_guard<std::mutex> command_lock(bridge.lowcmd->mutex_);
                        sum_waist_target+=bridge.lowcmd->msg_.motor_cmd()[14].q()*physics_dt;
                    }
                    if(elapsed>=next_motion_log) {
                        std::cout<<"Motion: t="<<data->time<<"s, world_xy="<<data->qpos[0]<<","<<data->qpos[1]
                                 <<"m, yaw="<<std::atan2(rotation[3],rotation[0])*180./3.141592653589793
                                 <<"deg, body_vxy="<<velocity[3]<<","<<velocity[4]
                                 <<"m/s, body_wz="<<velocity[2]<<"rad/s, heading_vxy="<<heading_vx<<","<<heading_vy
                                 <<"m/s, world_wz="<<world_velocity[2]<<"rad/s\n";
                        std::cout<<"Pitch: pelvis="<<pitch(pelvis_body)<<"deg, torso="<<pitch(torso_body)
                                 <<"deg, waist="<<data->qpos[model->jnt_qposadr[waist_joint]]<<"rad\n";
                        next_motion_log+=1.;
                    }
                }
                for(int i=0;i<model->nq;++i) Require(std::isfinite(data->qpos[i]),"Non-finite simulated state");
                for(int i=0;i<29;++i) Require(std::isfinite(data->ctrl[i]),"Non-finite simulated torque");
                next+=std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(physics_dt));std::this_thread::sleep_until(next);
            }
            Require(controller.Done(),"Controller failed to exit");
            const auto output=controller.Output();std::cout<<output;
            Require(controller.ExitCode()==(stale?1:0),"Unexpected controller exit code");
            Require(Count(output,label+" policy ON.")==((ground||reset||band_mode)?1:2),"Policy entry/reentry failed");
            if(!ground && !reset && !band_mode)
                Require(output.find("Policy start rejected: initialize joints first")!=std::string::npos,
                        "Premature policy start must explain the missing initialization");
            if(reset) {
                Require(Count(output,"Simulation reset received; policy OFF;")==1,"Reset before enable started policy");
                Require(Count(output,"Simulation reset received; policy ON;")==2,"Reset failed to preserve running policy");
                Require(Count(output,"Velocity: 0.1, 0, 0")==2,"Commands failed after reset");
                Require(Count(output,"Velocity: 0, 0, 0.1")==1,"Reset did not clear velocity");
            }
            if(ground_reset) {
                Require(Count(output,"Simulation reset received; policy OFF;")==1,"Ground reset enabled policy prematurely");
                Require(Count(output,"Simulation reset received; policy ON;")==2,"Ground reset failed to preserve policy");
            }
            Require(policy_changed,"Policy failed to produce new joint targets");
            if(jitter) {
                Require(relay->discarded>=4,"Fault injection failed to discard packets");
                Require(Count(output,"Observation gap recovered:")>=2,"Missing packets were not recovered");
                Require(Count(output,"Policy running:")>=4,"Policy failed to continue after jitter");
                std::cout<<"Recovered after 80ms scheduler pause and "<<relay->discarded<<" deliberately dropped UDP packets\n";
            }
            if(stale) Require(output.find("stale; stopping")!=std::string::npos,"Stale watchdog did not fire");
            if(band_mode) {
                std::cout<<"Terrain/band: minimum base="<<minimum_height<<"m, XY distance="<<maximum_drift<<"m\n";
                Require(minimum_height>.65,"Band release/terrain standing failed");
                if(terrain) Require(maximum_drift>1.,"Terrain forward command did not move robot");
            }
            if(ground) {
                if(motion_duration>0)
                    std::cout<<"Mean motion (4.5..9s): body_vxy="<<sum_vx/motion_duration<<","<<sum_vy/motion_duration
                             <<"m/s, body_wz="<<sum_wz/motion_duration<<"rad/s\n";
                if(motion_duration>0)
                    std::cout<<"Mean heading-frame motion (4.5..9s): heading_vxy="<<sum_heading_vx/motion_duration
                             <<","<<sum_heading_vy/motion_duration<<"m/s, world_wz="<<sum_world_wz/motion_duration<<"rad/s\n";
                if(motion_duration>0)
                    std::cout<<"Mean pitch (4.5..9s; positive=forward): pelvis="<<sum_pelvis_pitch/motion_duration
                             <<"deg, torso="<<sum_torso_pitch/motion_duration<<"deg, waist="<<sum_waist_pitch/motion_duration
                             <<"rad, waist_target="<<sum_waist_target/motion_duration<<"rad\n";
                std::cout<<"Ground: "<<ground_duration<<"s, minimum base="<<minimum_height
                         <<"m, maximum XY drift="<<maximum_drift<<"m\n";
                Require(minimum_height>.65 && (walk?maximum_drift>1.:maximum_drift<.5),
                        walk?"Forward-command walking failed":"Zero-command standing failed");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            std::lock_guard<std::mutex> lock(bridge.lowcmd->mutex_);
            for(int i=0;i<29;++i) {
                const auto& motor=bridge.lowcmd->msg_.motor_cmd()[i];
                Require(motor.kp()==0 && motor.kd()==0 && motor.tau()==0,"Output not cleared on exit");
            }
            std::cout<<"PASS C++ MuJoCo / depth UDP / C++ ONNX / DDS / PD: "<<mode<<'\n';
        } catch(...) {
            std::cerr<<controller.Output();throw;
        }
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
