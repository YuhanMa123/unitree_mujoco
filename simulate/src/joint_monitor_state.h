#pragma once

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <locale>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace unitree_mujoco {

// Read-only DDS state, deliberately independent of MuJoCo and its physics lock.
class JointMonitorState {
public:
    using Clock = std::chrono::steady_clock;
    using Time = Clock::time_point;
    using Cmd = unitree_hg::msg::dds_::LowCmd_;
    using State = unitree_hg::msg::dds_::LowState_;
private:
    struct Command { double q{}, dq{}, kp{}, kd{}, tau{}; };
    std::mutex mutex_;
    std::array<Command,29> command_{}, target_{};
    std::array<bool,29> enable_{};
    Time started_, next_, capture_{}, command_at_{}, target_at_{}, enable_at_{};
    bool have_capture_=false, have_command_=false, have_target_=false, have_enable_=false;
    double period_;
    uint64_t sample_=0;
    std::string snapshot_="\"sample_id\":0,\"simulation_time\":0,\"joints\":[]";

    static bool Fresh(bool have, Time stamp, Time now) {
        return have && now-stamp < std::chrono::duration<double>(.5);
    }
    static std::ostringstream Json() {
        std::ostringstream out; out.imbue(std::locale::classic()); out<<std::setprecision(10); return out;
    }
    template<class Values> static void Vector(std::ostream& out,const Values& values) {
        for(auto v:values) if(!std::isfinite(v)) {out<<"null";return;}
        out<<'[';bool first=true;
        for(auto v:values) {if(!first)out<<',';first=false;out<<v;}out<<']';
    }
    static void Imu(std::ostream& out,const State& msg) {
        const auto& imu=msg.imu_state();const auto& q=imu.quaternion();
        out<<"{\"quaternion\":";Vector(out,q);out<<",\"rpy\":";
        double norm=0;for(auto v:q)norm+=double(v)*v;norm=std::sqrt(norm);
        if(!std::isfinite(norm)||norm<=1e-9) out<<"null";
        else {
            const double w=q[0]/norm,x=q[1]/norm,y=q[2]/norm,z=q[3]/norm;
            Vector(out,std::array<double,3>{std::atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
                std::asin(std::clamp(2*(w*y-z*x),-1.,1.)),std::atan2(2*(w*z+x*y),1-2*(y*y+z*z))});
        }
        out<<",\"gyroscope\":";Vector(out,imu.gyroscope());
        out<<",\"accelerometer\":";Vector(out,imu.accelerometer());out<<'}';
    }
    void Store(const Cmd& msg,bool target,Time now) {
        std::array<Command,29> values;
        for(int i=0;i<29;++i) {
            const auto& m=msg.motor_cmd()[i];values[i]={m.q(),m.dq(),m.kp(),m.kd(),m.tau()};
            for(double v:{double(m.q()),double(m.dq()),double(m.kp()),double(m.kd()),double(m.tau())})
                if(!std::isfinite(v))return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if(target){target_=values;target_at_=now;have_target_=true;}
        else{command_=values;command_at_=now;have_command_=true;}
    }
public:
    explicit JointMonitorState(double rate=50,Time now=Clock::now()):started_(now),next_(now),period_(1/rate) {
        if(!std::isfinite(rate)||rate<=0)throw std::runtime_error("Monitor rate must be positive and finite");
    }
    static const std::array<std::string,29>& Names() {
        static const auto names=[] {
            std::array<std::string,29> result;size_t i=0;
            for(auto side:{"left","right"})for(auto joint:{"hip_pitch","hip_roll","hip_yaw","knee","ankle_pitch","ankle_roll"})
                result[i++]=std::string(side)+"_"+joint+"_joint";
            for(auto joint:{"yaw","roll","pitch"})result[i++]=std::string("waist_")+joint+"_joint";
            for(auto side:{"left","right"})for(auto joint:{"shoulder_pitch","shoulder_roll","shoulder_yaw","elbow","wrist_roll","wrist_pitch","wrist_yaw"})
                result[i++]=std::string(side)+"_"+joint+"_joint";
            return result;
        }();return names;
    }
    void OnCommand(const Cmd& msg,Time now=Clock::now()){Store(msg,false,now);}
    void OnTarget(const Cmd& msg,Time now=Clock::now()){Store(msg,true,now);}
    void OnEnable(const Cmd& msg,Time now=Clock::now()) {
        std::array<bool,29> values;
        for(int i=0;i<29;++i){int mode=msg.motor_cmd()[i].mode();if(mode!=0&&mode!=1)return;values[i]=mode==1;}
        std::lock_guard<std::mutex> lock(mutex_);enable_=values;enable_at_=now;have_enable_=true;
    }
    void OnState(const State& msg,Time now=Clock::now()) {
        if(msg.mode_machine()!=0&&msg.mode_machine()!=5)return;
        for(int i=0;i<29;++i){const auto& m=msg.motor_state()[i];
            if(!std::isfinite(m.q())||!std::isfinite(m.dq())||!std::isfinite(m.tau_est()))return;}
        std::lock_guard<std::mutex> lock(mutex_);if(now<next_)return;
        const bool cf=Fresh(have_command_,command_at_,now),tf=Fresh(have_target_,target_at_,now),ef=Fresh(have_enable_,enable_at_,now);
        auto out=Json();out<<std::boolalpha<<"\"sample_id\":"<<++sample_<<",\"simulation_time\":"
            <<std::chrono::duration<double>(now-started_).count()<<",\"output_enabled\":";
        if(ef)out<<enable_[0];else out<<"null";
        out<<",\"imu\":";Imu(out,msg);out<<",\"joints\":[";
        for(int i=0;i<29;++i) {
            const auto& m=msg.motor_state()[i];const auto& desired=tf?target_[i]:command_[i];
            const auto& actual=command_[i];if(i)out<<',';
            out<<"{\"index\":"<<i<<",\"name\":\""<<Names()[i]<<"\",\"q\":"<<m.q()<<",\"dq\":"<<m.dq()
                <<",\"tau\":"<<m.tau_est()<<",\"q_target\":"<<desired.q<<",\"dq_target\":"<<desired.dq
                <<",\"kp\":"<<desired.kp<<",\"kd\":"<<desired.kd<<",\"error\":"<<desired.q-m.q()
                <<",\"target_available\":"<<(tf||cf)<<",\"enabled\":";
            if(ef)out<<enable_[i];else out<<"null";
            out<<",\"active\":"<<(cf&&(actual.kp!=0||actual.kd!=0||actual.tau!=0))<<'}';
        }
        out<<']';snapshot_=out.str();capture_=now;have_capture_=true;
        next_+=std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(
            (std::floor(std::chrono::duration<double>(now-next_).count()/period_)+1)*period_));
    }
    std::string Snapshot(Time now=Clock::now()) {
        std::lock_guard<std::mutex> lock(mutex_);auto out=Json();out<<'{'+snapshot_<<",\"age_ms\":";
        if(have_capture_)out<<std::chrono::duration<double,std::milli>(now-capture_).count();else out<<"null";
        out<<'}';return out.str();
    }
};
} // namespace unitree_mujoco
