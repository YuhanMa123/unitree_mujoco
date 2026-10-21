#include "joint_monitor_http.h"
#include "joint_monitor_page.h"
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <boost/program_options.hpp>
#include <iostream>

namespace { volatile std::sig_atomic_t stopping=0; void Stop(int){stopping=1;} }

int main(int argc,char** argv) {
    namespace po=boost::program_options;
    try {
        int domain=1,port=8765;double rate=50;std::string host="127.0.0.1",interface="lo";
        po::options_description options("G1 29DOF read-only web joint monitor");
        options.add_options()("help,h","Show usage")
            ("domain",po::value<int>(&domain)->default_value(domain),"DDS simulation domain")
            ("interface",po::value<std::string>(&interface)->default_value(interface),"DDS network interface")
            ("host",po::value<std::string>(&host)->default_value(host),"HTTP IPv4 bind address")
            ("port",po::value<int>(&port)->default_value(port),"HTTP port")
            ("rate",po::value<double>(&rate)->default_value(rate),"Maximum sampling and SSE rate in Hz");
        po::variables_map values;po::store(po::parse_command_line(argc,argv,options),values);po::notify(values);
        if(values.count("help")){std::cout<<options<<'\n';return 0;}
        if(domain<=0||port<1||port>65535||!std::isfinite(rate)||rate<=0||rate>1000)
            throw std::runtime_error("Use domain > 0, port 1..65535 and rate > 0 and <= 1000 Hz");
        std::signal(SIGINT,Stop);std::signal(SIGTERM,Stop);
        unitree_mujoco::JointMonitorState state(rate);
        unitree_mujoco::JointMonitorHttp http(state,host,port,rate,kJointMonitorPage);
        unitree::robot::ChannelFactory::Instance()->Init(domain,interface);
        // Subscribers are destroyed before callback state and the HTTP server.
        using Cmd=unitree_mujoco::JointMonitorState::Cmd;using State=unitree_mujoco::JointMonitorState::State;
        unitree::robot::ChannelSubscriber<State> feedback("rt/lowstate");
        unitree::robot::ChannelSubscriber<Cmd> command("rt/lowcmd"),target("rt/g1/reset_target"),enable("rt/g1/reset_enable");
        feedback.InitChannel([&](const void* msg){state.OnState(*static_cast<const State*>(msg));},1);
        command.InitChannel([&](const void* msg){state.OnCommand(*static_cast<const Cmd*>(msg));},1);
        target.InitChannel([&](const void* msg){state.OnTarget(*static_cast<const Cmd*>(msg));},1);
        enable.InitChannel([&](const void* msg){state.OnEnable(*static_cast<const Cmd*>(msg));},1);
        std::cout<<"Joint monitor: http://"<<host<<':'<<port<<"\nWaiting for G1 29DOF DDS data: domain="
            <<domain<<", interface="<<interface<<". Ctrl+C to exit.\n"<<std::flush;
        http.Run(stopping);
        return 0;
    }catch(const std::exception& error){std::cerr<<"Joint monitor: "<<error.what()<<'\n';return 1;}
}
