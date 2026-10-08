#include "server/server.hpp"
#include <xgc2_ros_visualizer/bootstrap.hpp>
#include <ros/ros.h>
#include <exception>
#include <csignal>
#include <iostream>
#include <string>
#include <vector>
namespace {
volatile std::sig_atomic_t stop_requested=0;
void requestStop(int) { stop_requested=1; }
}
int main(int argc,char** argv) {
  try {
    std::string world_clock="wall";
    bool has_clock=false,has_remap=false;
    std::vector<std::string> arguments;
    for(int i=0;i<argc;++i) {
      const std::string argument=argv[i];
      const std::string prefix="_world_clock:=";
      if(argument.compare(0,prefix.size(),prefix)==0) {
        if(has_clock)throw std::invalid_argument("world_clock must occur once");
        has_clock=true;world_clock=argument.substr(prefix.size());
      }
      const std::string sim_prefix="_use_sim_time:=";
      if(argument.compare(0,sim_prefix.size(),sim_prefix)==0)throw std::invalid_argument("use world_clock to select the immutable server clock");
      if(argument.compare(0,15,"/use_sim_time:=")==0)has_remap=true;
      arguments.push_back(argument);
    }
    if(world_clock!="simulation"&&world_clock!="wall")throw std::invalid_argument("world_clock must be simulation or wall");
    arguments.push_back("_use_sim_time:="+std::string(world_clock=="simulation"?"true":"false"));
    if(!has_remap)arguments.push_back("/use_sim_time:=_use_sim_time");
    std::vector<char*> pointers;
    for(auto& argument:arguments)pointers.push_back(&argument[0]);
    int ros_argc=static_cast<int>(pointers.size());
    ros::init(ros_argc,pointers.data(),"xgc2_ros_visualizer",ros::init_options::NoSigintHandler);
    std::signal(SIGINT,requestStop);std::signal(SIGTERM,requestStop);
    ros::NodeHandle parameters("~");
    std::string socket,identity,rates_json;int workers;
    if(!parameters.getParam("socket_path",socket)||!parameters.getParam("server_instance_id",identity))
      throw std::invalid_argument("socket_path and server_instance_id are required startup parameters");
    parameters.param("callback_workers",workers,2);parameters.param<std::string>("rates_json",rates_json,"{}");
    if(workers<1||workers>32)throw std::invalid_argument("callback_workers must be within 1..32");
    xgc2_ros_visualizer::Server server(identity,static_cast<std::size_t>(workers),xgc2_ros_visualizer::parseRates(xgc2_ros_visualizer::parseJson(rates_json),false),&stop_requested);
    xgc2_ros_visualizer::RpcServer transport(socket,[&server](const std::string& method,const std::string& path,const Json::Value& body){return server.route(method,path,body);});
    std::string initial_file;
    parameters.param<std::string>("initial_instance_file",initial_file,"");
    if(!initial_file.empty()) {
      const auto initial=xgc2_ros_visualizer::readBootstrap(initial_file);
      const auto reply=server.route("PUT","/v1/instances/"+initial.instance_id,initial.request);
      if(reply.status!=200||!reply.body["ready"].asBool())
        throw std::invalid_argument("initial instance rejected: "+xgc2_ros_visualizer::jsonText(reply.body));
    }
    transport.run(server.stopping());server.stop();server.rethrowFailure();ros::shutdown();return 0;
  } catch(const std::exception& error) {std::cerr<<"visualizer server failed: "<<error.what()<<'\n';return 1;}
}
