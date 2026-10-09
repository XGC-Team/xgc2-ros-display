#include "server/server.hpp"
#include "server/routes.hpp"
#include "layout/rviz_layout.hpp"
#include <xgc2/xrpc/bootstrap.hpp>
#include <ros/ros.h>
#include <algorithm>
#include <exception>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
extern char** environ;
namespace {
volatile std::sig_atomic_t stop_requested=0;
void requestStop(int) { stop_requested=1; }
class OwnedFd {
 public:
  explicit OwnedFd(int value):value_(value) {
    if(value_<0)throw std::invalid_argument("unable to open the explicit native directory allocation");
  }
  ~OwnedFd(){::close(value_);}
  OwnedFd(const OwnedFd&)=delete;
  OwnedFd& operator=(const OwnedFd&)=delete;
  int get() const {return value_;}
 private:
  int value_;
};
xgc2::xrpc::DirectoryGrant allocatedGrant(const std::string& path,xgc2::xrpc::GrantPurpose purpose) {
  OwnedFd directory(::open(path.c_str(),O_RDONLY|O_DIRECTORY|O_CLOEXEC|O_NOFOLLOW));
  return xgc2::xrpc::DirectoryGrant::from_owned_directory(directory.get(),purpose);
}
std::string allocatedDirectory(const char* name) {
  const char* value=std::getenv(name);
  if(!value||!*value||value[0]!='/')
    throw std::invalid_argument(std::string(name)+" requires an explicit absolute supervisor allocation");
  return value;
}
}
int main(int argc,char** argv) {
  try {
    if(argc==2&&std::string(argv[1])=="--prepare-rviz") {
      std::string bytes;char buffer[65536];
      while(std::cin.read(buffer,sizeof(buffer))||std::cin.gcount()) {
        if(bytes.size()+static_cast<std::size_t>(std::cin.gcount())>8*1024*1024)
          throw std::invalid_argument("RViz preparation input exceeds 8 MiB");
        bytes.append(buffer,static_cast<std::size_t>(std::cin.gcount()));
      }
      if(!std::cin.eof())throw std::invalid_argument("unable to read RViz preparation input");
      Json::StreamWriterBuilder writer;writer["indentation"]="";
      std::cout<<Json::writeString(writer,xgc2_ros_visualizer::prepareRvizLayout(xgc2_ros_visualizer::parseJson(bytes)));
      return 0;
    }
    xgc2_ros_visualizer::RpcOptions rpc_options;
    rpc_options.instance_id=xgc2_ros_visualizer::RpcServer::newInstanceId();
    rpc_options.ros_home=allocatedDirectory("ROS_HOME");
    rpc_options.ros_log_dir=allocatedDirectory("ROS_LOG_DIR");
    // Only the composition root snapshots the reserved namespace. Neither
    // callbacks nor the SDK read ambient process settings per request.
    for(char** entry=environ;entry&&*entry;++entry) {
      const std::string item=*entry;
      if(item.compare(0,10,"XGC2_XRPC_")==0) {
        const auto equal=item.find('=');
        rpc_options.environment.emplace_back(item.substr(0,equal),equal==std::string::npos?"":item.substr(equal+1));
      }
    }
    std::string bootstrap_path;
    std::vector<std::string> arguments{argv[0]};
    std::vector<std::string> selected;
    bool has_remap=false;
    for(int i=1;i<argc;++i) {
      const std::string argument=argv[i];
      if(argument.compare(0,2,"--")==0) {
        if(std::find(selected.begin(),selected.end(),argument)!=selected.end())
          throw std::invalid_argument("startup option occurs more than once: "+argument);
        selected.push_back(argument);
        if(argument!="--bootstrap-input")
          throw std::invalid_argument("unknown startup option: "+argument);
        if(++i==argc)throw std::invalid_argument("startup option requires a value: "+argument);
        bootstrap_path=argv[i];
      } else {
        // Product bootstrap never reads ROS master parameters. Only standard
        // ROS remappings remain; old private product parameters fail explicitly.
        if(argument.empty()||argument.find(":=")==std::string::npos||
            (argument[0]=='_'&&argument.compare(0,2,"__")!=0))
          throw std::invalid_argument("product startup uses explicit --options; only ROS remappings may follow");
        if(argument.compare(0,2,"__")==0) {
          const auto name=argument.substr(0,argument.find(":="));
          const std::vector<std::string> allowed{"__name","__ns","__master","__ip","__hostname"};
          if(std::find(allowed.begin(),allowed.end(),name)==allowed.end())
            throw std::invalid_argument("unsupported ROS startup override; log writes use allocated ROS_LOG_DIR");
        }
        if(argument.compare(0,15,"/use_sim_time:=")==0)has_remap=true;
        arguments.push_back(argument);
      }
    }
    if(bootstrap_path.empty())throw std::invalid_argument("--bootstrap-input is required");
    // The shared loader owns bounded input, identity, credentials and grants.
    // This composition root only maps declared native ROS allocations to them.
    auto bootstrap=xgc2::xrpc::loadBootstrapInput(bootstrap_path);
    const auto& binding=bootstrap.binding();
    if(binding.service()!="xgc2.visualization"||binding.api_version()!="1"||
        binding.profile()!="http.v1"||binding.endpoint().kind!="unix"||
        binding.authentication()!="local_private")
      throw std::invalid_argument("this native provider requires local-private visualization HTTP v1");
    const auto application=bootstrap.application_json();
    if(!application)throw std::invalid_argument("native ROS allocation grant names are required");
    const auto native=xgc2_ros_visualizer::parseJson(std::string(*application));
    if(!native.isObject())throw std::invalid_argument("native application settings must be an object");
    const std::vector<std::string> native_fields{"rosHomeGrant","rosLogGrant","callbackWorkers","worldClock","rates"};
    const auto names=native.getMemberNames();
    if(!std::all_of(names.begin(),names.end(),[&](const std::string& name){
          return std::find(native_fields.begin(),native_fields.end(),name)!=native_fields.end();
        })||
        !native["rosHomeGrant"].isString()||!native["rosLogGrant"].isString()||
        native["rosHomeGrant"].asString().empty()||native["rosLogGrant"].asString().empty()||
        native["rosHomeGrant"]==native["rosLogGrant"]||binding.storage_grants().size()!=2)
      throw std::invalid_argument("native application requires distinct rosHomeGrant and rosLogGrant");
    const auto worker_value=native.get("callbackWorkers",Json::Value(2));
    if((worker_value.type()!=Json::intValue&&worker_value.type()!=Json::uintValue)||
        !worker_value.isUInt64()||worker_value.asUInt64()<1||worker_value.asUInt64()>32)
      throw std::invalid_argument("callbackWorkers must be an integer within 1..32");
    const auto workers=static_cast<std::size_t>(worker_value.asUInt64());
    const auto clock_value=native.get("worldClock",Json::Value("wall"));
    if(!clock_value.isString()||(clock_value.asString()!="simulation"&&clock_value.asString()!="wall"))
      throw std::invalid_argument("worldClock must be simulation or wall");
    const auto world_clock=clock_value.asString();
    const auto rates=xgc2_ros_visualizer::parseRates(native.get("rates",Json::Value(Json::objectValue)),false);
    const auto reference=binding.service_ref(rpc_options.instance_id);
    const auto socket=reference.endpoint.address;
    const auto runtime=bootstrap.resolve_runtime([&socket](const auto&,const auto&){
      const auto slash=socket.rfind('/');
      return allocatedGrant(slash?socket.substr(0,slash):"/",xgc2::xrpc::GrantPurpose::Runtime);
    });
    const auto storage=bootstrap.resolve_storage([&](const auto& handle,const auto&){
      if(handle.matches(native["rosHomeGrant"].asString()))
        return allocatedGrant(rpc_options.ros_home,xgc2::xrpc::GrantPurpose::Storage);
      if(handle.matches(native["rosLogGrant"].asString()))
        return allocatedGrant(rpc_options.ros_log_dir,xgc2::xrpc::GrantPurpose::Storage);
      throw std::invalid_argument("native storage grant is unresolved");
    });
    OwnedFd runtime_fd(runtime.duplicate_fd());
    rpc_options.retained_parent_fd=runtime_fd.get();
    rpc_options.target_id=reference.target_id;
    // The immutable provider clock is also the native ROS data clock. Private
    // ROS time initialization is implementation data, not a control API.
    arguments.push_back("_use_sim_time:="+std::string(world_clock=="simulation"?"true":"false"));
    if(!has_remap)arguments.push_back("/use_sim_time:=~use_sim_time");
    std::vector<char*> pointers;
    for(auto& argument:arguments)pointers.push_back(&argument[0]);
    int ros_argc=static_cast<int>(pointers.size());
    ros::init(ros_argc,pointers.data(),"xgc2_ros_visualizer",ros::init_options::NoSigintHandler);
    ros::NodeHandle ros_runtime; // Starts and retains the native ROS data graph.
    if(ros::Time::isSimTime()!=(world_clock=="simulation"))
      throw std::invalid_argument("ROS clock remap conflicts with the explicit immutable world clock");
    std::signal(SIGINT,requestStop);std::signal(SIGTERM,requestStop);
    xgc2_ros_visualizer::Server server(rpc_options.instance_id,workers,rates,&stop_requested);
    xgc2_ros_visualizer::RpcServer transport(socket,[&server](const std::string& method,const std::string& path,const Json::Value& body){return xgc2_ros_visualizer::routeRpc(server,method,path,body);},std::move(rpc_options),[&server]{
      try {server.stop();}
      catch(...) {ros::shutdown();throw;}
      ros::shutdown();
    });
    transport.run(server.stopping());server.rethrowFailure();return 0;
  } catch(const std::exception& error) {std::cerr<<"visualizer server failed: "<<std::string(error.what()).substr(0,512)<<'\n';return 1;}
}
