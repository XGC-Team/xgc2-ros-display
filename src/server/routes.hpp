#pragma once
#include "rpc.hpp"
namespace xgc2_ros_visualizer {
class Server;
// Wire mapping only. Native state and success conditions belong to Server.
RpcReply routeRpc(Server& server,const std::string& method,const std::string& path,const Json::Value& body);
} // namespace xgc2_ros_visualizer
