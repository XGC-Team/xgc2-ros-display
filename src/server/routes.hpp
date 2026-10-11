#pragma once
#include "rpc.hpp"
namespace xgc2_ros_visualizer {
class Server;
// Wire mapping only. Native state and success conditions belong to Server.
RpcReply routeRpc(Server& server,const std::string& method,const std::string& path,const Json::Value& body);
// The readiness envelope of GET /v1/describe: identity, readiness and the facts
// that make this provider valid. Safe from any thread.
Json::Value describeService(const Server& server,const std::string& instance_id);
} // namespace xgc2_ros_visualizer
