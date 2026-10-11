#include "routes.hpp"
#include "server.hpp"
#include <set>
#include <vector>
namespace xgc2_ros_visualizer {
namespace {
RpcReply error(int status,const std::string& message) {
  Json::Value body;body["ok"]=false;
  body["error"]["code"]=status==400||status==405?"invalid_argument":status==404?"not_found":status==409?"conflict":status==503?"unavailable":"internal";
  body["error"]["message"]=message;return {status,body};
}
std::vector<std::string> segments(const std::string& path) {
  std::vector<std::string> parts;
  std::size_t begin=0;
  for(;;) {
    const auto end=path.find('/',begin);
    parts.push_back(path.substr(begin,end==std::string::npos?end:end-begin));
    if(end==std::string::npos)return parts;
    begin=end+1;
  }
}
RpcReply rates(Server& server,const std::string& method,const Json::Value& body) {
  if(method=="GET")return {200,server.ratesStatus()};
  if(method!="PUT")return error(405,"rates supports GET or PUT");
  if(!body.isObject())throw std::invalid_argument("rate update must be an object");
  const auto names=body.getMemberNames();const std::set<std::string> required{"expectedRevision","rates"};
  if(std::set<std::string>(names.begin(),names.end())!=required||
      (body["expectedRevision"].type()!=Json::intValue&&body["expectedRevision"].type()!=Json::uintValue)||
      !body["expectedRevision"].isUInt64()||!body["expectedRevision"].asUInt64())
    throw std::invalid_argument("rate update requires exactly positive expectedRevision and complete rates");
  return {200,server.replaceRates(body["expectedRevision"].asUInt64(),body["rates"])};
}
RpcReply instance(Server& server,const std::string& method,const std::string& id,const Json::Value& body) {
  if(method=="GET")return {200,server.instanceStatus(id)};
  if(method=="PUT")return {200,server.applyInstance(id,body)};
  if(method=="DELETE")return {200,server.removeInstance(id)};
  validateInstanceId(id);return error(405,"instance supports GET, PUT or DELETE");
}
RpcReply robot(Server& server,const std::string& method,const std::string& id,const std::string& robot_id,const Json::Value& body) {
  if(method=="GET")return {200,server.robotStatus(id,robot_id)};
  if(method=="PUT")return {200,server.applyRobot(id,robot_id,body)};
  if(method=="DELETE")return {200,server.removeRobot(id,robot_id)};
  return error(405,"robot supports GET, PUT or DELETE");
}
RpcReply robotRates(Server& server,const std::string& method,const std::string& id,const std::string& robot_id,const Json::Value& body) {
  if(method=="GET")return {200,server.robotRates(id,robot_id)};
  if(method=="PUT")return {200,server.applyRobotRates(id,robot_id,body)};
  return error(405,"robot rates supports GET or PUT");
}
}
Json::Value describeService(const Server& server,const std::string& instance_id) {
  Json::Value result(Json::objectValue);
  result["service"]="xgc2.visualization";
  result["api_version"]="1";
  result["instance_id"]=instance_id;
  result["ready"]=server.ready();
  result["facts"]=server.facts();
  return result;
}
RpcReply routeRpc(Server& server,const std::string& method,const std::string& path,const Json::Value& body) {
  if(server.stopping().load())return error(503,"server is stopping");
  try {
    if(path=="/v1/status"&&method=="GET")return {200,server.status()};
    if(path=="/v1/rates")return rates(server,method,body);
    const std::string prefix="/v1/instances/";
    if(path.compare(0,prefix.size(),prefix)!=0)return error(404,"unknown route");
    const auto parts=segments(path.substr(prefix.size()));
    validateInstanceId(parts[0]);
    if(parts.size()==1)return instance(server,method,parts[0],body);
    if(parts.size()>=3&&parts[1]=="robots") {
      validateRobotId(parts[2]);
      if(parts.size()==3)return robot(server,method,parts[0],parts[2],body);
      if(parts.size()==4&&parts[3]=="rates")return robotRates(server,method,parts[0],parts[2],body);
    }
    return error(404,"unknown route");
  } catch(const ControlFailure& failure) {
    const int status=failure.code==ControlError::NotFound?404:failure.code==ControlError::Conflict?409:503;
    return error(status,failure.what());
  } catch(const std::invalid_argument& failure) {return error(400,failure.what());}
  catch(...) {return error(500,"internal native control failure");}
}
} // namespace xgc2_ros_visualizer
