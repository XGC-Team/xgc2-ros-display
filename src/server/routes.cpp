#include "routes.hpp"
#include "server.hpp"
#include <set>
namespace xgc2_ros_visualizer {
namespace {
RpcReply error(int status,const std::string& message) {
  Json::Value body;body["ok"]=false;
  body["error"]["code"]=status==400||status==405?"invalid_argument":status==404?"not_found":status==409?"conflict":status==503?"unavailable":"internal";
  body["error"]["message"]=message;return {status,body};
}
}
RpcReply routeRpc(Server& server,const std::string& method,const std::string& path,const Json::Value& body) {
  if(server.stopping().load())return error(503,"server is stopping");
  try {
    if(path=="/v1/health"&&method=="GET")return {200,server.health()};
    if(path=="/v1/status"&&method=="GET")return {200,server.status()};
    if(path=="/v1/rates") {
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
    const std::string prefix="/v1/instances/";
    if(path.compare(0,prefix.size(),prefix)!=0)return error(404,"unknown route");
    const auto id=path.substr(prefix.size());
    if(method=="GET")return {200,server.instanceStatus(id)};
    if(method=="PUT")return {200,server.activateInstance(id,body)};
    if(method=="DELETE")return {200,server.removeInstance(id)};
    validateInstanceId(id);return error(405,"instance supports GET, PUT or DELETE");
  } catch(const ControlFailure& failure) {
    const int status=failure.code==ControlError::NotFound?404:failure.code==ControlError::Conflict?409:503;
    return error(status,failure.what());
  } catch(const std::invalid_argument& failure) {return error(400,failure.what());}
  catch(...) {return error(500,"internal native control failure");}
}
} // namespace xgc2_ros_visualizer
