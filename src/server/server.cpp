#include "server.hpp"
#include <algorithm>
#include <chrono>
#include <regex>
#include <limits>
#include <set>
#include <stdexcept>
namespace xgc2_ros_visualizer {
namespace {
RpcReply error(int status,const std::string& message) {Json::Value body;body["ok"]=false;body["error"]["code"]=status==400||status==405?"invalid_argument":status==404?"not_found":status==409?"conflict":status==503?"unavailable":"internal";body["error"]["message"]=message;return {status,body};}
RpcReply success(const Json::Value& body) {return {200,body};}
std::int64_t steadyNs() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
}
Server::Server(std::string identity,std::size_t workers,Rates rates,const volatile std::sig_atomic_t* signal_stop)
    : identity_(std::move(identity)),input_(workers),snapshot_(new std::vector<std::shared_ptr<Instance>>),rates_(new Rates(std::move(rates))),signal_stop_(signal_stop) {
  if(identity_.empty()||identity_.size()>128) throw std::invalid_argument("provider incarnation must contain 1..128 bytes");
  rates_applied_steady_ns_=steadyNs();
  publisher_=std::thread(&Server::publishLoop,this);
}
Server::~Server() {stop();}
void Server::wakePublisher() {
  std::lock_guard<std::mutex> lock(wait_mutex_);
  ++revision_;wake_.notify_all();
}
void Server::replaceSnapshot() {
  std::shared_ptr<std::vector<std::shared_ptr<Instance>>> next(new std::vector<std::shared_ptr<Instance>>);
  next->reserve(instances_.size());for(const auto& item:instances_)next->push_back(item.second);
  std::shared_ptr<const std::vector<std::shared_ptr<Instance>>> immutable=next;std::atomic_store(&snapshot_,immutable);
  wakePublisher();
}
void Server::publishLoop() {
  try {
    while(!stopping_.load()&&ros::ok()&&(!signal_stop_||!*signal_stop_)) {
      const auto revision=revision_.load();
      const auto started=std::chrono::steady_clock::now();auto rates=std::atomic_load(&rates_);auto members=std::atomic_load(&snapshot_);
      input_.rethrowFailure();
      const auto now=ros::Time::now();for(const auto& instance:*members)instance->tick(*rates,now);
      // One shared wait; membership/rates/Stop wake immediately. A bounded wall
      // heartbeat also observes signal flags safely, without notifying from a handler.
      const auto period=std::chrono::duration<double>(std::min(.1,1.0/rates->maximum()));
      std::unique_lock<std::mutex> wait(wait_mutex_);
      wake_.wait_until(wait,started+std::chrono::duration_cast<std::chrono::steady_clock::duration>(period),[&] {
        return stopping_.load()||revision_.load()!=revision||!ros::ok()||(signal_stop_&&*signal_stop_);
      });
    }
  } catch(...) {std::lock_guard<std::mutex> lock(failure_mutex_);failure_=std::current_exception();}
  stopping_.store(true);
}
void Server::stop() {
  stopping_.store(true);wakePublisher();if(publisher_.joinable())publisher_.join();
  for(auto& item:instances_)item.second->deactivate();
  instances_.clear();replaceSnapshot();input_.stop();
}
void Server::rethrowFailure() {std::lock_guard<std::mutex> lock(failure_mutex_);if(failure_)std::rethrow_exception(failure_);}
Json::Value Server::ratesStatus() const {
  Json::Value result;result["ok"]=true;result["rates"]=std::atomic_load(&rates_)->json();
  result["desiredRevision"]=Json::UInt64(rates_revision_);result["appliedRevision"]=Json::UInt64(rates_revision_);
  result["persistedRevision"]=Json::Value();result["state"]="applied";result["appliedAtSteadyNs"]=Json::Int64(rates_applied_steady_ns_);
  return result;
}
RpcReply Server::route(const std::string& method,const std::string& path,const Json::Value& body) {
  if(stopping_.load())return error(503,"server is stopping");
  try {
    if(path=="/v1/health"&&method=="GET") {
      Json::Value result;result["ok"]=true;result["instanceId"]=identity_;result["ready"]=true;result["callbackWorkers"]=Json::UInt64(input_.size());return success(result);
    }
    if(path=="/v1/rates") {
      if(method=="GET") return success(ratesStatus());
      if(method=="PUT") {
        if(!body.isObject())return error(400,"rate update must be an object");
        const auto names=body.getMemberNames();const std::set<std::string> required{"expectedRevision","rates"};
        if(std::set<std::string>(names.begin(),names.end())!=required||(body["expectedRevision"].type()!=Json::intValue&&body["expectedRevision"].type()!=Json::uintValue)||!body["expectedRevision"].isUInt64()||!body["expectedRevision"].asUInt64())
          return error(400,"rate update requires exactly positive expectedRevision and complete rates");
        if(body["expectedRevision"].asUInt64()!=rates_revision_)return error(409,"rate revision conflict");
        if(rates_revision_==std::numeric_limits<std::uint64_t>::max())return error(409,"rate revision exhausted");
        auto prepared=parseRates(body["rates"],true);std::shared_ptr<const Rates> replacement(new Rates(std::move(prepared)));std::atomic_store(&rates_,replacement);
        // This publication is the application boundary. A tick already holding
        // the old immutable snapshot finishes with that snapshot; subsequent
        // ticks load the new complete table, without partial channel updates.
        ++rates_revision_;rates_applied_steady_ns_=steadyNs();
        wakePublisher();
        return success(ratesStatus());
      }
      return error(405,"rates supports GET or PUT");
    }
    if(path=="/v1/status"&&method=="GET") {
      Json::Value result;result["ok"]=true;result["instanceId"]=identity_;result["ready"]=true;result["callbackWorkers"]=Json::UInt64(input_.size());result["publisherWorkers"]=1;result["instanceCount"]=Json::UInt64(instances_.size());result["rates"]=std::atomic_load(&rates_)->json();
      Json::UInt64 robots=0,descriptions=0,relays=0;result["instances"]=Json::Value(Json::arrayValue);
      for(const auto& item:instances_) {auto status=item.second->status();robots+=status["robotCount"].asUInt64();descriptions+=status["descriptionCount"].asUInt64();relays+=status["relayCount"].asUInt64();result["instances"].append(status);}
      result["robotCount"]=robots;result["descriptionCount"]=descriptions;result["relayCount"]=relays;return success(result);
    }
    const std::string prefix="/v1/instances/";
    if(path.compare(0,prefix.size(),prefix)!=0)return error(404,"unknown route");
    const std::string id=path.substr(prefix.size());static const std::regex identity("^[A-Za-z0-9_.-]{1,128}$");
    if(!std::regex_match(id,identity))return error(400,"invalid instance identity");
    auto found=instances_.find(id);
    if(method=="GET") {if(found==instances_.end())return error(404,"instance not found");return success(found->second->status());}
    if(method=="DELETE") {
      if(found!=instances_.end()) {auto instance=found->second;instances_.erase(found);replaceSnapshot();instance->deactivate();}
      Json::Value result;result["ok"]=true;result["id"]=id;result["removed"]=true;return success(result);
    }
    if(method!="PUT")return error(405,"instance supports GET, PUT or DELETE");
    if(found!=instances_.end()) {
      if(found->second->configuration()!=body)return error(409,"different instance content; DELETE explicitly before replacing its configuration");
      auto status=found->second->status();status["unchanged"]=true;return success(status);
    }
    if(instances_.size()>=64)return error(409,"server supports at most 64 instances");
    auto config=parseInstance(body);
    if(body["settings"].isMember("use_sim_time")&&body["settings"]["use_sim_time"].asBool()!=ros::Time::isSimTime())
      return error(409,"instance clock conflicts with the immutable server world_clock");
    std::shared_ptr<Instance> prepared(new Instance(id,std::move(config)));
    const auto claims=prepared->claims();
    for(const auto& item:instances_) {const auto occupied=item.second->claims();for(const auto& claim:claims)if(occupied.count(claim))return error(409,"instance output conflicts with "+item.first+": "+claim);}
    prepared->activate(input_);instances_.emplace(id,prepared);replaceSnapshot();return success(prepared->status());
  } catch(const std::invalid_argument& failure) {return error(400,failure.what());}
  catch(const std::exception& failure) {return error(500,failure.what());}
}
} // namespace xgc2_ros_visualizer
