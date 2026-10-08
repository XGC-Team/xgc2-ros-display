#include "server.hpp"
#include <algorithm>
#include <chrono>
#include <regex>
#include <limits>
#include <stdexcept>
namespace xgc2_ros_visualizer {
namespace {
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
  input_.stop();
  for(auto& item:instances_)item.second->deactivate();
  instances_.clear();replaceSnapshot();
}
void Server::rethrowFailure() {std::lock_guard<std::mutex> lock(failure_mutex_);if(failure_)std::rethrow_exception(failure_);}
Json::Value Server::ratesStatus() const {
  Json::Value result;result["ok"]=true;result["rates"]=std::atomic_load(&rates_)->json();
  result["desiredRevision"]=Json::UInt64(rates_revision_);result["appliedRevision"]=Json::UInt64(rates_revision_);
  result["persistedRevision"]=Json::Value();result["state"]="applied";result["appliedAtSteadyNs"]=Json::Int64(rates_applied_steady_ns_);
  return result;
}
void validateInstanceId(const std::string& id) {
  static const std::regex identity("^[A-Za-z0-9_.-]{1,128}$");
  if(!std::regex_match(id,identity))throw std::invalid_argument("invalid instance identity");
}
void Server::requireRunning() const {
  if(stopping_.load())throw ControlFailure(ControlError::Unavailable,"server is stopping");
}
Json::Value Server::health() const {
  Json::Value result;result["ok"]=true;result["instanceId"]=identity_;
  result["ready"]=!stopping_.load();result["callbackWorkers"]=Json::UInt64(input_.size());return result;
}
Json::Value Server::status() const {
  auto result=health();result["publisherWorkers"]=1;result["instanceCount"]=Json::UInt64(instances_.size());result["rates"]=std::atomic_load(&rates_)->json();
  Json::UInt64 robots=0,descriptions=0,relays=0;result["instances"]=Json::Value(Json::arrayValue);
  for(const auto& item:instances_) {auto status=item.second->status();robots+=status["robotCount"].asUInt64();descriptions+=status["descriptionCount"].asUInt64();relays+=status["relayCount"].asUInt64();result["instances"].append(status);}
  result["robotCount"]=robots;result["descriptionCount"]=descriptions;result["relayCount"]=relays;return result;
}
Json::Value Server::replaceRates(std::uint64_t expected_revision,const Json::Value& complete_rates) {
  requireRunning();
  if(!expected_revision)throw std::invalid_argument("positive rate revision required");
  if(expected_revision!=rates_revision_)throw ControlFailure(ControlError::Conflict,"rate revision conflict");
  if(rates_revision_==std::numeric_limits<std::uint64_t>::max())throw ControlFailure(ControlError::Conflict,"rate revision exhausted");
  auto prepared=parseRates(complete_rates,true);std::shared_ptr<const Rates> replacement(new Rates(std::move(prepared)));std::atomic_store(&rates_,replacement);
  // This publication is the application boundary. A tick already holding
  // the old immutable snapshot finishes with that snapshot; subsequent
  // ticks load the new complete table, without partial channel updates.
  ++rates_revision_;rates_applied_steady_ns_=steadyNs();wakePublisher();return ratesStatus();
}
Json::Value Server::instanceStatus(const std::string& id) const {
  validateInstanceId(id);const auto found=instances_.find(id);
  if(found==instances_.end())throw ControlFailure(ControlError::NotFound,"instance not found");
  return found->second->status();
}
Json::Value Server::removeInstance(const std::string& id) {
  requireRunning();validateInstanceId(id);const auto found=instances_.find(id);
  if(found!=instances_.end()) {auto instance=found->second;instances_.erase(found);replaceSnapshot();instance->deactivate();}
  Json::Value result;result["ok"]=true;result["id"]=id;result["removed"]=true;return result;
}
Json::Value Server::activateInstance(const std::string& id,const Json::Value& configuration) {
  requireRunning();validateInstanceId(id);const auto found=instances_.find(id);
  if(found!=instances_.end()) {
    if(found->second->configuration()!=configuration)throw ControlFailure(ControlError::Conflict,"different instance content; remove the existing instance explicitly before replacing its configuration");
    auto status=found->second->status();status["unchanged"]=true;return status;
  }
  if(instances_.size()>=64)throw ControlFailure(ControlError::Conflict,"server supports at most 64 instances");
  auto config=parseInstance(configuration);
  if(configuration["settings"].isMember("use_sim_time")&&configuration["settings"]["use_sim_time"].asBool()!=ros::Time::isSimTime())
    throw ControlFailure(ControlError::Conflict,"instance clock conflicts with the immutable server world_clock");
  std::shared_ptr<Instance> prepared(new Instance(id,std::move(config)));
  const auto claims=prepared->claims();
  for(const auto& item:instances_) {const auto occupied=item.second->claims();for(const auto& claim:claims)if(occupied.count(claim))throw ControlFailure(ControlError::Conflict,"instance output conflicts with "+item.first+": "+claim);}
  prepared->activate(input_);instances_.emplace(id,prepared);replaceSnapshot();return prepared->status();
}
} // namespace xgc2_ros_visualizer
