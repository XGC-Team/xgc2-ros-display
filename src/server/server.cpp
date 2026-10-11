#include "server.hpp"
#include <xgc2_ros_visualizer/instance_input.hpp>
#include <algorithm>
#include <chrono>
#include <regex>
#include <limits>
#include <set>
#include <stdexcept>
namespace xgc2_ros_visualizer {
namespace {
constexpr std::size_t kMaxInstances=64;
std::int64_t steadyNs() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
// A request that names the revision it expects: absent means unconditional.
bool expectedRevision(const Json::Value& request,std::uint64_t* revision) {
  if(!request.isMember("expectedRevision"))return false;
  const auto& value=request["expectedRevision"];
  if((value.type()!=Json::intValue&&value.type()!=Json::uintValue)||!value.isUInt64())throw std::invalid_argument("expectedRevision must be a nonnegative integer");
  *revision=value.asUInt64();return true;
}
void requireFields(const Json::Value& request,const std::set<std::string>& required,const std::set<std::string>& optional) {
  if(!request.isObject())throw std::invalid_argument("request must be an object");
  for(const auto& name:request.getMemberNames())
    if(!required.count(name)&&!optional.count(name))throw std::invalid_argument("unknown request field: "+name);
  for(const auto& name:required)
    if(!request.isMember(name))throw std::invalid_argument("missing request field: "+name);
}
}
Server::Server(std::size_t workers,Rates rates,const volatile std::sig_atomic_t* signal_stop)
    : input_(workers),snapshot_(new std::vector<std::shared_ptr<Instance>>),rates_(new Rates(std::move(rates))),signal_stop_(signal_stop) {
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
  countMembers();
  wakePublisher();
}
void Server::countMembers() {
  std::size_t robots=0;for(const auto& item:instances_)robots+=item.second->spec().robots.size();
  instance_count_.store(instances_.size());robot_count_.store(robots);
}
void Server::publishLoop() {
  try {
    while(!stopping_.load()&&ros::ok()&&(!signal_stop_||!*signal_stop_)) {
      const auto revision=revision_.load();
      const auto started=std::chrono::steady_clock::now();auto rates=std::atomic_load(&rates_);auto members=std::atomic_load(&snapshot_);
      input_.rethrowFailure();
      const auto now=ros::Time::now();double maximum=rates->maximum();
      for(const auto& instance:*members) {instance->tick(*rates,now);maximum=std::max(maximum,instance->maximumOverride());}
      // One shared wait; membership/rates/Stop wake immediately. A bounded wall
      // heartbeat also observes signal flags safely, without notifying from a handler.
      const auto period=std::chrono::duration<double>(std::min(.1,1.0/maximum));
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
bool Server::ready() const {return !stopping_.load();}
Json::Value Server::facts() const {
  Json::Value result(Json::objectValue);
  result["world_clock"]=ros::Time::isSimTime()?"simulation":"wall";
  result["callback_workers"]=Json::UInt64(input_.size());
  result["instances"]=Json::UInt64(instance_count_.load());
  result["robots"]=Json::UInt64(robot_count_.load());
  result["max_instances"]=Json::UInt64(kMaxInstances);
  result["max_robots_per_instance"]=Json::UInt64(kMaxRobots);
  if(!ready())result["reason"]="stopping";
  return result;
}
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
void validateRobotId(const std::string& id) {
  static const std::regex identity("^[A-Za-z_][A-Za-z0-9_]{0,126}$");
  if(!std::regex_match(id,identity))throw std::invalid_argument("invalid robot identity");
}
void Server::requireRunning() const {
  if(stopping_.load())throw ControlFailure(ControlError::Unavailable,"server is stopping");
}
Instance& Server::instance(const std::string& id) const {
  validateInstanceId(id);const auto found=instances_.find(id);
  if(found==instances_.end())throw ControlFailure(ControlError::NotFound,"instance not found");
  return *found->second;
}
void Server::requireFreeClaims(const std::string& id,const InstanceSpec& spec) const {
  const auto wanted=claims(spec);
  for(const auto& item:instances_) {
    if(item.first==id)continue;
    for(const auto& claim:item.second->claims())
      if(wanted.count(claim))throw ControlFailure(ControlError::Conflict,"instance output conflicts with "+item.first+": "+claim);
  }
}
Json::Value Server::status() const {
  Json::Value result;result["ok"]=true;result["ready"]=ready();result["callbackWorkers"]=Json::UInt64(input_.size());
  result["publisherWorkers"]=1;result["instanceCount"]=Json::UInt64(instances_.size());result["rates"]=std::atomic_load(&rates_)->json();
  Json::UInt64 robots=0,scene_robots=0,relays=0;result["instances"]=Json::Value(Json::arrayValue);
  for(const auto& item:instances_) {auto status=item.second->status();robots+=status["descriptionCount"].asUInt64();scene_robots+=status["robotCount"].asUInt64();relays+=status["relayCount"].asUInt64();result["instances"].append(status);}
  result["robotCount"]=scene_robots;result["descriptionCount"]=robots;result["relayCount"]=relays;return result;
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
Json::Value Server::instanceStatus(const std::string& id) const {return instance(id).status();}
Json::Value Server::removeInstance(const std::string& id) {
  requireRunning();validateInstanceId(id);const auto found=instances_.find(id);
  if(found!=instances_.end()) {auto removed=found->second;instances_.erase(found);replaceSnapshot();removed->deactivate();}
  Json::Value result;result["ok"]=true;result["id"]=id;result["removed"]=true;return result;
}
Json::Value Server::applyInstance(const std::string& id,const Json::Value& input) {
  requireRunning();validateInstanceId(id);
  const auto spec=projectInstanceInput(input);
  const auto found=instances_.find(id);
  requireFreeClaims(id,spec);
  if(found==instances_.end()) {
    if(instances_.size()>=kMaxInstances)throw ControlFailure(ControlError::Conflict,"server supports at most 64 instances");
    std::shared_ptr<Instance> created(new Instance(id));
    const auto change=created->apply(spec,input_);
    instances_.emplace(id,created);replaceSnapshot();
    auto status=created->status();status["created"]=true;status["unchanged"]=false;status["changes"]=change.json();return status;
  }
  const auto change=found->second->apply(spec,input_);
  countMembers();wakePublisher();
  auto status=found->second->status();status["created"]=false;status["unchanged"]=change.empty();status["changes"]=change.json();return status;
}
Json::Value Server::robotStatus(const std::string& id,const std::string& robot) const {
  auto& owner=instance(id);validateRobotId(robot);
  if(!owner.hasRobot(robot))throw ControlFailure(ControlError::NotFound,"robot not found");
  return owner.robotStatus(robot,*std::atomic_load(&rates_));
}
Json::Value Server::applyRobot(const std::string& id,const std::string& robot,const Json::Value& request) {
  requireRunning();auto& owner=instance(id);validateRobotId(robot);
  requireFields(request,{"profile"},{"expectedRevision"});
  std::uint64_t expected=0;
  if(expectedRevision(request,&expected)) {
    // 0 expects that the robot does not exist yet.
    const std::uint64_t current=owner.hasRobot(robot)?owner.robotRevision(robot):0;
    if(expected!=current)throw ControlFailure(ControlError::Conflict,"robot revision conflict");
  }
  InstanceSpec candidate=owner.spec();
  candidate.robots[robot]=parseRobotProfile(robot,request["profile"]);
  requireFreeClaims(id,candidate);
  const auto change=owner.apply(candidate,input_);
  countMembers();wakePublisher();
  auto status=owner.robotStatus(robot,*std::atomic_load(&rates_));status["unchanged"]=change.empty();status["changes"]=change.json();return status;
}
Json::Value Server::removeRobot(const std::string& id,const std::string& robot) {
  requireRunning();auto& owner=instance(id);validateRobotId(robot);
  Json::Value result;result["ok"]=true;result["instanceId"]=id;result["id"]=robot;result["removed"]=true;
  if(!owner.hasRobot(robot))return result;
  InstanceSpec candidate=owner.spec();candidate.robots.erase(robot);
  owner.apply(candidate,input_);
  countMembers();wakePublisher();return result;
}
Json::Value Server::robotRates(const std::string& id,const std::string& robot) const {
  auto status=robotStatus(id,robot);
  Json::Value result;
  for(const char* name:{"ok","instanceId","id","kind","configuration","rates"})result[name]=status[name];
  return result;
}
Json::Value Server::applyRobotRates(const std::string& id,const std::string& robot,const Json::Value& request) {
  requireRunning();auto& owner=instance(id);validateRobotId(robot);
  requireFields(request,{"rates"},{"expectedRevision"});
  if(!owner.hasRobot(robot))throw ControlFailure(ControlError::NotFound,"robot not found");
  std::uint64_t expected=0;
  if(expectedRevision(request,&expected)&&expected!=owner.robotRevision(robot))throw ControlFailure(ControlError::Conflict,"robot revision conflict");
  const auto overrides=parseRateOverrides(owner.spec().robots.at(robot).kind,request["rates"]);
  const bool changed=owner.setRobotRates(robot,overrides);
  if(changed)wakePublisher();
  auto result=robotRates(id,robot);result["unchanged"]=!changed;return result;
}
} // namespace xgc2_ros_visualizer
