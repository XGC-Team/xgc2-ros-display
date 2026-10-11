#pragma once
#include <atomic>
#include <csignal>
#include <cstdint>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include "../runtime/instance.hpp"
namespace xgc2_ros_visualizer {
enum class ControlError { NotFound, Conflict, Unavailable };
class ControlFailure : public std::runtime_error {
 public:
  ControlFailure(ControlError code,const std::string& message):std::runtime_error(message),code(code) {}
  const ControlError code;
};
void validateInstanceId(const std::string& id);
void validateRobotId(const std::string& id);

class Server {
 public:
  Server(std::size_t workers,Rates rates,const volatile std::sig_atomic_t* signal_stop=nullptr);
  ~Server();
  // Readiness for the describe contract; safe from any thread.
  bool ready() const;
  Json::Value facts() const;
  // Native control owner only; the fixed domain worker calls these directly.
  // They own validation, atomic application and publication fences.
  Json::Value status() const;
  Json::Value ratesStatus() const;
  Json::Value replaceRates(std::uint64_t expected_revision,const Json::Value& complete_rates);
  Json::Value instanceStatus(const std::string& id) const;
  Json::Value applyInstance(const std::string& id,const Json::Value& input);
  Json::Value removeInstance(const std::string& id);
  Json::Value robotStatus(const std::string& id,const std::string& robot) const;
  Json::Value applyRobot(const std::string& id,const std::string& robot,const Json::Value& request);
  Json::Value removeRobot(const std::string& id,const std::string& robot);
  Json::Value robotRates(const std::string& id,const std::string& robot) const;
  Json::Value applyRobotRates(const std::string& id,const std::string& robot,const Json::Value& request);
  const std::atomic<bool>& stopping() const { return stopping_; }
  void stop();
  void rethrowFailure();
 private:
  void publishLoop();
  void replaceSnapshot();
  void wakePublisher();
  void watchMaster();
  void requireRunning() const;
  Instance& instance(const std::string& id) const;
  void requireFreeClaims(const std::string& id,const InstanceSpec& spec) const;
  void countMembers();
  InputPool input_;
  // The ROS master this server is bound to: its address and, when it publishes
  // one, the identity of this master run. A replaced master ends the server.
  std::string master_uri_,master_run_id_;
  std::atomic<bool> master_reachable_{true};
  std::map<std::string,std::shared_ptr<Instance>> instances_; // Control-loop owner only.
  std::shared_ptr<const std::vector<std::shared_ptr<Instance>>> snapshot_;
  std::shared_ptr<const Rates> rates_;
  std::uint64_t rates_revision_{1}; // Domain-worker owner; never persisted.
  std::int64_t rates_applied_steady_ns_{0};
  std::atomic<std::size_t> instance_count_{0},robot_count_{0};
  std::atomic<bool> stopping_{false};
  std::thread publisher_,master_watch_;
  std::mutex failure_mutex_;
  std::exception_ptr failure_;
  const volatile std::sig_atomic_t* signal_stop_;
  std::condition_variable wake_;
  std::mutex wait_mutex_;
  std::atomic<std::uint64_t> revision_{0};
};
} // namespace xgc2_ros_visualizer
