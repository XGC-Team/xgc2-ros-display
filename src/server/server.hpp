#pragma once
#include <atomic>
#include <csignal>
#include <cstdint>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
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
class Server {
 public:
  Server(std::string identity,std::size_t workers,Rates rates,const volatile std::sig_atomic_t* signal_stop=nullptr);
  ~Server();
  // Native control owner only; startup and the fixed domain worker call these
  // directly. They own validation, atomic application and publication fences.
  Json::Value health() const;
  Json::Value status() const;
  Json::Value ratesStatus() const;
  Json::Value replaceRates(std::uint64_t expected_revision,const Json::Value& complete_rates);
  Json::Value instanceStatus(const std::string& id) const;
  Json::Value activateInstance(const std::string& id,const Json::Value& configuration);
  Json::Value removeInstance(const std::string& id);
  const std::atomic<bool>& stopping() const { return stopping_; }
  void stop();
  void rethrowFailure();
 private:
  void publishLoop();
  void replaceSnapshot();
  void wakePublisher();
  void requireRunning() const;
  std::string identity_;
  InputPool input_;
  std::map<std::string,std::shared_ptr<Instance>> instances_; // Control-loop owner only.
  std::shared_ptr<const std::vector<std::shared_ptr<Instance>>> snapshot_;
  std::shared_ptr<const Rates> rates_;
  std::uint64_t rates_revision_{1}; // Domain-worker owner; never persisted.
  std::int64_t rates_applied_steady_ns_{0};
  std::atomic<bool> stopping_{false};
  std::thread publisher_;
  std::mutex failure_mutex_;
  std::exception_ptr failure_;
  const volatile std::sig_atomic_t* signal_stop_;
  std::condition_variable wake_;
  std::mutex wait_mutex_;
  std::atomic<std::uint64_t> revision_{0};
};
} // namespace xgc2_ros_visualizer
