#pragma once
#include <atomic>
#include <csignal>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include "rpc.hpp"
#include "../runtime/instance.hpp"
namespace xgc2_ros_visualizer {
class Server {
 public:
  Server(std::string identity,std::size_t workers,Rates rates,const volatile std::sig_atomic_t* signal_stop=nullptr);
  ~Server();
  RpcReply route(const std::string& method,const std::string& path,const Json::Value& body);
  const std::atomic<bool>& stopping() const { return stopping_; }
  void stop();
  void rethrowFailure();
 private:
  void publishLoop();
  void replaceSnapshot();
  void wakePublisher();
  std::string identity_;
  InputPool input_;
  std::map<std::string,std::shared_ptr<Instance>> instances_; // Control-loop owner only.
  std::shared_ptr<const std::vector<std::shared_ptr<Instance>>> snapshot_;
  std::shared_ptr<const Rates> rates_;
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
