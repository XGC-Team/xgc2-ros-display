#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <json/json.h>

namespace xgc2_ros_visualizer {

struct RpcReply {
  int status;
  Json::Value body;
};

using RpcHandler = std::function<RpcReply(const std::string& method,
                                          const std::string& path,
                                          const Json::Value& body)>;

struct RpcOptions {
  std::string target_id;
  std::string instance_id;
  std::string ros_home;
  std::string ros_log_dir;
  // Borrowed resolved runtime-directory grant; the shared host duplicates it.
  int retained_parent_fd{-1};
  std::vector<std::pair<std::string, std::string>> environment;
};

// A C++14 facade for the C++20 XRPC host, without ROS or SDK types in its ABI.
// One fixed domain worker owns the handler. Its preallocated handoff is bounded
// by XRPC admission, and retained replies keep the endpoint lease until actual
// work ends. Runtime policy is resolved once from the supplied startup snapshot.
class RpcServer {
 public:
  RpcServer(std::string socket_path, RpcHandler handler, RpcOptions options,
            std::function<void()> quiesce_native = {});
  ~RpcServer();
  static std::string newInstanceId();

  RpcServer(const RpcServer&) = delete;
  RpcServer& operator=(const RpcServer&) = delete;
  RpcServer(RpcServer&&) = delete;
  RpcServer& operator=(RpcServer&&) = delete;

  // Call once. Quiesce the domain worker and native owner before SDK drain
  // releases the endpoint lease. The native callback's captures outlive this host.
  // Return only after both are quiescent. In-progress
  // native work is never represented as rolled back by transport cancellation.
  void run(const std::atomic<bool>& stopping);
  void stop() noexcept;
  const std::string& socket_path() const noexcept;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace xgc2_ros_visualizer
