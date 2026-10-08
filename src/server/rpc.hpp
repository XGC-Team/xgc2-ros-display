#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>

#include <json/json.h>

namespace xgc2_ros_visualizer {

struct RpcReply {
  int status;
  Json::Value body;
};

using RpcHandler = std::function<RpcReply(const std::string& method,
                                          const std::string& path,
                                          const Json::Value& body)>;

// One Unix HTTP/JSON control loop, independent of ROS and domain state.
// Construction binds an absent absolute socket path with permissions 0600.
// Each connection carries one request and is then closed. Limits are fixed:
// 32 clients, 16 KiB headers, 1 MiB JSON bodies/replies, 5 s client lifetime.
// The handler runs synchronously on the run() thread; it must be bounded and
// must not wait for another RPC request on this server.
class RpcServer {
 public:
  RpcServer(std::string socket_path, RpcHandler handler);
  ~RpcServer();

  RpcServer(const RpcServer&) = delete;
  RpcServer& operator=(const RpcServer&) = delete;
  RpcServer(RpcServer&&) = delete;
  RpcServer& operator=(RpcServer&&) = delete;

  // Call once. Stop is checked at least every 100 ms between handler calls.
  // The caller must let run() return before destroying the server.
  void run(const std::atomic<bool>& stopping);
  void stop() noexcept;
  const std::string& socket_path() const noexcept;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace xgc2_ros_visualizer
