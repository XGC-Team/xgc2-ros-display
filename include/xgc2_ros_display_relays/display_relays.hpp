#pragma once
#include <memory>
#include <string>
#include <vector>
namespace ros { class NodeHandle; }
namespace xgc2_ros_display_relays {
struct RelaySpec {
  std::string source, topic, message_type;
  double max_rate_hz;
};
// Same schema and rejection boundaries as the authoritative relaysJson.
std::vector<RelaySpec> parseRelaySpecs(const std::string& json);
void validateRelaySpecs(const std::vector<RelaySpec>& specs);
// ROS must be initialized by the embedding owner. No global spinner is needed.
// start/stop are serialized; stop waits for this owner's callbacks to finish.
// No message is published after stop returns. Other ROS owners are untouched.
class DisplayRelays final {
 public:
  explicit DisplayRelays(std::vector<RelaySpec> specs);
  ~DisplayRelays();
  DisplayRelays(const DisplayRelays&) = delete;
  DisplayRelays& operator=(const DisplayRelays&) = delete;
  void start(const ros::NodeHandle& owner_node);
  void stop() noexcept;
  void rethrowFailure() const;
 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace xgc2_ros_display_relays
