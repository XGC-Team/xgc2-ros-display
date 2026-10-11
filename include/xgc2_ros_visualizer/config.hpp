#pragma once

#include <array>
#include <string>
#include <vector>
#include <json/json.h>
#include <xgc2_ros_visualizer/scene_contract.hpp>

namespace xgc2_ros_visualizer {
// A robot's kind selects its renderer and its row of the rate table. Global is
// the row of everything that belongs to no concrete robot kind (standard TF
// root, static frames, world boundary, readiness and robots that only publish
// their description).
enum class RateKind : std::size_t { Fs150, Scout, Mecanum, Global, Count };
enum class Channel : std::size_t {
  PoseTf, JointTf, Markers, Scene, ScenePath, Path, ArPath, ArIdentity,
  HeightProjection, HeightProjectionAr, TfRoot, TfStatic, WorldBoundary,
  Readiness, DisplayPointcloud, DisplayGrid, DisplayPath, DisplayPoseArray, Count
};
constexpr std::size_t kKindCount = static_cast<std::size_t>(RateKind::Count);
constexpr std::size_t kChannelCount = static_cast<std::size_t>(Channel::Count);
const char* kindName(RateKind kind);
const char* channelName(Channel channel);
RateKind parseKind(const std::string& name);
RateKind rateKind(RobotModelKind kind);
RobotModelKind modelKind(RateKind kind);
bool applicable(RateKind kind, Channel channel);
// A robot can override the channels of its own kind row, except the display
// relay channels: relays belong to the instance, not to one robot.
bool robotChannel(RateKind kind, Channel channel);

// The per-kind table. It holds the defaults of every robot of that kind.
struct Rates {
  std::array<std::array<double, kChannelCount>, kKindCount> values{};
  double get(RateKind kind, Channel channel) const;
  double maximum() const;
  Json::Value json() const;
};
Rates defaultRates();
// The runtime table is always replaced as a whole.
Rates parseRates(const Json::Value& value, bool complete);

// Rates of one robot that differ from its kind row. A zero follows the table.
struct RateOverrides {
  std::array<double, kChannelCount> values{};
  double get(Channel channel, double inherited) const;
  double maximum() const;
  bool empty() const;
  Json::Value json() const;
  bool operator==(const RateOverrides& other) const { return values == other.values; }
  bool operator!=(const RateOverrides& other) const { return !(*this == other); }
};
// Strict: unknown or non-overridable channels and rates outside 0.1..1000 Hz fail.
RateOverrides parseRateOverrides(RateKind kind, const Json::Value& value);

Json::Value parseJson(const std::string& text);
std::string jsonText(const Json::Value& value);

// A byte-preserving copy of one source topic below /xgc/display. Its rate comes
// from the kind row of the robot that owns the source.
struct RelayConfig {
  std::string source, topic, message_type;
  RateKind kind;
  Channel channel;
  bool operator==(const RelayConfig& other) const {
    return source == other.source && topic == other.topic && message_type == other.message_type &&
        kind == other.kind && channel == other.channel;
  }
};
RelayConfig relayConfig(const std::string& source, const std::string& topic,
                      const std::string& message_type, RateKind kind);
} // namespace xgc2_ros_visualizer
