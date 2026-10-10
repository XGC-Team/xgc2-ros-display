#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <json/json.h>
#include <xgc2_ros_visualizer/robot_roster.hpp>
#include <xgc2_ros_visualizer/scene_contract.hpp>

namespace xgc2_ros_visualizer {
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
bool applicable(RateKind kind, Channel channel);
struct Rates {
  std::array<std::array<double, kChannelCount>, kKindCount> values{};
  double get(RateKind kind, Channel channel) const;
  double maximum() const;
  Json::Value json() const;
};
Rates defaultRates();
// Startup may overlay defaults; runtime PUT requires the complete table.
Rates parseRates(const Json::Value& value, bool complete);
Json::Value parseJson(const std::string& text);
std::string jsonText(const Json::Value& value);

struct RelayConfig {
  std::string source, topic, message_type;
  RateKind kind;
  Channel channel;
};
std::vector<RelayConfig> parseRelays(const Json::Value& value);

struct Settings {
  std::string frame_id{"world"}, scene_topic{"/xgc/scene"}, transform_topic{"/xgc/tf"};
  std::array<std::set<std::string>, 3> models;
  SceneLabelStyle label_style;
  SceneLabelOffsets label_offsets;
  WorldBoundaryDisplayMode boundary_mode{WorldBoundaryDisplayMode::kWalls};
  bool publish_markers{false}, publish_transforms{true}, publish_scene{true};
  bool publish_scene_paths{false}, publish_paths{true};
  bool track_ugv{true};
  double pose_timeout{0.5}, state_timeout{2.0}, motion_timeout{0.5};
  double rotor_ground{25.0}, rotor_transition{90.0}, rotor_airborne{60.0};
  double uav_mesh_scale{1.0}, scout_mesh_scale{1.0}, mecanum_mesh_scale{0.001};
  double wheel_radius{0.08}, track_width{0.416}, wheel_deadband{0.02}, wheel_max_speed{35.0};
};
struct InstanceConfig {
  Json::Value original;
  Settings settings;
  WorldBoundaryDisplay boundary;
  std::vector<xgc2_ros_visualizer::RobotDescription> robots, descriptions;
  std::vector<RelayConfig> relays;
};
InstanceConfig parseInstance(const Json::Value& value);
RobotModelKind modelKind(const Settings& settings, const std::string& model);
} // namespace xgc2_ros_visualizer
