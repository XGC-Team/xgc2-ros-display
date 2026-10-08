#include <xgc2_ros_visualizer/config.hpp>
#include <algorithm>
#include <cmath>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>

namespace xgc2_ros_visualizer {
namespace {
constexpr const char* kinds[] = {"fs150", "scout", "mecanum", "global"};
constexpr const char* channels[] = {"pose_tf", "joint_tf", "markers", "scene", "scene_path", "path",
  "ar_path", "ar_identity", "height_projection", "height_projection_ar", "tf_root", "tf_static",
  "world_boundary", "readiness", "display_pointcloud", "display_grid", "display_path", "display_pose_array"};
void object(const Json::Value& v, const std::string& name) {
  if (!v.isObject()) throw std::invalid_argument(name + " must be an object");
}
double number(const Json::Value& v, double fallback, const std::string& name) {
  if (v.isNull()) return fallback;
  if (!v.isNumeric() || v.isBool() || !std::isfinite(v.asDouble()))
    throw std::invalid_argument(name + " must be finite numeric");
  return v.asDouble();
}
bool boolean(const Json::Value& v, bool fallback, const std::string& name) {
  if (v.isNull()) return fallback;
  if (!v.isBool()) throw std::invalid_argument(name + " must be boolean");
  return v.asBool();
}
std::string string(const Json::Value& v, const std::string& fallback, const std::string& name) {
  if (v.isNull()) return fallback;
  if (!v.isString()) throw std::invalid_argument(name + " must be a string");
  return v.asString();
}
std::vector<xgc2_robot_visualization::RobotDescription> roster(const Json::Value& value) {
  if (!value.isArray()) throw std::invalid_argument("robot roster must be an array");
  std::vector<xgc2_robot_visualization::RobotDescription> result;
  std::string error;
  if (!xgc2_robot_visualization::readRobotVisualizationRoster(jsonText(value), &result, &error))
    throw std::invalid_argument(error);
  return result;
}
bool canonicalTopic(const std::string& name) {
  static const std::regex pattern("^/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$");
  return name.size() <= 499 && std::regex_match(name, pattern);
}
}
const char* kindName(RateKind kind) { return kinds[static_cast<std::size_t>(kind)]; }
const char* channelName(Channel channel) { return channels[static_cast<std::size_t>(channel)]; }
RateKind parseKind(const std::string& name) {
  for (std::size_t i=0; i<kKindCount; ++i) if (name == kinds[i]) return static_cast<RateKind>(i);
  throw std::invalid_argument("unknown robotKind: " + name);
}
RateKind rateKind(RobotModelKind kind) {
  if (kind == RobotModelKind::kFs150) return RateKind::Fs150;
  if (kind == RobotModelKind::kScout) return RateKind::Scout;
  if (kind == RobotModelKind::kMecanum) return RateKind::Mecanum;
  return RateKind::Global;
}
bool applicable(RateKind kind, Channel channel) {
  if (channel >= Channel::DisplayPointcloud) return true;
  if (kind == RateKind::Global) return channel == Channel::JointTf || (channel >= Channel::TfRoot && channel <= Channel::Readiness);
  if (channel >= Channel::TfRoot) return false;
  if (channel == Channel::HeightProjection || channel == Channel::HeightProjectionAr)
    return kind == RateKind::Fs150;
  return true;
}
double Rates::get(RateKind kind, Channel channel) const {
  return values[static_cast<std::size_t>(kind)][static_cast<std::size_t>(channel)];
}
double Rates::maximum() const {
  double result=0.1;
  for (const auto& row : values) for (double value : row) result=std::max(result,value);
  return result;
}
Json::Value Rates::json() const {
  Json::Value result(Json::objectValue);
  for (std::size_t k=0; k<kKindCount; ++k) for (std::size_t c=0; c<kChannelCount; ++c)
    if (applicable(static_cast<RateKind>(k),static_cast<Channel>(c))) result[kinds[k]][channels[c]]=values[k][c];
  return result;
}
Rates defaultRates() {
  Rates result;
  for (std::size_t k=0; k<kKindCount; ++k) for (std::size_t c=0; c<kChannelCount; ++c) {
    auto channel=static_cast<Channel>(c);
    if (!applicable(static_cast<RateKind>(k),channel)) continue;
    double hz=10;
    if (channel==Channel::PoseTf) hz=120;
    if (channel==Channel::JointTf || channel==Channel::Markers || channel==Channel::TfRoot) hz=30;
    if (channel==Channel::TfStatic || channel==Channel::WorldBoundary || channel==Channel::Readiness) hz=1;
    result.values[k][c]=hz;
  }
  return result;
}
Rates parseRates(const Json::Value& value, bool complete) {
  object(value,"rates");
  Rates result=defaultRates();
  std::size_t entries=0;
  for (const auto& kind_name : value.getMemberNames()) {
    auto kind=parseKind(kind_name); object(value[kind_name],kind_name);
    for (const auto& channel_name : value[kind_name].getMemberNames()) {
      std::size_t c=0; for (; c<kChannelCount && channel_name!=channels[c]; ++c) {}
      if (c==kChannelCount || !applicable(kind,static_cast<Channel>(c)))
        throw std::invalid_argument("non-applicable or unknown rate: " + kind_name + "/" + channel_name);
      double hz=number(value[kind_name][channel_name],0,channel_name);
      if (hz<0.1 || hz>1000) throw std::invalid_argument("rate must be within 0.1..1000 Hz");
      result.values[static_cast<std::size_t>(kind)][c]=hz; ++entries;
    }
  }
  if (complete) {
    std::size_t expected=0;
    for (std::size_t k=0;k<kKindCount;++k) for (std::size_t c=0;c<kChannelCount;++c)
      if (applicable(static_cast<RateKind>(k),static_cast<Channel>(c))) ++expected;
    if (entries!=expected) throw std::invalid_argument("runtime rates require the complete kind/channel table");
  }
  return result;
}
Json::Value parseJson(const std::string& text) {
  Json::CharReaderBuilder builder;
  builder["allowComments"]=false; builder["failIfExtra"]=true; builder["rejectDupKeys"]=true;
  Json::Value result; std::string errors; std::istringstream stream(text);
  if (!Json::parseFromStream(builder,stream,&result,&errors)) throw std::invalid_argument("invalid JSON: "+errors);
  return result;
}
std::string jsonText(const Json::Value& value) {
  Json::StreamWriterBuilder builder; builder["indentation"]=""; return Json::writeString(builder,value);
}
std::vector<RelayConfig> parseRelays(const Json::Value& value) {
  if (!value.isArray() || value.size()>64) throw std::invalid_argument("displayRelays must be an array of at most 64");
  std::vector<RelayConfig> result; std::set<std::string> sources;
  const std::set<std::string> fields{"source","topic","messageType","robotKind"};
  for (const auto& item : value) {
    object(item,"relay"); auto names=item.getMemberNames();
    if (std::set<std::string>(names.begin(),names.end())!=fields) throw std::invalid_argument("relay fields must be source/topic/messageType/robotKind");
    for (const auto& field:fields) if (!item[field].isString()) throw std::invalid_argument("relay fields must be strings");
    RelayConfig spec{item["source"].asString(),item["topic"].asString(),item["messageType"].asString(),parseKind(item["robotKind"].asString()),Channel::DisplayPath};
    if (!canonicalTopic(spec.source) || spec.source=="/xgc/display" || spec.source.compare(0,13,"/xgc/display/")==0 ||
        spec.topic!="/xgc/display"+spec.source || !sources.insert(spec.source).second)
      throw std::invalid_argument("relay needs a unique canonical source and display copy");
    if (spec.message_type=="sensor_msgs/PointCloud2") spec.channel=Channel::DisplayPointcloud;
    else if (spec.message_type=="nav_msgs/OccupancyGrid") spec.channel=Channel::DisplayGrid;
    else if (spec.message_type=="nav_msgs/Path") spec.channel=Channel::DisplayPath;
    else if (spec.message_type=="geometry_msgs/PoseArray") spec.channel=Channel::DisplayPoseArray;
    else throw std::invalid_argument("relay requires one of the four full-state types");
    result.push_back(std::move(spec));
  }
  return result;
}
RobotModelKind modelKind(const Settings& settings,const std::string& model) {
  if (settings.models[0].count(model)) return RobotModelKind::kFs150;
  if (settings.models[1].count(model)) return RobotModelKind::kScout;
  if (settings.models[2].count(model)) return RobotModelKind::kMecanum;
  return RobotModelKind::kNone;
}
InstanceConfig parseInstance(const Json::Value& value) {
  object(value,"instance"); const std::set<std::string> fields{"robots","descriptions","worldBoundary","settings","displayRelays"};
  auto names=value.getMemberNames();
  if (std::set<std::string>(names.begin(),names.end())!=fields) throw std::invalid_argument("instance requires exactly robots/descriptions/worldBoundary/settings/displayRelays");
  InstanceConfig result; result.original=value; result.robots=roster(value["robots"]); result.descriptions=roster(value["descriptions"]);
  result.relays=parseRelays(value["displayRelays"]);
  if (!value["worldBoundary"].isNull() && !value["worldBoundary"].isObject()) throw std::invalid_argument("worldBoundary must be object or null");
  result.boundary=parseWorldBoundaryDisplay(value["worldBoundary"].isNull()?"":jsonText(value["worldBoundary"]));
  const auto& v=value["settings"]; object(v,"settings"); auto& s=result.settings;
  const std::set<std::string> allowed{"frame_id","tracked_fs150_models","tracked_scout_models","tracked_mecanum_models","marker_color","world_boundary_mode","label_scale_invariant","label_font_size_meters","label_font_size_pixels","marker_opacity","uav_label_offset","scout_label_offset","mecanum_label_offset","use_sim_time","track_ugv","publish_markers","publish_transforms","publish_scene_update","publish_scene_paths","publish_paths","scene_update_topic","transform_topic","canonical_pose_timeout","mavros_state_timeout","ugv_motion_timeout","uav_rotor_speed_ground","uav_rotor_speed_transition","uav_rotor_speed_airborne","uav_mesh_scale","ugv_mesh_scale","mecanum_mesh_scale","ugv_visual_wheel_radius","ugv_visual_track_width","ugv_wheel_motion_deadband","ugv_max_visual_wheel_speed_rad_s"};
  for (const auto& field:v.getMemberNames()) if (!allowed.count(field)) throw std::invalid_argument("unknown setting: "+field);
  s.frame_id=string(v["frame_id"],s.frame_id,"frame_id"); if (!isWorldFixedFrame(s.frame_id)) throw std::invalid_argument("frame_id must be world");
  s.scene_topic=string(v["scene_update_topic"],s.scene_topic,"scene_update_topic"); s.transform_topic=string(v["transform_topic"],s.transform_topic,"transform_topic");
  if (!canonicalTopic(s.scene_topic) || !canonicalTopic(s.transform_topic)) throw std::invalid_argument("output topics must be canonical absolute names");
  s.models[0]=parseModelNames(string(v["tracked_fs150_models"],"","tracked_fs150_models"));
  s.models[1]=parseModelNames(string(v["tracked_scout_models"],"","tracked_scout_models"));
  s.models[2]=parseModelNames(string(v["tracked_mecanum_models"],"","tracked_mecanum_models"));
  if (!modelListsAreDisjoint(s.models[0],s.models[1],s.models[2])) throw std::invalid_argument("robot model lists overlap");
  const auto color=string(v["marker_color"],"#00a2ff","marker_color");
  const bool invariant=boolean(v["label_scale_invariant"],false,"label_scale_invariant");
  const double meters=number(v["label_font_size_meters"],.24,"label_font_size_meters"),pixels=number(v["label_font_size_pixels"],16,"label_font_size_pixels");
  sceneLabelStyleFromMarkerColor(color,false,meters,number(v["marker_opacity"],1,"marker_opacity"));
  sceneLabelStyleFromMarkerColor(color,true,pixels,number(v["marker_opacity"],1,"marker_opacity"));
  s.label_style=sceneLabelStyleFromMarkerColor(color,invariant,invariant?pixels:meters,number(v["marker_opacity"],1,"marker_opacity"));
  s.label_offsets.uav=number(v["uav_label_offset"],.55,"uav_label_offset"); s.label_offsets.scout=number(v["scout_label_offset"],.65,"scout_label_offset"); s.label_offsets.mecanum=number(v["mecanum_label_offset"],.32,"mecanum_label_offset"); validateSceneLabelOffsets(s.label_offsets);
  s.boundary_mode=worldBoundaryDisplayModeFromString(string(v["world_boundary_mode"],"walls","world_boundary_mode"));
  s.publish_markers=boolean(v["publish_markers"],false,"publish_markers"); s.publish_transforms=boolean(v["publish_transforms"],true,"publish_transforms"); s.publish_scene=boolean(v["publish_scene_update"],true,"publish_scene_update"); s.publish_scene_paths=boolean(v["publish_scene_paths"],false,"publish_scene_paths"); s.publish_paths=boolean(v["publish_paths"],true,"publish_paths");
  s.track_ugv=boolean(v["track_ugv"],true,"track_ugv"); boolean(v["use_sim_time"],false,"use_sim_time");
  const std::pair<const char*,double*> numeric[]={{"canonical_pose_timeout",&s.pose_timeout},{"mavros_state_timeout",&s.state_timeout},{"ugv_motion_timeout",&s.motion_timeout},{"uav_rotor_speed_ground",&s.rotor_ground},{"uav_rotor_speed_transition",&s.rotor_transition},{"uav_rotor_speed_airborne",&s.rotor_airborne},{"uav_mesh_scale",&s.uav_mesh_scale},{"ugv_mesh_scale",&s.scout_mesh_scale},{"mecanum_mesh_scale",&s.mecanum_mesh_scale},{"ugv_visual_wheel_radius",&s.wheel_radius},{"ugv_visual_track_width",&s.track_width},{"ugv_wheel_motion_deadband",&s.wheel_deadband},{"ugv_max_visual_wheel_speed_rad_s",&s.wheel_max_speed}};
  for (const auto& field:numeric) { *field.second=number(v[field.first],*field.second,field.first); if (*field.second<0) throw std::invalid_argument(std::string(field.first)+" must be nonnegative"); }
  if (s.wheel_radius<=0 || s.track_width<=0 || s.uav_mesh_scale<=0 || s.scout_mesh_scale<=0 || s.mecanum_mesh_scale<=0) throw std::invalid_argument("scales and wheel geometry must be positive");
  std::set<std::string> models,paths;
  for (const auto& robot:result.robots) {
    if (robot.scene_model.empty()) continue; // A description-only robot has no scene class.
    if (modelKind(s,robot.scene_model)==RobotModelKind::kNone || !models.insert(robot.scene_model).second)
      throw std::invalid_argument("scene robot must map exactly once to a configured kind/model");
    // The retained renderer's label namespace contract must fail in the cold
    // request path, never later in the shared publication thread.
    visualization_msgs::MarkerArray label_contract;
    applyRobotMarkerLabel(&label_contract,0,modelKind(s,robot.scene_model),robot.ros_namespace);
    if (!paths.insert(robot.ros_namespace+"/"+robot.path_topic).second || (!robot.ar_path_topic.empty() && !paths.insert(robot.ros_namespace+"/"+robot.ar_path_topic).second)) throw std::invalid_argument("robot path topic collision");
  }
  if (models.size()!=s.models[0].size()+s.models[1].size()+s.models[2].size()) throw std::invalid_argument("every configured model must occur in the robot roster");
  return result;
}
} // namespace xgc2_ros_visualizer
