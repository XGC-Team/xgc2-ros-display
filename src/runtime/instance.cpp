#include "instance.hpp"
#include "descriptions.hpp"
#include "../relay/relay.hpp"
#include <xgc2_ros_visualizer/source_history.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <xgc2_robot_visualization/fs150_uav_visualizer.hpp>
#include <xgc2_robot_visualization/scout_ugv_visualizer.hpp>
#include <xgc2_robot_visualization/mecanum_ugv_visualizer.hpp>
#include <xgc2_robot_visualization/robot_frames.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <tf2_msgs/TFMessage.h>

namespace xgc2_ros_visualizer {
namespace {
bool finite(const geometry_msgs::Pose& p) {
  return std::isfinite(p.position.x)&&std::isfinite(p.position.y)&&std::isfinite(p.position.z)&&
    std::isfinite(p.orientation.x)&&std::isfinite(p.orientation.y)&&std::isfinite(p.orientation.z)&&std::isfinite(p.orientation.w);
}
bool finite(const geometry_msgs::Twist& t) {
  return std::isfinite(t.linear.x)&&std::isfinite(t.linear.y)&&std::isfinite(t.linear.z)&&std::isfinite(t.angular.x)&&std::isfinite(t.angular.y)&&std::isfinite(t.angular.z);
}
geometry_msgs::Pose normalized(geometry_msgs::Pose p) {
  auto& q=p.orientation; const double n=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
  if (!std::isfinite(n)||n<1e-9) { q.x=q.y=q.z=0; q.w=1; }
  else { q.x/=n; q.y/=n; q.z/=n; q.w/=n; }
  return p;
}
double yaw(const geometry_msgs::Pose& p) {
  auto q=normalized(p).orientation; return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
}
bool fresh(const ros::Time& stamp,const ros::Time& now,double timeout) { return !stamp.isZero() && now-stamp<=ros::Duration(timeout); }
struct PoseValue {
  geometry_msgs::Pose pose;
  ros::Time stamp;
  // Fixed state stores the accepted frame category, rather than copying strings under its lock.
  std::uint8_t frame{0};
  bool available{false};
  CanonicalPoseSample sample() const {
    CanonicalPoseSample result; result.available=available; result.pose=pose; result.stamp=stamp;
    result.frame_id=frame==1?"world":frame==2?"map":"invalid"; return result;
  }
};
struct RobotState {
  PoseValue pose,ar;
  geometry_msgs::Twist cmd,twist;
  ros::Time cmd_stamp,twist_stamp,state_stamp,extended_stamp;
  bool cmd_available{false},twist_available{false},twist_world{false},state_available{false},extended_available{false},armed{false};
  std::uint8_t landed{0};
};
struct RobotInput {
  std::mutex mutex;
  RobotState state;
  SourceHistory path,ar_path;
};
struct Gate {
  ros::Time last;
  bool initialized{false};
  bool take(const ros::Time& now,double hz) {
    if (!initialized || now<last) { initialized=true;last=now;return true; }
    const double elapsed=(now-last).toSec(),period=1/hz;
    if (elapsed+1e-9<period) return false;
    last+=ros::Duration(std::floor((elapsed+1e-9)/period)*period); return true;
  }
};
foxglove_msgs::SceneEntityDeletion deletion(const std::string& id,const ros::Time& stamp) {
  foxglove_msgs::SceneEntityDeletion result; result.type=foxglove_msgs::SceneEntityDeletion::MATCHING_ID; result.id=id; result.timestamp=stamp; return result;
}
}
class Instance::Impl {
 public:
  struct Robot {
    xgc2_robot_visualization::RobotDescription identity;
    RobotModelKind model_kind;
    RateKind kind;
    std::shared_ptr<RobotInput> input{new RobotInput};
    std::vector<ros::Subscriber> subscribers;
    bool path{false},ar_path{false};
    nav_msgs::Path path_message,ar_path_message;
    visualization_msgs::Marker path_marker;
    std::uint64_t path_revision{0},ar_path_revision{0};
    ros::Publisher path_pub,ar_path_pub;
    bool label_sent{false},ar_label_sent{false};
    bool height_present{false},height_ar_present{false};
    ros::Time pose_tf_stamp,ar_tf_stamp,height_stamp,height_ar_stamp;
  };
  Impl(std::string name,InstanceConfig cfg) : id(std::move(name)),config(std::move(cfg)) {
    const auto& s=config.settings;
    xgc2_robot_visualization::Fs150UavVisualizer::Config u; u.frame_id=s.frame_id;u.mesh_scale=s.uav_mesh_scale;
    uav.reset(new xgc2_robot_visualization::Fs150UavVisualizer(u));
    xgc2_robot_visualization::ScoutUgvVisualizer::Config g;g.frame_id=s.frame_id;g.mesh_scale=s.scout_mesh_scale;
    g.visual_wheel_radius=s.wheel_radius;g.visual_track_width=s.track_width;g.wheel_motion_deadband=s.wheel_deadband;g.max_visual_wheel_speed_rad_s=s.wheel_max_speed;
    scout.reset(new xgc2_robot_visualization::ScoutUgvVisualizer(g));
    xgc2_robot_visualization::MecanumUgvVisualizer::Config m;m.frame_id=s.frame_id;m.mesh_scale=s.mecanum_mesh_scale;
    mecanum.reset(new xgc2_robot_visualization::MecanumUgvVisualizer(m));
    xgc2_robot_visualization::Fs150UavVisualizer path_uav(u);
    xgc2_robot_visualization::ScoutUgvVisualizer path_scout(g);
    xgc2_robot_visualization::MecanumUgvVisualizer path_mecanum(m);
    robots.reserve(config.robots.size());
    for (const auto& identity:config.robots) {
      if (identity.scene_model.empty()) continue;
      if(!s.track_ugv&&modelKind(s,identity.scene_model)!=RobotModelKind::kFs150)continue;
      std::unique_ptr<Robot> robot(new Robot);robot->identity=identity;robot->model_kind=modelKind(s,identity.scene_model);robot->kind=rateKind(robot->model_kind);
      if (s.publish_paths) {
        robot->path=true;robot->ar_path=!identity.ar_pose_topic.empty();
        robot->path_message.header.frame_id=s.frame_id;robot->ar_path_message.header.frame_id=s.frame_id;
        robot->path_message.poses.reserve(61);robot->ar_path_message.poses.reserve(61);
      }
      // Extract exact SDK path metadata in cold configuration without advancing
      // running animation state. Runtime points come from the source-time ring.
      visualization_msgs::MarkerArray prototypes;
      for(unsigned sample=0;sample<2;++sample) {
        prototypes.markers.clear();const auto stamp=ros::Time(1+sample);
        if(robot->model_kind==RobotModelKind::kFs150) {
          xgc2_robot_visualization::UavVisualState state;state.name=identity.scene_model;state.stamp=stamp;state.pose.orientation.w=1;
          path_uav.append(state,&prototypes,nullptr,false,true,false);
        } else if(robot->model_kind==RobotModelKind::kScout) {
          xgc2_robot_visualization::UgvVisualState state;state.name=identity.scene_model;state.stamp=stamp;state.pose.orientation.w=1;
          path_scout.append(state,&prototypes,nullptr,false,true,false);
        } else {
          xgc2_robot_visualization::MecanumVisualState state;state.name=identity.scene_model;state.stamp=stamp;state.pose.orientation.w=1;
          path_mecanum.append(state,&prototypes,nullptr,false,true,false);
        }
      }
      for(const auto& marker:prototypes.markers)if(marker.type==visualization_msgs::Marker::LINE_STRIP)robot->path_marker=marker;
      robot->path_marker.points.clear();robot->path_marker.points.reserve(61);
      robot->subscribers.reserve(4);robots.push_back(std::move(robot));
    }
    descriptions.reserve(config.descriptions.size());
    for (const auto& description:config.descriptions) {
      auto kind=rateKind(modelKind(s,description.scene_model));
      // A non-scene description matching a configured slot uses that slot's kind.
      if (kind==RateKind::Global) for (const auto& robot:robots) if (robot->identity.ros_namespace==description.ros_namespace) kind=robot->kind;
      descriptions.emplace_back(new Description(description,kind));
    }
    const auto capacity=robots.size()*12+descriptions.size()*8+2;
    transforms.transforms.reserve(capacity);joints.transforms.reserve(capacity);standard_joints.transforms.reserve(capacity);statics.transforms.reserve(capacity);
    markers.markers.reserve(robots.size()*8);scratch.markers.reserve(8);scratch_tf.reserve(12);
    scene.entities.reserve(robots.size()*2);scene_ar.entities.reserve(robots.size());height.entities.reserve(robots.size());height_ar.entities.reserve(robots.size());
    height.deletions.reserve(robots.size());height_ar.deletions.reserve(robots.size());
    for(auto& row:counts) for(auto& value:row) value.store(0);
  }
  void counted(RateKind kind,Channel channel) { ++counts[static_cast<std::size_t>(kind)][static_cast<std::size_t>(channel)]; }
  bool due[ kKindCount ][ kChannelCount ]{};
  void clear() {
    transforms.transforms.clear();joints.transforms.clear();standard_joints.transforms.clear();markers.markers.clear();
    scene.entities.clear();scene.deletions.clear();scene_ar.entities.clear();scene_ar.deletions.clear();height.entities.clear();height.deletions.clear();height_ar.entities.clear();height_ar.deletions.clear();
  }
  void subscribeRobot(Robot& robot,InputPool& pool) {
    const auto& r=robot.identity;auto input=robot.input;auto node=pool.node(r.ros_namespace);
    auto topic=slotVisualizationPoseTopic(robot.model_kind,r.ros_namespace);
    const auto kind=robot.model_kind;
    const bool history=robot.path||config.settings.publish_markers||config.settings.publish_scene_paths;
    const double timeout=config.settings.pose_timeout;
    robot.subscribers.push_back(node.subscribe<geometry_msgs::PoseStamped>(topic,1,[input,kind,history,timeout](const geometry_msgs::PoseStampedConstPtr& msg) {
      if (!finite(msg->pose)) return;
      PoseValue value;value.available=true;value.pose=normalized(msg->pose);value.stamp=msg->header.stamp;
      value.frame=isWorldFixedFrame(msg->header.frame_id)?1:(msg->header.frame_id=="map"||msg->header.frame_id=="/map")?2:0;
      const bool accepted=value.frame==1||(kind==RobotModelKind::kFs150&&value.frame==2);
      const bool record=history&&accepted&&fresh(value.stamp,ros::Time::now(),timeout);
      const auto path_pose=slotHistoryPathPose(kind,value.pose);
      std::lock_guard<std::mutex> lock(input->mutex);input->state.pose=value;
      if(record)input->path.append(value.stamp,path_pose);
    },ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay()));
    if (!r.ar_pose_topic.empty()) {
      const auto offset=r.world_offset;
      const bool ar_history=robot.ar_path;
      robot.subscribers.push_back(node.subscribe<geometry_msgs::PoseStamped>(r.ar_pose_topic,1,[input,offset,ar_history](const geometry_msgs::PoseStampedConstPtr& msg) {
        if (!finite(msg->pose)) return;
        PoseValue value;value.available=true;value.frame=1;value.pose=applyExperimentWorldOffsetOnce(msg->pose,offset);value.stamp=msg->header.stamp;
        if (!finite(value.pose)) return;
        std::lock_guard<std::mutex> lock(input->mutex);input->state.ar=value;
        if(ar_history)input->ar_path.append(value.stamp,value.pose);
      },ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay()));
    }
    if (robot.model_kind==RobotModelKind::kFs150) {
      robot.subscribers.push_back(node.subscribe<mavros_msgs::State>(r.ros_namespace+"/mavros/state",1,[input](const mavros_msgs::StateConstPtr& msg) {
        auto stamp=msg->header.stamp.isZero()?ros::Time::now():msg->header.stamp;
        std::lock_guard<std::mutex> lock(input->mutex);input->state.armed=msg->armed;input->state.state_stamp=stamp;input->state.state_available=true;
      }));
      robot.subscribers.push_back(node.subscribe<mavros_msgs::ExtendedState>(r.ros_namespace+"/mavros/extended_state",1,[input](const mavros_msgs::ExtendedStateConstPtr& msg) {
        const auto now=ros::Time::now();std::lock_guard<std::mutex> lock(input->mutex);
        input->state.landed=msg->landed_state;input->state.extended_stamp=now;input->state.extended_available=true;
      }));
    } else {
      robot.subscribers.push_back(node.subscribe<geometry_msgs::Twist>(r.ros_namespace+"/cmd_vel",1,[input](const geometry_msgs::TwistConstPtr& msg) {
        if(!finite(*msg)) return;
        const auto now=ros::Time::now();std::lock_guard<std::mutex> lock(input->mutex);
        input->state.cmd=*msg;input->state.cmd_stamp=now;input->state.cmd_available=true;
      }));
      robot.subscribers.push_back(node.subscribe<geometry_msgs::TwistStamped>(r.ros_namespace+"/twist",1,[input](const geometry_msgs::TwistStampedConstPtr& msg) {
        if(!finite(msg->twist)) return;
        const auto stamp=msg->header.stamp.isZero()?ros::Time::now():msg->header.stamp;
        const bool world=isWorldFixedFrame(msg->header.frame_id);std::lock_guard<std::mutex> lock(input->mutex);
        input->state.twist=msg->twist;input->state.twist_stamp=stamp;input->state.twist_world=world;input->state.twist_available=true;
      }));
    }
    if (robot.path) robot.path_pub=node.advertise<nav_msgs::Path>(r.ros_namespace+"/"+r.path_topic,1,true);
    if (robot.ar_path) robot.ar_path_pub=node.advertise<nav_msgs::Path>(r.ros_namespace+"/"+r.ar_path_topic,1,true);
  }
  void motion(const RobotState& sample,const geometry_msgs::Pose& pose,const ros::Time& now,double* forward,double* lateral,double* angular,bool* hint) const {
    *forward=*lateral=*angular=0;*hint=false;
    const bool twist=sample.twist_available&&fresh(sample.twist_stamp,now,config.settings.motion_timeout);
    const bool cmd=sample.cmd_available&&fresh(sample.cmd_stamp,now,config.settings.motion_timeout);
    if (!twist&&!cmd) return;
    const auto& value=twist?sample.twist:sample.cmd;*hint=true;*angular=value.angular.z;
    if (twist&&sample.twist_world) { const double a=yaw(pose);*forward=value.linear.x*std::cos(a)+value.linear.y*std::sin(a);*lateral=-value.linear.x*std::sin(a)+value.linear.y*std::cos(a); }
    else { *forward=value.linear.x;*lateral=value.linear.y; }
  }
  void publishHistory(Robot& robot,const ros::Time& now,bool ar) {
    auto& revision=ar?robot.ar_path_revision:robot.path_revision;
    std::unique_lock<std::mutex> lock(robot.input->mutex);
    auto& history=ar?robot.input->ar_path:robot.input->path;
    history.expire(now);
    if(revision==history.revision)return;
    const SourceHistory snapshot=history;
    lock.unlock();
    auto& message=ar?robot.ar_path_message:robot.path_message;
    message.poses.resize(snapshot.size);message.header.stamp=snapshot.stamp;
    for(std::size_t i=0;i<snapshot.size;++i) {
      auto& point=message.poses[i];point.header.frame_id=config.settings.frame_id;
      point.header.stamp=snapshot.at(i).stamp;point.pose=snapshot.at(i).pose;
    }
    (ar?robot.ar_path_pub:robot.path_pub).publish(message);revision=snapshot.revision;
    counted(robot.kind,ar?Channel::ArPath:Channel::Path);
  }
  void appendSdk(Robot& robot,const RobotState& sample,const CanonicalWorldPose& pose,const ros::Time& now,bool mesh,bool path,bool label,bool joint) {
    scratch.markers.clear();scratch_tf.clear();const auto& s=config.settings;
    auto* marker_output=(mesh||path||label)?&scratch:nullptr;auto* tf_output=joint?&scratch_tf:nullptr;
    if(robot.model_kind==RobotModelKind::kFs150) {
      xgc2_robot_visualization::UavVisualState state;state.name=robot.identity.scene_model;state.pose=pose.pose;state.stamp=now;
      state.rotors_active=sample.state_available&&sample.armed&&fresh(sample.state_stamp,now,s.state_timeout);
      state.rotor_speed_rad_s=s.rotor_airborne;
      if(sample.extended_available&&fresh(sample.extended_stamp,now,s.state_timeout)) {
        if(sample.landed==mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) state.rotor_speed_rad_s=s.rotor_ground;
        else if(sample.landed==mavros_msgs::ExtendedState::LANDED_STATE_TAKEOFF||sample.landed==mavros_msgs::ExtendedState::LANDED_STATE_LANDING) state.rotor_speed_rad_s=s.rotor_transition;
      }
      uav->append(state,marker_output,tf_output,mesh,false,label);
    } else {
      double forward,lateral,angular;bool hint;motion(sample,pose.pose,now,&forward,&lateral,&angular,&hint);
      if(robot.model_kind==RobotModelKind::kMecanum) {
        xgc2_robot_visualization::MecanumVisualState state;state.name=robot.identity.scene_model;state.pose=pose.pose;state.stamp=now;state.has_motion_hint=hint;state.forward_velocity_m_s=forward;state.lateral_velocity_m_s=lateral;state.yaw_rate_rad_s=angular;
        mecanum->append(state,marker_output,tf_output,mesh,false,label);
      } else {
        xgc2_robot_visualization::UgvVisualState state;state.name=robot.identity.scene_model;state.pose=pose.pose;state.stamp=now;state.has_motion_hint=hint;state.forward_velocity_m_s=forward;state.yaw_rate_rad_s=angular;
        scout->append(state,marker_output,tf_output,mesh,false,label);
      }
    }
    if(marker_output) applyRobotMarkerLabel(marker_output,0,robot.model_kind,robot.identity.ros_namespace);
    if(path) {
      SourceHistory history;
      {
        std::lock_guard<std::mutex> lock(robot.input->mutex);
        robot.input->path.expire(now);history=robot.input->path;
      }
      // Preserve SDK IDs/style/frame and its two-point visibility threshold.
      if(history.size>=2) {
        auto& marker=robot.path_marker;marker.points.resize(history.size);marker.header.stamp=history.stamp;
        for(std::size_t i=0;i<history.size;++i)marker.points[i]=history.at(i).pose.position;
        scratch.markers.push_back(marker);
      }
    }
    for(const auto& value:scratch_tf) if(value.header.frame_id!=s.frame_id) joints.transforms.push_back(value);
  }
  void projectHeight(Robot& robot,const CanonicalPoseSample& canonical,const CanonicalPoseSample& ar,const ros::Time& now,bool image) {
    if(robot.model_kind!=RobotModelKind::kFs150 || robot.identity.height_projection_color.empty()) return;
    auto channel=image?Channel::HeightProjectionAr:Channel::HeightProjection;
    if(!due[0][static_cast<std::size_t>(channel)]) return;
    auto& present=image?robot.height_ar_present:robot.height_present;
    auto& stamp=image?robot.height_ar_stamp:robot.height_stamp;
    auto& update=image?height_ar:height;
    auto pose=selectUavHeightProjectionWorldPose(image?HeightProjectionView::kVrpn:HeightProjectionView::kLocalPosition,canonical,ar,now,config.settings.pose_timeout);
    if(pose.found) {
      // Build the complete current layer when any member changes; latched consumers retain siblings.
      update.entities.push_back(uavHeightProjectionEntity(robot.identity.scene_model,pose.pose.position,pose.stamp,config.settings.frame_id,sceneColorFromHex(robot.identity.height_projection_color)));
      if(!present||stamp!=pose.stamp) { if(image) height_ar_changed=true;else height_changed=true; }
      present=true;stamp=pose.stamp;
    } else if(present) {
      update.deletions.push_back(uavHeightProjectionDeletion(robot.identity.scene_model,now));present=false;
      if(image) height_ar_changed=true;else height_changed=true;
    }
  }
  std::string id;
  InstanceConfig config;
  std::vector<std::unique_ptr<Robot>> robots;
  std::vector<std::unique_ptr<Description>> descriptions;
  std::vector<std::unique_ptr<Relay>> relays;
  std::unique_ptr<xgc2_robot_visualization::Fs150UavVisualizer> uav;
  std::unique_ptr<xgc2_robot_visualization::ScoutUgvVisualizer> scout;
  std::unique_ptr<xgc2_robot_visualization::MecanumUgvVisualizer> mecanum;
  std::atomic<bool> active{false};std::atomic<unsigned> in_flight{0};
  std::mutex completion_mutex;std::condition_variable completion;
  Gate gates[kKindCount][kChannelCount];
  std::array<std::array<std::atomic<std::uint64_t>,kChannelCount>,kKindCount> counts;
  ros::Publisher tf_pub,tf_root_pub,tf_static_pub,standard_tf_pub,scene_pub,scene_ar_pub,height_pub,height_ar_pub,markers_pub,scene_ready_pub,description_ready_pub;
  std::array<ros::Publisher,4> boundary_pub;
  tf2_msgs::TFMessage transforms,joints,standard_joints,statics;
  visualization_msgs::MarkerArray markers,scratch;
  std::vector<geometry_msgs::TransformStamped> scratch_tf;
  foxglove_msgs::SceneUpdate scene,scene_ar,height,height_ar;
  std::map<std::string,foxglove_msgs::SceneEntity> scene_labels,ar_labels;
  std::set<std::pair<std::string,int>> marker_ids;
  bool static_sent{false},boundary_sent{false},scene_ready_sent{false},description_ready_sent{false},height_changed{false},height_ar_changed{false};
  bool activated{false};
};
Instance::Instance(std::string id,InstanceConfig config) : impl_(new Impl(std::move(id),std::move(config))) {}
Instance::~Instance() { deactivate(); }
const std::string& Instance::id() const { return impl_->id; }
const Json::Value& Instance::configuration() const { return impl_->config.original; }
std::set<std::string> Instance::claims() const {
  std::set<std::string> result;
  const auto& p=*impl_;const auto& s=p.config.settings;
  if(s.publish_transforms) result.insert("scene-frame-tree");
  if(s.publish_scene) result.insert("scene-global-topics");
  if(s.publish_markers&&!p.robots.empty()) result.insert("topic:/markers");
  for(const auto& robot:p.robots) if(s.publish_paths) {
    result.insert("topic:"+robot->identity.ros_namespace+"/"+robot->identity.path_topic);
    if(!robot->identity.ar_path_topic.empty()) result.insert("topic:"+robot->identity.ros_namespace+"/"+robot->identity.ar_path_topic);
  }
  for(const auto& description:p.config.descriptions) {
    result.insert("param:"+description.ros_namespace+"/visual_robot_description");
    if(description.robot_state_publisher) { result.insert("param:"+description.ros_namespace+"/robot_description");result.insert("frame-prefix:"+description.name); }
  }
  if(s.publish_transforms||!p.descriptions.empty()) result.insert("topic:/tf_static");
  for(const auto& relay:p.config.relays) result.insert("topic:"+relay.topic);
  return result;
}
void Instance::activate(InputPool& pool) {
  auto& p=*impl_;if(p.activated) throw std::logic_error("instance already activated");
  p.activated=true;ros::NodeHandle node;const auto& s=p.config.settings;
  try {
    for(auto& description:p.descriptions) description->activate(pool);
    for(auto& robot:p.robots) p.subscribeRobot(*robot,pool);
    for(const auto& relay:p.config.relays) p.relays.push_back(makeRelay(pool,relay));
    if(s.publish_transforms) {
      p.tf_pub=node.advertise<tf2_msgs::TFMessage>(s.transform_topic,10,false);
      p.tf_root_pub=node.advertise<tf2_msgs::TFMessage>("/tf",10,false);
    }
    if(s.publish_transforms||!p.descriptions.empty()) {
      p.tf_static_pub=node.advertise<tf2_msgs::TFMessage>("/tf_static",1,true);
      p.statics.transforms.push_back(algorithmOverlayFrameAlias(s.frame_id,ros::Time(0)));
      p.statics.transforms.push_back(worldFixedFrameRoot(s.frame_id,ros::Time(0)));
      for(const auto& description:p.descriptions) description->appendFixed(&p.statics.transforms);
      p.standard_tf_pub=node.advertise<tf2_msgs::TFMessage>("/tf",10,false);
    }
    if(s.publish_scene) {
      p.scene_pub=node.advertise<foxglove_msgs::SceneUpdate>(s.scene_topic,1,true);
      p.scene_ar_pub=node.advertise<foxglove_msgs::SceneUpdate>(kIdentityArTopic,1,true);
      p.height_pub=node.advertise<foxglove_msgs::SceneUpdate>(kUavHeightProjectionTopic,1,true);
      p.height_ar_pub=node.advertise<foxglove_msgs::SceneUpdate>(kUavHeightProjectionArTopic,1,true);
      const char* topics[]={kWorldBoundaryTopic,kWorldBoundaryArTopic,kWorldBoundaryWallsTopic,kWorldBoundaryWallsArTopic};
      for(std::size_t i=0;i<4;++i) p.boundary_pub[i]=node.advertise<foxglove_msgs::SceneUpdate>(topics[i],1,true);
    }
    if(s.publish_markers&&!p.robots.empty()) p.markers_pub=node.advertise<visualization_msgs::MarkerArray>("markers",1,false);
    if(s.publish_scene||s.publish_transforms||!p.robots.empty()||!p.descriptions.empty())
      p.scene_ready_pub=node.advertise<std_msgs::Empty>("/xgc/robot_scene/ready",1,true);
    if(!p.descriptions.empty()) p.description_ready_pub=node.advertise<std_msgs::Empty>("/xgc/robot_descriptions/ready",1,true);
    p.active.store(true);
  } catch(...) { deactivate();throw; }
}
void Instance::tick(const Rates& rates,const ros::Time& now) {
  auto& p=*impl_;if(!p.active.load()) return;
  ++p.in_flight;
  struct Complete {
    Impl& p;
    ~Complete() {
      // Predicate mutation and wait use the same mutex: DELETE cannot miss
      // the final tick's completion between its predicate check and sleep.
      std::lock_guard<std::mutex> lock(p.completion_mutex);
      if(p.in_flight.fetch_sub(1)==1)p.completion.notify_all();
    }
  } complete{p};
  if(!p.active.load()) return;
  for(std::size_t k=0;k<kKindCount;++k) for(std::size_t c=0;c<kChannelCount;++c)
    p.due[k][c]=applicable(static_cast<RateKind>(k),static_cast<Channel>(c))&&p.gates[k][c].take(now,rates.values[k][c]);
  p.clear();p.height_changed=p.height_ar_changed=false;const auto& s=p.config.settings;
  for(auto& record:p.robots) {
    auto& robot=*record;RobotState sample;{std::lock_guard<std::mutex> lock(robot.input->mutex);sample=robot.input->state;}
    const auto canonical=sample.pose.sample(),ar=sample.ar.sample();auto pose=selectSlotVisualizationWorldPose(robot.model_kind,canonical,now,s.pose_timeout);
    auto ar_pose=selectArIdentityWorldPose(robot.model_kind,canonical,ar,now,s.pose_timeout);
    const auto k=static_cast<std::size_t>(robot.kind);auto due=[&](Channel c){return p.due[k][static_cast<std::size_t>(c)];};
    if(robot.path&&due(Channel::Path))p.publishHistory(robot,now,false);
    if(robot.ar_path&&due(Channel::ArPath))p.publishHistory(robot,now,true);
    if(p.tf_pub&&due(Channel::PoseTf)) {
      if(pose.found&&pose.stamp!=robot.pose_tf_stamp) {
        const auto values=canonicalRobotPoseTransforms(robot.model_kind,robot.identity.scene_model,pose.pose,pose.stamp,s.frame_id,s.label_offsets);
        p.transforms.transforms.insert(p.transforms.transforms.end(),values.begin(),values.end());robot.pose_tf_stamp=pose.stamp;p.counted(robot.kind,Channel::PoseTf);
      }
      if(ar_pose.found&&ar_pose.stamp!=robot.ar_tf_stamp) {
        p.transforms.transforms.push_back(canonicalArIdentityLabelTransform(robot.model_kind,robot.identity.scene_model,ar_pose.pose,ar_pose.stamp,s.frame_id,s.label_offsets));robot.ar_tf_stamp=ar_pose.stamp;
      }
    }
    if(p.scene_ar_pub&&ar_pose.found&&!robot.ar_label_sent&&due(Channel::ArIdentity)) {
      p.scratch.markers.clear();p.scratch.markers.push_back(identityLabelMarker(robot.identity.scene_model,xgc2_robot_visualization::robotFramePrefix(robot.identity.scene_model)+"/label_ar",ar_pose.stamp));
      applyRobotMarkerLabel(&p.scratch,0,robot.model_kind,robot.identity.ros_namespace);
      foxglove_msgs::SceneUpdate update;appendSceneEntityPart(robot.model_kind,robot.identity.name,SceneEntityPart::kArLabel,p.scratch,0,ar_pose.stamp,s.frame_id,s.label_style,&update);
      if(!update.entities.empty()) {p.ar_labels[robot.identity.scene_model]=std::move(update.entities.front());robot.ar_label_sent=true;p.scene_ar.entities.push_back(p.ar_labels[robot.identity.scene_model]);p.counted(robot.kind,Channel::ArIdentity);}
    }
    p.projectHeight(robot,canonical,ar,now,false);p.projectHeight(robot,canonical,ar,now,true);
    if(!pose.found) continue;
    const bool markers=p.markers_pub&&due(Channel::Markers),joint=p.tf_pub&&due(Channel::JointTf);
    const bool label=p.scene_pub&&due(Channel::Scene)&&(!robot.label_sent||s.publish_scene_paths);
    const bool path=p.scene_pub&&s.publish_scene_paths&&due(Channel::ScenePath);
    if(markers||joint||label||path) {
      p.appendSdk(robot,sample,pose,now,markers,markers||path,markers||label,joint);
      if(markers) {for(const auto& m:p.scratch.markers)p.marker_ids.emplace(m.ns,m.id);p.markers.markers.insert(p.markers.markers.end(),p.scratch.markers.begin(),p.scratch.markers.end());p.counted(robot.kind,Channel::Markers);}
      if(joint) p.counted(robot.kind,Channel::JointTf);
      if(path) {appendSceneEntityPart(robot.model_kind,robot.identity.name,SceneEntityPart::kPath,p.scratch,0,now,s.frame_id,s.label_style,&p.scene);p.counted(robot.kind,Channel::ScenePath);}
      if(label) {
        foxglove_msgs::SceneUpdate update;appendSceneEntityPart(robot.model_kind,robot.identity.name,SceneEntityPart::kLabel,p.scratch,0,now,s.frame_id,s.label_style,&update);
        if(!update.entities.empty()) {p.scene_labels[robot.identity.scene_model]=std::move(update.entities.front());p.scene.entities.push_back(p.scene_labels[robot.identity.scene_model]);robot.label_sent=true;p.counted(robot.kind,Channel::Scene);}
      }
    }
  }
  if(!p.transforms.transforms.empty()) p.tf_pub.publish(p.transforms);
  if(!p.joints.transforms.empty()) p.tf_pub.publish(p.joints);
  if(!p.markers.markers.empty()) p.markers_pub.publish(p.markers);
  if(!p.scene.entities.empty()) {
    if(!s.publish_scene_paths) {p.scene.entities.clear();for(const auto& item:p.scene_labels)p.scene.entities.push_back(item.second);}
    p.scene_pub.publish(p.scene);
  }
  if(!p.scene_ar.entities.empty()) {p.scene_ar.entities.clear();for(const auto& item:p.ar_labels)p.scene_ar.entities.push_back(item.second);p.scene_ar_pub.publish(p.scene_ar);}
  if(p.height_changed&&p.height_pub) {p.height_pub.publish(p.height);p.counted(RateKind::Fs150,Channel::HeightProjection);}
  if(p.height_ar_changed&&p.height_ar_pub) {p.height_ar_pub.publish(p.height_ar);p.counted(RateKind::Fs150,Channel::HeightProjectionAr);}
  auto global=[&](Channel channel){return p.due[3][static_cast<std::size_t>(channel)];};
  if(p.tf_root_pub&&global(Channel::TfRoot)) {p.standard_joints.transforms.push_back(worldFixedFrameRoot(s.frame_id,now));p.tf_root_pub.publish(p.standard_joints);p.standard_joints.transforms.clear();p.counted(RateKind::Global,Channel::TfRoot);}
  if(p.tf_static_pub&&!p.static_sent&&global(Channel::TfStatic)) {p.tf_static_pub.publish(p.statics);p.static_sent=true;p.counted(RateKind::Global,Channel::TfStatic);}
  for(const auto& description:p.descriptions) if(p.due[static_cast<std::size_t>(description->kind())][static_cast<std::size_t>(Channel::JointTf)]) {
    const auto before=p.standard_joints.transforms.size();description->appendJoint(&p.standard_joints.transforms);
    if(p.standard_joints.transforms.size()!=before)p.counted(description->kind(),Channel::JointTf);
  }
  if(!p.standard_joints.transforms.empty())p.standard_tf_pub.publish(p.standard_joints);
  if(p.scene_pub&&!p.boundary_sent&&!now.isZero()&&global(Channel::WorldBoundary)) {
    auto layers=worldBoundaryLayerMessages(p.config.boundary,s.boundary_mode,now,s.frame_id);
    p.boundary_pub[0].publish(layers.ground);p.boundary_pub[1].publish(layers.ground);p.boundary_pub[2].publish(layers.walls);p.boundary_pub[3].publish(layers.walls);p.boundary_sent=true;p.counted(RateKind::Global,Channel::WorldBoundary);
  }
  if(global(Channel::Readiness)) {
    if(p.scene_ready_pub&&!p.scene_ready_sent) {p.scene_ready_pub.publish(std_msgs::Empty());p.scene_ready_sent=true;p.counted(RateKind::Global,Channel::Readiness);}
    if(p.description_ready_pub&&!p.description_ready_sent) {p.description_ready_pub.publish(std_msgs::Empty());p.description_ready_sent=true;p.counted(RateKind::Global,Channel::Readiness);}
  }
  const auto wall=std::chrono::steady_clock::now();
  for(std::size_t i=0;i<p.relays.size();++i)if(p.relays[i]->publish(rates,wall))p.counted(p.config.relays[i].kind,p.config.relays[i].channel);
}
void Instance::deactivate() {
  auto& p=*impl_;p.active.store(false);if(!p.activated)return;
  for(auto& robot:p.robots)for(auto& subscriber:robot->subscribers)subscriber.shutdown();
  {std::unique_lock<std::mutex> lock(p.completion_mutex);p.completion.wait(lock,[&]{return p.in_flight.load()==0;});}
  for(auto& relay:p.relays)relay->stop();
  // All deletions name this instance's own entities; never clear an entire topic.
  const auto now=ros::Time::now();foxglove_msgs::SceneUpdate scene,ar,height,height_ar;
  for(const auto& robot:p.robots) {
    if(!now.isZero()) {
      scene.deletions.push_back(deletion(sceneEntityPartID(robot->model_kind,robot->identity.name,SceneEntityPart::kLabel),now));
      scene.deletions.push_back(deletion(sceneEntityPartID(robot->model_kind,robot->identity.name,SceneEntityPart::kPath),now));
      ar.deletions.push_back(deletion(sceneEntityPartID(robot->model_kind,robot->identity.name,SceneEntityPart::kArLabel),now));
      height.deletions.push_back(uavHeightProjectionDeletion(robot->identity.scene_model,now));height_ar.deletions.push_back(uavHeightProjectionDeletion(robot->identity.scene_model,now));
    }
    if(robot->path_pub) {nav_msgs::Path empty;empty.header.frame_id=p.config.settings.frame_id;empty.header.stamp=now;robot->path_pub.publish(empty);}
    if(robot->ar_path_pub) {nav_msgs::Path empty;empty.header.frame_id=p.config.settings.frame_id;empty.header.stamp=now;robot->ar_path_pub.publish(empty);}
  }
  if(p.scene_pub)p.scene_pub.publish(scene);
  if(p.scene_ar_pub)p.scene_ar_pub.publish(ar);
  if(p.height_pub)p.height_pub.publish(height);
  if(p.height_ar_pub)p.height_ar_pub.publish(height_ar);
  for(std::size_t i=0;i<4;++i)if(p.boundary_pub[i]&&!now.isZero()) {foxglove_msgs::SceneUpdate update;update.deletions.push_back(i<2?worldBoundaryDeletion(now):worldWallsDeletion(now));p.boundary_pub[i].publish(update);}
  if(p.markers_pub) {visualization_msgs::MarkerArray update;for(const auto& id:p.marker_ids){visualization_msgs::Marker m;m.ns=id.first;m.id=id.second;m.action=visualization_msgs::Marker::DELETE;update.markers.push_back(m);}p.markers_pub.publish(update);}
  for(auto& description:p.descriptions)description->stop();
  for(auto& robot:p.robots){robot->path_pub.shutdown();robot->ar_path_pub.shutdown();}
  p.tf_pub.shutdown();p.tf_root_pub.shutdown();p.tf_static_pub.shutdown();p.standard_tf_pub.shutdown();p.scene_pub.shutdown();p.scene_ar_pub.shutdown();p.height_pub.shutdown();p.height_ar_pub.shutdown();p.markers_pub.shutdown();p.scene_ready_pub.shutdown();p.description_ready_pub.shutdown();for(auto& pub:p.boundary_pub)pub.shutdown();
  p.activated=false;
}
Json::Value Instance::status() const {
  const auto& p=*impl_;Json::Value result(Json::objectValue);result["ok"]=true;result["id"]=p.id;result["ready"]=p.active.load();result["robotCount"]=Json::UInt64(p.robots.size());result["descriptionCount"]=Json::UInt64(p.descriptions.size());result["relayCount"]=Json::UInt64(p.config.relays.size());
  result["configuration"]["desiredRevision"]=1;result["configuration"]["appliedRevision"]=p.active.load()?Json::Value(1):Json::Value();result["configuration"]["persistedRevision"]=Json::Value();result["configuration"]["immutable"]=true;
  for(std::size_t k=0;k<kKindCount;++k)for(std::size_t c=0;c<kChannelCount;++c)if(applicable(static_cast<RateKind>(k),static_cast<Channel>(c))) result["publicationCounters"][kindName(static_cast<RateKind>(k))][channelName(static_cast<Channel>(c))]=Json::UInt64(p.counts[k][c].load());
  std::uint64_t relay_count=0;for(const auto& relay:p.relays)relay_count+=relay->count();result["relayPublications"]=Json::UInt64(relay_count);return result;
}
} // namespace xgc2_ros_visualizer
