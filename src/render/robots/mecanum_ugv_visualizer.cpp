#include "render/robots/mecanum_ugv_visualizer.hpp"

#include "render/robots/robot_frames.hpp"

#include <algorithm>
#include <cmath>
#include <regex>
#include <string>
#include <utility>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

namespace xgc2_ros_visualizer {
namespace {

constexpr const char* kBaseMesh = "package://mecanum_description/meshes/lod10k/nexus_base_link.dae";
constexpr const char* kShaftMesh = "package://mecanum_description/meshes/lod10k/wheel_shaft.dae";
constexpr const char* kLeftWheelMesh =
    "package://mecanum_description/meshes/lod10k/mecanum_wheel_left.dae";
constexpr const char* kRightWheelMesh =
    "package://mecanum_description/meshes/lod10k/mecanum_wheel_right.dae";
constexpr const char* kRangeSensorMesh = "package://mecanum_description/meshes/lod10k/urm04.dae";

struct MeshPart {
    const char* name;
    const char* mesh;
    geometry_msgs::Pose pose;
};

struct WheelVisual {
    const char* name;
    const char* mesh;
    geometry_msgs::Pose pose;
};

geometry_msgs::Vector3 makeVector3(double x, double y, double z) {
    geometry_msgs::Vector3 out;
    out.x = x;
    out.y = y;
    out.z = z;
    return out;
}

geometry_msgs::Point makePoint(double x, double y, double z) {
    geometry_msgs::Point out;
    out.x = x;
    out.y = y;
    out.z = z;
    return out;
}

geometry_msgs::Quaternion makeQuaternion(double x, double y, double z, double w) {
    geometry_msgs::Quaternion out;
    out.x = x;
    out.y = y;
    out.z = z;
    out.w = w;
    return out;
}

geometry_msgs::Quaternion normalize(const geometry_msgs::Quaternion& q) {
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(norm) || norm < 1.0e-9) {
        return makeQuaternion(0.0, 0.0, 0.0, 1.0);
    }
    return makeQuaternion(q.x / norm, q.y / norm, q.z / norm, q.w / norm);
}

geometry_msgs::Quaternion multiply(const geometry_msgs::Quaternion& lhs, const geometry_msgs::Quaternion& rhs) {
    const geometry_msgs::Quaternion a = normalize(lhs);
    const geometry_msgs::Quaternion b = normalize(rhs);
    return makeQuaternion(a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                          a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
}

geometry_msgs::Quaternion multiplyRaw(const geometry_msgs::Quaternion& a, const geometry_msgs::Quaternion& b) {
    return makeQuaternion(a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                          a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
}

geometry_msgs::Quaternion rpyQuaternion(double roll, double pitch, double yaw) {
    const double cr = std::cos(0.5 * roll);
    const double sr = std::sin(0.5 * roll);
    const double cp = std::cos(0.5 * pitch);
    const double sp = std::sin(0.5 * pitch);
    const double cy = std::cos(0.5 * yaw);
    const double sy = std::sin(0.5 * yaw);
    return makeQuaternion(sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                          cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy);
}

geometry_msgs::Pose makePose(double x, double y, double z, double roll, double pitch, double yaw) {
    geometry_msgs::Pose pose;
    pose.position = makePoint(x, y, z);
    pose.orientation = rpyQuaternion(roll, pitch, yaw);
    return pose;
}

geometry_msgs::Vector3 rotateVector(const geometry_msgs::Quaternion& q, const geometry_msgs::Vector3& v) {
    const geometry_msgs::Quaternion qn = normalize(q);
    const geometry_msgs::Quaternion vq = makeQuaternion(v.x, v.y, v.z, 0.0);
    const geometry_msgs::Quaternion qi = makeQuaternion(-qn.x, -qn.y, -qn.z, qn.w);
    const geometry_msgs::Quaternion out = multiplyRaw(multiplyRaw(qn, vq), qi);
    return makeVector3(out.x, out.y, out.z);
}

geometry_msgs::Pose composePose(const geometry_msgs::Pose& parent, const geometry_msgs::Pose& child) {
    geometry_msgs::Pose out;
    const geometry_msgs::Vector3 child_translation = makeVector3(child.position.x, child.position.y, child.position.z);
    const geometry_msgs::Vector3 rotated = rotateVector(parent.orientation, child_translation);
    out.position = makePoint(parent.position.x + rotated.x, parent.position.y + rotated.y, parent.position.z + rotated.z);
    out.orientation = multiply(parent.orientation, child.orientation);
    return out;
}

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
    const geometry_msgs::Quaternion normalized = normalize(q);
    const double siny_cosp = 2.0 * (normalized.w * normalized.z + normalized.x * normalized.y);
    const double cosy_cosp = 1.0 - 2.0 * (normalized.y * normalized.y + normalized.z * normalized.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

double shortestAngleDelta(double from, double to) {
    return std::atan2(std::sin(to - from), std::cos(to - from));
}

double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

std_msgs::ColorRGBA makeColor(double r, double g, double b, double a) {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

visualization_msgs::Marker makeMeshMarker(const std::string& ns, int id, const std::string& mesh,
                                          const std::string& frame_id, const geometry_msgs::Pose& pose,
                                          const ros::Time& stamp, const std_msgs::ColorRGBA& color, double scale,
                                          bool use_embedded_materials) {
    visualization_msgs::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.mesh_resource = mesh;
    marker.mesh_use_embedded_materials = use_embedded_materials;
    marker.pose = pose;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color = color;
    return marker;
}

// The label anchor travels with the robot and ignores its attitude, so a banked
// aircraft keeps its name directly overhead instead of swinging it out to the
// side. Height is the same 0.32 m the label used when it carried absolute
// world coordinates, so nothing moves on screen.
geometry_msgs::Pose labelAnchor(const geometry_msgs::Pose& pose) {
    geometry_msgs::Pose anchor;
    anchor.position = makePoint(pose.position.x, pose.position.y, pose.position.z + 0.32);
    anchor.orientation = makeQuaternion(0.0, 0.0, 0.0, 1.0);
    return anchor;
}

geometry_msgs::TransformStamped makeTransform(const std::string& parent_frame, const std::string& child_frame,
                                              const geometry_msgs::Pose& pose, const ros::Time& stamp) {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    transform.transform.translation = makeVector3(pose.position.x, pose.position.y, pose.position.z);
    transform.transform.rotation = pose.orientation;
    return transform;
}

const std::vector<MeshPart>& fixedParts() {
    static const std::vector<MeshPart> parts = {
        {"base_link", kBaseMesh, makePose(0.0, 0.0, 0.05, 0.0, 0.0, 0.0)},
        {"upper_left_wheel_shaft", kShaftMesh, makePose(0.150, 0.108, 0.05, 0.0, 0.0, 0.0)},
        {"upper_right_wheel_shaft", kShaftMesh, makePose(0.150, -0.108, 0.05, M_PI, 0.0, 0.0)},
        {"lower_left_wheel_shaft", kShaftMesh, makePose(-0.150, 0.108, 0.05, 0.0, 0.0, 0.0)},
        {"lower_right_wheel_shaft", kShaftMesh, makePose(-0.150, -0.108, 0.05, M_PI, 0.0, 0.0)},
        {"front_sensor", kRangeSensorMesh, makePose(0.205, 0.0, 0.06, M_PI / 2.0, 0.0, 0.0)},
        {"left_sensor", kRangeSensorMesh, makePose(0.0, 0.108, 0.06, M_PI / 2.0, 0.0, M_PI / 2.0)},
        {"right_sensor", kRangeSensorMesh, makePose(0.0, -0.108, 0.06, M_PI / 2.0, 0.0, -M_PI / 2.0)},
        {"rear_sensor", kRangeSensorMesh, makePose(-0.201, 0.0, 0.06, M_PI / 2.0, 0.0, M_PI)},
    };
    return parts;
}

// Preserve the SSS visualization order and signs: UL, UR, LL, LR. The phases
// are reconstructed from body motion and never depend on simulator joints.
const std::vector<WheelVisual>& mecanumWheels() {
    static const std::vector<WheelVisual> wheels = {
        {"upper_left_wheel", kLeftWheelMesh, makePose(0.150, 0.150, 0.05, 0.0, 0.0, 0.0)},
        {"upper_right_wheel", kRightWheelMesh, makePose(0.150, -0.150, 0.05, M_PI, 0.0, 0.0)},
        {"lower_left_wheel", kRightWheelMesh, makePose(-0.150, 0.150, 0.05, 0.0, 0.0, 0.0)},
        {"lower_right_wheel", kLeftWheelMesh, makePose(-0.150, -0.150, 0.05, M_PI, 0.0, 0.0)},
    };
    return wheels;
}

std::string trailingNumber(const std::string& name) {
    static const std::regex pattern("([0-9]+)$");
    std::smatch match;
    return std::regex_search(name, match, pattern) ? match.str(1) : std::string();
}

std::string displayName(const MecanumVisualState& state) {
    const std::string number = trailingNumber(state.name);
    return "UGV " + (number.empty() ? state.name : number);
}

} // namespace

MecanumUgvVisualizer::MecanumUgvVisualizer(const Config& config) : config_(config) {
    applyPathHistoryConfig(&config_.path_publish_rate, &config_.path_history_duration_sec, &config_.path_limit);
    config_.mesh_scale = std::max(0.001, config_.mesh_scale);
    config_.visual_wheel_radius = std::max(0.001, config_.visual_wheel_radius);
    config_.visual_wheelbase_plus_track = std::max(0.001, config_.visual_wheelbase_plus_track);
    config_.wheel_motion_deadband = std::max(0.0, config_.wheel_motion_deadband);
    config_.max_visual_wheel_speed_rad_s = std::max(0.0, config_.max_visual_wheel_speed_rad_s);
}

void MecanumUgvVisualizer::append(const MecanumVisualState& state, visualization_msgs::MarkerArray* markers,
                                  std::vector<geometry_msgs::TransformStamped>* transforms) {
    append(state, markers, transforms, true, true, true);
}

void MecanumUgvVisualizer::append(const MecanumVisualState& state, visualization_msgs::MarkerArray* markers,
                                  std::vector<geometry_msgs::TransformStamped>* transforms,
                                bool meshes, bool path, bool label) {
    MecanumVisualState display = state;
    display.pose = placeGroundVehicleBodyPose(state.pose, mecanumDisplayBodyZ());
    ModelVisualState& visual = models_[display.name];
    if (visual.wheel_phases.size() != mecanumWheels().size()) {
        visual.wheel_phases.assign(mecanumWheels().size(), 0.0);
    }

    const double dt =
        visual.last_update_stamp.isZero() ? 0.0 : std::max(0.0, (state.stamp - visual.last_update_stamp).toSec());
    const MotionEstimate motion = estimateMotion(visual, state, dt);
    updateWheelPhases(&visual, state, motion, dt);
    updatePath(&visual, state);
    visual.previous_pose = display.pose;
    visual.has_previous_pose = true;
    visual.last_update_stamp = display.stamp;

    if (transforms != nullptr) {
        transforms->push_back(
            makeTransform(config_.frame_id, robotBodyFrame(display.name), display.pose, display.stamp));
        transforms->push_back(makeTransform(config_.frame_id, robotLabelFrame(display.name),
                                            labelAnchor(display.pose), display.stamp));
    }
    addBodyMarkers(display, meshes ? markers : nullptr, transforms);
    addWheelMarkers(display, visual, meshes ? markers : nullptr, transforms);
    if (markers != nullptr && path) addPathMarker(display, visual, markers);
    if (markers != nullptr && label) addLabelMarker(display, markers);
}

MecanumUgvVisualizer::MotionEstimate
MecanumUgvVisualizer::estimateMotion(const ModelVisualState& visual, const MecanumVisualState& state, double dt) const {
    MotionEstimate motion;
    if (state.has_motion_hint) {
        motion.forward_velocity_m_s = state.forward_velocity_m_s;
        motion.lateral_velocity_m_s = state.lateral_velocity_m_s;
        motion.yaw_rate_rad_s = state.yaw_rate_rad_s;
        return motion;
    }
    if (!visual.has_previous_pose || dt <= 0.0) {
        return motion;
    }

    const double current_yaw = yawFromQuaternion(state.pose.orientation);
    const double previous_yaw = yawFromQuaternion(visual.previous_pose.orientation);
    const double vx = (state.pose.position.x - visual.previous_pose.position.x) / dt;
    const double vy = (state.pose.position.y - visual.previous_pose.position.y) / dt;
    motion.forward_velocity_m_s = vx * std::cos(current_yaw) + vy * std::sin(current_yaw);
    motion.lateral_velocity_m_s = -vx * std::sin(current_yaw) + vy * std::cos(current_yaw);
    motion.yaw_rate_rad_s = shortestAngleDelta(previous_yaw, current_yaw) / dt;
    return motion;
}

void MecanumUgvVisualizer::updateWheelPhases(ModelVisualState* visual, const MecanumVisualState&,
                                             const MotionEstimate& motion, double dt) const {
    if (visual == nullptr || dt <= 0.0 || config_.max_visual_wheel_speed_rad_s <= 0.0) {
        return;
    }
    const double coupling = motion.yaw_rate_rad_s * config_.visual_wheelbase_plus_track;
    const double front = motion.forward_velocity_m_s;
    const double left = motion.lateral_velocity_m_s;
    const double wheel_speeds[] = {
        (front - left - coupling) / config_.visual_wheel_radius,
        -(front + left + coupling) / config_.visual_wheel_radius,
        (front + left - coupling) / config_.visual_wheel_radius,
        -(front - left + coupling) / config_.visual_wheel_radius,
    };
    for (std::size_t index = 0; index < visual->wheel_phases.size(); ++index) {
        if (std::abs(wheel_speeds[index]) < config_.wheel_motion_deadband) {
            continue;
        }
        const double wheel_speed = clamp(wheel_speeds[index], -config_.max_visual_wheel_speed_rad_s,
                                         config_.max_visual_wheel_speed_rad_s);
        visual->wheel_phases[index] = std::fmod(visual->wheel_phases[index] + wheel_speed * dt, 2.0 * M_PI);
    }
}

void MecanumUgvVisualizer::updatePath(ModelVisualState* visual, const MecanumVisualState& state) const {
    if (visual == nullptr) {
        return;
    }
    pushPathHistory(&visual->path, state.stamp, flattenGroundVehicleHistoryPoint(state.pose.position),
                    config_.path_publish_rate, config_.path_history_duration_sec, config_.path_limit);
}

void MecanumUgvVisualizer::addBodyMarkers(const MecanumVisualState& state, visualization_msgs::MarkerArray* markers,
                                          std::vector<geometry_msgs::TransformStamped>* transforms) const {
    if (markers == nullptr && transforms == nullptr) return;
    const std::vector<MeshPart>& parts = fixedParts();
    for (std::size_t index = 0; index < parts.size(); ++index) {
        const MeshPart& part = parts[index];
        const bool body = index == 0U;
        if (markers != nullptr) {
            const geometry_msgs::Pose world_pose = composePose(state.pose, part.pose);
            markers->markers.push_back(makeMeshMarker(state.name + "_mecanum_" + part.name, static_cast<int>(index),
                                                      part.mesh, config_.frame_id, world_pose, state.stamp,
                                                      body ? makeColor(0.9, 0.8, 0.6, 1.0) : makeColor(0.2, 0.2, 0.2, 1.0),
                                                      config_.mesh_scale, false));
        }
        if (!body) {
            if (transforms != nullptr) {
                transforms->push_back(makeTransform(robotBodyFrame(state.name), robotFramePrefix(state.name) + "/" + part.name, part.pose,
                                                    state.stamp));
            }
        }
    }
}

void MecanumUgvVisualizer::addWheelMarkers(const MecanumVisualState& state, const ModelVisualState& visual,
                                           visualization_msgs::MarkerArray* markers,
                                           std::vector<geometry_msgs::TransformStamped>* transforms) const {
    if (markers == nullptr && transforms == nullptr) return;
    const std::vector<WheelVisual>& wheels = mecanumWheels();
    for (std::size_t index = 0; index < wheels.size(); ++index) {
        const WheelVisual& wheel = wheels[index];
        geometry_msgs::Pose wheel_pose = wheel.pose;
        wheel_pose.orientation = multiply(wheel.pose.orientation, rpyQuaternion(0.0, visual.wheel_phases[index], 0.0));
        if (markers != nullptr) {
            markers->markers.push_back(makeMeshMarker(state.name + "_mecanum_" + wheel.name,
                                                      static_cast<int>(fixedParts().size() + index), wheel.mesh,
                                                      config_.frame_id, composePose(state.pose, wheel_pose), state.stamp,
                                                      makeColor(0.6, 0.6, 0.6, 1.0), config_.mesh_scale, false));
        }
        if (transforms != nullptr) {
            transforms->push_back(makeTransform(robotBodyFrame(state.name), robotFramePrefix(state.name) + "/" + wheel.name, wheel_pose,
                                                state.stamp));
        }
    }
}

void MecanumUgvVisualizer::addPathMarker(const MecanumVisualState& state, const ModelVisualState& visual,
                                         visualization_msgs::MarkerArray* markers) const {
    if (visual.path.size() < 2U) {
        return;
    }
    visualization_msgs::Marker marker;
    marker.header.stamp = state.stamp;
    marker.header.frame_id = config_.frame_id;
    marker.ns = state.name + "_actual_path";
    marker.id = 20;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.025;
    marker.color = makeColor(0.95, 0.65, 0.1, 0.95);
    assignPathHistoryPoints(visual.path, &marker.points);
    markers->markers.push_back(std::move(marker));
}

void MecanumUgvVisualizer::addLabelMarker(const MecanumVisualState& state,
                                          visualization_msgs::MarkerArray* markers) const {
    visualization_msgs::Marker marker;
    marker.header.stamp = state.stamp;
    // The label sits at the origin of its own frame; the height offset lives
    // in that frame's transform, so it tracks the robot at the transform rate.
    marker.header.frame_id = robotLabelFrame(state.name);
    marker.ns = state.name + "_label";
    marker.id = 21;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = makePoint(0.0, 0.0, 0.0);
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.18;
    marker.color = makeColor(1.0, 0.85, 0.25, 1.0);
    marker.text = displayName(state);
    markers->markers.push_back(marker);
}

} // namespace xgc2_ros_visualizer
