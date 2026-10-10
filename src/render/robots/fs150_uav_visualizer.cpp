#include "render/robots/fs150_uav_visualizer.hpp"

#include "render/robots/robot_frames.hpp"
#include <fs150_description/visual_geometry.hpp>

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

constexpr const char* kFs150BodyMesh = fs150_description::kBodyMesh;

struct RotorVisual {
    const char* name;
    geometry_msgs::Vector3 offset;
    const char* mesh;
    double direction;
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

std_msgs::ColorRGBA makeColor(double r, double g, double b, double a) {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

const std::vector<RotorVisual>& fs150Rotors() {
    static const std::vector<RotorVisual> rotors = [] {
        std::vector<RotorVisual> result;
        for (const auto& rotor : fs150_description::kRotors) {
            result.push_back({rotor.name, makeVector3(rotor.x, rotor.y, rotor.z), rotor.mesh, rotor.direction});
        }
        return result;
    }();
    return rotors;
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

geometry_msgs::Quaternion yawQuaternion(double yaw) {
    return makeQuaternion(0.0, 0.0, std::sin(0.5 * yaw), std::cos(0.5 * yaw));
}

geometry_msgs::Vector3 rotateVector(const geometry_msgs::Quaternion& q, const geometry_msgs::Vector3& v) {
    const geometry_msgs::Quaternion qn = normalize(q);
    const geometry_msgs::Quaternion vq = makeQuaternion(v.x, v.y, v.z, 0.0);
    const geometry_msgs::Quaternion qi = makeQuaternion(-qn.x, -qn.y, -qn.z, qn.w);
    const geometry_msgs::Quaternion out = multiplyRaw(multiplyRaw(qn, vq), qi);
    return makeVector3(out.x, out.y, out.z);
}

visualization_msgs::Marker makeMeshMarker(const std::string& ns, int id, const std::string& mesh,
                                          const std::string& frame_id, const geometry_msgs::Pose& pose,
                                          const ros::Time& stamp, const std_msgs::ColorRGBA& color, double scale) {
    visualization_msgs::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.mesh_resource = mesh;
    marker.mesh_use_embedded_materials = true;
    marker.pose = pose;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color = color;
    return marker;
}

// The label anchor travels with the robot and ignores its attitude, so a banked
// aircraft keeps its name directly overhead instead of swinging it out to the
// side. Height is the same 0.55 m the label used when it carried absolute
// world coordinates, so nothing moves on screen.
geometry_msgs::Pose labelAnchor(const geometry_msgs::Pose& pose) {
    geometry_msgs::Pose anchor;
    anchor.position = makePoint(pose.position.x, pose.position.y, pose.position.z + 0.55);
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

std::string trailingNumber(const std::string& name) {
    static const std::regex pattern("([0-9]+)$");
    std::smatch match;
    return std::regex_search(name, match, pattern) ? match.str(1) : std::string();
}

std::string displayName(const UavVisualState& state) {
    const std::string number = trailingNumber(state.name);
    return "UAV " + (number.empty() ? state.name : number);
}

} // namespace

Fs150UavVisualizer::Fs150UavVisualizer(const Config& config) : config_(config) {
    applyPathHistoryConfig(&config_.path_publish_rate, &config_.path_history_duration_sec, &config_.path_limit);
    config_.mesh_scale = std::max(0.001, config_.mesh_scale);
    config_.rotor_speed_rad_s = std::max(0.0, config_.rotor_speed_rad_s);
}

void Fs150UavVisualizer::append(const UavVisualState& state, visualization_msgs::MarkerArray* markers,
                                std::vector<geometry_msgs::TransformStamped>* transforms) {
    append(state, markers, transforms, true, true, true);
}

void Fs150UavVisualizer::append(const UavVisualState& state, visualization_msgs::MarkerArray* markers,
                                std::vector<geometry_msgs::TransformStamped>* transforms,
                                bool meshes, bool path, bool label) {
    ModelVisualState& visual = models_[state.name];
    if (visual.rotor_phases.size() != fs150Rotors().size()) {
        visual.rotor_phases.assign(fs150Rotors().size(), 0.0);
    }

    updateRotorPhases(&visual, state);
    updatePath(&visual, state);

    if (transforms != nullptr) {
        transforms->push_back(
            makeTransform(config_.frame_id, robotBodyFrame(state.name), state.pose, state.stamp));
        transforms->push_back(makeTransform(config_.frame_id, robotLabelFrame(state.name),
                                            labelAnchor(state.pose), state.stamp));
        geometry_msgs::Pose camera_pose;
        camera_pose.position = makePoint(fs150_description::kCameraX, fs150_description::kCameraY,
                                         fs150_description::kCameraZ);
        camera_pose.orientation = makeQuaternion(0, 0, 0, 1);
        const std::string camera_frame = robotFramePrefix(state.name) + "/camera_link";
        transforms->push_back(makeTransform(robotBodyFrame(state.name), camera_frame, camera_pose, state.stamp));
        geometry_msgs::Pose optical_pose;
        optical_pose.orientation = makeQuaternion(-0.5, 0.5, -0.5, 0.5);
        transforms->push_back(makeTransform(camera_frame, robotFramePrefix(state.name) + "/camera_optical_frame",
                                            optical_pose, state.stamp));
    }
    if (markers != nullptr && meshes) addBodyMarker(state, markers);
    addRotorMarkers(state, visual, meshes ? markers : nullptr, transforms);
    if (markers != nullptr && path) addPathMarker(state, visual, markers);
    if (markers != nullptr && label) addLabelMarker(state, markers);
}

void Fs150UavVisualizer::updateRotorPhases(ModelVisualState* visual, const UavVisualState& state) const {
    const double dt =
        visual->last_update_stamp.isZero() ? 0.0 : std::max(0.0, (state.stamp - visual->last_update_stamp).toSec());
    const double rotor_speed =
        state.rotor_speed_rad_s > 0.0 ? state.rotor_speed_rad_s : config_.rotor_speed_rad_s;
    if (state.rotors_active && dt > 0.0) {
        const std::vector<RotorVisual>& rotors = fs150Rotors();
        for (std::size_t i = 0; i < rotors.size(); ++i) {
            visual->rotor_phases[i] =
                std::fmod(visual->rotor_phases[i] + rotors[i].direction * rotor_speed * dt,
                          2.0 * M_PI);
        }
    }
    visual->last_update_stamp = state.stamp;
}

void Fs150UavVisualizer::updatePath(ModelVisualState* visual, const UavVisualState& state) const {
    pushPathHistory(&visual->path, state.stamp, state.pose.position, config_.path_publish_rate,
                    config_.path_history_duration_sec, config_.path_limit);
}

void Fs150UavVisualizer::addBodyMarker(const UavVisualState& state, visualization_msgs::MarkerArray* markers) const {
    markers->markers.push_back(makeMeshMarker(state.name + "_body", 0, kFs150BodyMesh, config_.frame_id, state.pose,
                                              state.stamp, makeColor(1.0, 1.0, 1.0, 1.0), config_.mesh_scale));
}

void Fs150UavVisualizer::addRotorMarkers(const UavVisualState& state, const ModelVisualState& visual,
                                         visualization_msgs::MarkerArray* markers,
                                         std::vector<geometry_msgs::TransformStamped>* transforms) const {
    if (markers == nullptr && transforms == nullptr) return;
    const std::vector<RotorVisual>& rotors = fs150Rotors();
    for (std::size_t i = 0; i < rotors.size(); ++i) {
        const RotorVisual& rotor = rotors[i];
        const double phase = visual.rotor_phases[i];

        geometry_msgs::Pose rotor_relative_pose;
        rotor_relative_pose.position = makePoint(rotor.offset.x, rotor.offset.y, rotor.offset.z);
        rotor_relative_pose.orientation = yawQuaternion(phase);
        if (transforms != nullptr) {
            transforms->push_back(
                makeTransform(robotBodyFrame(state.name), robotFramePrefix(state.name) + "/" + rotor.name, rotor_relative_pose, state.stamp));
        }

        if (markers != nullptr) {
            geometry_msgs::Pose rotor_pose;
            const geometry_msgs::Vector3 offset = rotateVector(state.pose.orientation, rotor.offset);
            rotor_pose.position = makePoint(state.pose.position.x + offset.x, state.pose.position.y + offset.y,
                                            state.pose.position.z + offset.z);
            rotor_pose.orientation = multiply(state.pose.orientation, yawQuaternion(phase));
            markers->markers.push_back(makeMeshMarker(state.name + "_" + rotor.name, static_cast<int>(i) + 1, rotor.mesh,
                                                      config_.frame_id, rotor_pose, state.stamp, makeColor(1.0, 1.0, 1.0, 1.0),
                                                      config_.mesh_scale));
        }
    }
}

void Fs150UavVisualizer::addPathMarker(const UavVisualState& state, const ModelVisualState& visual,
                                       visualization_msgs::MarkerArray* markers) const {
    if (visual.path.size() < 2) {
        return;
    }

    visualization_msgs::Marker marker;
    marker.header.stamp = state.stamp;
    marker.header.frame_id = config_.frame_id;
    marker.ns = state.name + "_actual_path";
    marker.id = 10;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    assignPathHistoryPoints(visual.path, &marker.points);
    marker.scale.x = 0.018;
    marker.color.r = 0.0;
    marker.color.g = 0.55;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    markers->markers.push_back(std::move(marker));
}

void Fs150UavVisualizer::addLabelMarker(const UavVisualState& state, visualization_msgs::MarkerArray* markers) const {
    visualization_msgs::Marker marker;
    marker.header.stamp = state.stamp;
    // The label sits at the origin of its own frame. The offset that used to be
    // added here now lives in that frame's transform, so the label follows the
    // robot at the transform rate rather than only when this marker is resent.
    marker.header.frame_id = robotLabelFrame(state.name);
    marker.ns = state.name + "_label";
    marker.id = 11;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = makePoint(0.0, 0.0, 0.0);
    marker.pose.orientation = makeQuaternion(0.0, 0.0, 0.0, 1.0);
    marker.scale.z = 0.32;
    marker.color = makeColor(1.0, 1.0, 1.0, 1.0);
    marker.text = displayName(state);
    markers->markers.push_back(marker);
}

} // namespace xgc2_ros_visualizer
