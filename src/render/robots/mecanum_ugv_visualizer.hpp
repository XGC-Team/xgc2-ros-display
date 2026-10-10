#pragma once

#include <deque>
#include <map>
#include <string>
#include <vector>

#include "render/robots/path_history.hpp"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/time.h>
#include <visualization_msgs/MarkerArray.h>

namespace xgc2_ros_visualizer {

// Body-frame velocities match the Mecanum Gazebo contract: +x drives forward
// and +y drives left. Keeping this distinct from UgvVisualState prevents a
// lateral Mecanum command from being rendered as an impossible Scout motion.
struct MecanumVisualState {
    std::string name;
    geometry_msgs::Pose pose;
    ros::Time stamp;
    bool has_motion_hint{false};
    double forward_velocity_m_s{0.0};
    double lateral_velocity_m_s{0.0};
    double yaw_rate_rad_s{0.0};
};

class MecanumUgvVisualizer {
  public:
    struct Config {
        std::string frame_id{"world"};
        // The Nexus STL assets are authored in millimetres; this matches the
        // 0.001 scale for the millimetre-authored mecanum_description meshes.
        double mesh_scale{1.0};
        double path_publish_rate{kDefaultPathPublishRateHz};
        double path_history_duration_sec{kDefaultPathHistoryDurationSec};
        int path_limit{0};
        double visual_wheel_radius{0.05};
        double visual_wheelbase_plus_track{0.30};
        double wheel_motion_deadband{0.02};
        double max_visual_wheel_speed_rad_s{35.0};
    };

    explicit MecanumUgvVisualizer(const Config& config);

    void append(const MecanumVisualState& state, visualization_msgs::MarkerArray* markers,
                std::vector<geometry_msgs::TransformStamped>* transforms);
    // Nullable outputs; selection changes construction, never phase/history.
    void append(const MecanumVisualState& state, visualization_msgs::MarkerArray* markers,
                std::vector<geometry_msgs::TransformStamped>* transforms,
                bool meshes, bool path, bool label);

  private:
    struct ModelVisualState {
        ros::Time last_update_stamp;
        std::deque<PathSample> path;
        std::vector<double> wheel_phases;
        geometry_msgs::Pose previous_pose;
        bool has_previous_pose{false};
    };

    struct MotionEstimate {
        double forward_velocity_m_s{0.0};
        double lateral_velocity_m_s{0.0};
        double yaw_rate_rad_s{0.0};
    };

    MotionEstimate estimateMotion(const ModelVisualState& visual, const MecanumVisualState& state, double dt) const;
    void updateWheelPhases(ModelVisualState* visual, const MecanumVisualState& state, const MotionEstimate& motion,
                           double dt) const;
    void updatePath(ModelVisualState* visual, const MecanumVisualState& state) const;
    void addBodyMarkers(const MecanumVisualState& state, visualization_msgs::MarkerArray* markers,
                        std::vector<geometry_msgs::TransformStamped>* transforms) const;
    void addWheelMarkers(const MecanumVisualState& state, const ModelVisualState& visual,
                         visualization_msgs::MarkerArray* markers,
                         std::vector<geometry_msgs::TransformStamped>* transforms) const;
    void addPathMarker(const MecanumVisualState& state, const ModelVisualState& visual,
                       visualization_msgs::MarkerArray* markers) const;
    void addLabelMarker(const MecanumVisualState& state, visualization_msgs::MarkerArray* markers) const;

    Config config_;
    std::map<std::string, ModelVisualState> models_;
};

} // namespace xgc2_ros_visualizer
