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

struct UavVisualState {
    std::string name;
    geometry_msgs::Pose pose;
    bool rotors_active{false};
    // Spin rate for this sample, in rad/s. Zero means "use the configured
    // default". A caller supplies it to reflect a flight-state tier -- parked,
    // climbing, airborne -- without this package learning what a flight state
    // is. The animation only has to say which of those the vehicle is in.
    double rotor_speed_rad_s{0.0};
    ros::Time stamp;
};

class Fs150UavVisualizer {
  public:
    struct Config {
        std::string frame_id{"world"};
        double rotor_speed_rad_s{70.0};
        double mesh_scale{1.0};
        double path_publish_rate{kDefaultPathPublishRateHz};
        double path_history_duration_sec{kDefaultPathHistoryDurationSec};
        int path_limit{0};
    };

    explicit Fs150UavVisualizer(const Config& config);

    void append(const UavVisualState& state, visualization_msgs::MarkerArray* markers,
                std::vector<geometry_msgs::TransformStamped>* transforms);
    // Nullable outputs; selection changes construction, never phase/history.
    void append(const UavVisualState& state, visualization_msgs::MarkerArray* markers,
                std::vector<geometry_msgs::TransformStamped>* transforms,
                bool meshes, bool path, bool label);

  private:
    struct ModelVisualState {
        ros::Time last_update_stamp;
        std::deque<PathSample> path;
        std::vector<double> rotor_phases;
    };

    void updateRotorPhases(ModelVisualState* visual, const UavVisualState& state) const;
    void updatePath(ModelVisualState* visual, const UavVisualState& state) const;
    void addBodyMarker(const UavVisualState& state, visualization_msgs::MarkerArray* markers) const;
    void addRotorMarkers(const UavVisualState& state, const ModelVisualState& visual,
                         visualization_msgs::MarkerArray* markers,
                         std::vector<geometry_msgs::TransformStamped>* transforms) const;
    void addPathMarker(const UavVisualState& state, const ModelVisualState& visual,
                       visualization_msgs::MarkerArray* markers) const;
    void addLabelMarker(const UavVisualState& state, visualization_msgs::MarkerArray* markers) const;

    Config config_;
    std::map<std::string, ModelVisualState> models_;
};

} // namespace xgc2_ros_visualizer
