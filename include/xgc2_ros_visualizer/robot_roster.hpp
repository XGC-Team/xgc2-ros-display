#pragma once

#include <string>
#include <array>
#include <vector>

namespace xgc2_ros_visualizer {

// RobotDescription is the kind-neutral, frozen input shared by the managed
// description and scene runtimes. Core has already projected every field from
// the immutable Experiment Robot roster; no runtime guesses a namespace,
// scene identity, or public Path topic.
struct RobotDescription {
    std::string name;
    std::string ros_namespace;
    std::string description_package;
    std::string description_file;
    bool robot_state_publisher;
    std::string joint_state_topic;
    std::string scene_model;
    std::string odometry_topic;
    std::string path_topic;
    std::string height_projection_color;
    std::string ar_pose_topic;
    std::string ar_path_topic;
    std::array<double, 3> world_offset{{0.0, 0.0, 0.0}};
};

// readRobotVisualizationRoster strictly decodes the canonical JSON array
// supplied to the native visualization service. On failure, robots is left
// unchanged so callers cannot accidentally run a partially admitted fleet.
bool readRobotVisualizationRoster(const std::string& raw,
                                  std::vector<RobotDescription>* robots,
                                  std::string* error);

}  // namespace xgc2_ros_visualizer
