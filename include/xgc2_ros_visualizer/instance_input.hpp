#pragma once
#include <json/json.h>
#include <string>
namespace xgc2_ros_visualizer {
// Project the already-frozen public Robot/context values. LocalizationSources
// is authoritative: this function never resolves a profile or guesses a source.
// The instance identity and immutable ROS clock belong to the native caller.
Json::Value projectInstanceInput(const Json::Value& value,bool simulation_clock);
} // namespace xgc2_ros_visualizer
