#pragma once
#include <json/json.h>
#include <string>
namespace xgc2_ros_visualizer {
// Project the already-frozen public Robot/context values. LocalizationSources
// is authoritative: this function never resolves a profile or guesses a source.
// The instance identity belongs to the native caller, independently of this data.
Json::Value projectInstanceInput(const Json::Value& value);
} // namespace xgc2_ros_visualizer
