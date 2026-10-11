#pragma once
#include <json/json.h>
#include <xgc2_ros_visualizer/profile.hpp>

namespace xgc2_ros_visualizer {
// Project the already-frozen public Robot/context/panel values into the desired
// state of one instance: one profile per Robot that has an installed
// description, plus what the robots share. LocalizationSources is
// authoritative: this function never resolves a profile or guesses a source.
// A Robot's `visualization.profile` is its saved profile; it is applied over
// the values derived from the Robot facts and the panel settings.
InstanceSpec projectInstanceInput(const Json::Value& value);
} // namespace xgc2_ros_visualizer
