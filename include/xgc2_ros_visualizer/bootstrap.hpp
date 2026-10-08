#pragma once
#include <json/json.h>
#include <string>
namespace xgc2_ros_visualizer {
struct Bootstrap {
  std::string instance_id;
  Json::Value request;
};
// Project the already-frozen public Robot/context values. LocalizationSources
// is authoritative: this function never resolves a profile or guesses a source.
Bootstrap projectBootstrap(const Json::Value& value);
Bootstrap readBootstrap(const std::string& file);
} // namespace xgc2_ros_visualizer
