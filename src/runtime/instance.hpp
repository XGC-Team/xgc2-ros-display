#pragma once
#include <memory>
#include <set>
#include <string>
#include <json/json.h>
#include <ros/ros.h>
#include <xgc2_ros_visualizer/config.hpp>
#include "input_pool.hpp"
namespace xgc2_ros_visualizer {
class Instance {
 public:
  Instance(std::string id, InstanceConfig config);
  ~Instance();
  const std::string& id() const;
  const Json::Value& configuration() const;
  std::set<std::string> claims() const;
  void activate(InputPool& pool);
  void tick(const Rates& rates, const ros::Time& now);
  void deactivate();
  Json::Value status() const;
 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace xgc2_ros_visualizer
