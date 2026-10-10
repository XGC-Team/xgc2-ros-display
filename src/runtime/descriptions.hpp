#pragma once
#include <memory>
#include <string>
#include <vector>
#include <geometry_msgs/TransformStamped.h>
#include <xgc2_ros_visualizer/config.hpp>
#include "input_pool.hpp"
namespace xgc2_ros_visualizer {
// One cold URDF record, with one latest JointState handle. No RSP process/host.
class Description {
 public:
  Description(xgc2_ros_visualizer::RobotDescription robot,RateKind kind);
  ~Description();
  const std::string& parameter() const;
  RateKind kind() const;
  bool statePublisher() const;
  void activate(InputPool& pool);
  void appendFixed(std::vector<geometry_msgs::TransformStamped>* output) const;
  void appendJoint(std::vector<geometry_msgs::TransformStamped>* output);
  void stop();
 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace xgc2_ros_visualizer
