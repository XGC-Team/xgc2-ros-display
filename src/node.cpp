#include <xgc2_ros_display_relays/display_relays.hpp>
#include <ros/ros.h>
#include <csignal>
#include <chrono>
#include <thread>
#include <iostream>
#include <stdexcept>
#include <utility>
namespace {
volatile std::sig_atomic_t stopped = 0;
void stopSignal(int) { stopped = 1; }
}
int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::invalid_argument("usage: xgc2_display_relays RELAYS_JSON");
    auto specs = xgc2_ros_display_relays::parseRelaySpecs(argv[1]);
    std::signal(SIGINT, stopSignal); std::signal(SIGTERM, stopSignal);
    if (specs.empty()) {
      // Do not initialize ROS or touch the master for an empty layout.
      while (!stopped) std::this_thread::sleep_for(std::chrono::milliseconds(100));
      return 0;
    }
    int ros_argc = 1;
    ros::init(ros_argc, argv, "xgc2_display_relays", ros::init_options::NoSigintHandler);
    ros::NodeHandle node;
    xgc2_ros_display_relays::DisplayRelays relays(std::move(specs));
    relays.start(node);
    while (!stopped && ros::ok()) {
      relays.rethrowFailure();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    relays.rethrowFailure(); relays.stop(); ros::shutdown(); return 0;
  } catch (const std::exception& error) {
    std::cerr << "display relay failed: " << error.what() << '\n';
    if (ros::isInitialized()) ros::shutdown();
    return 1;
  }
}
