#include <xgc2_ros_display_relays/display_relays.hpp>
#include <xgc2_ros_display_relays/declared_wire.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <stdexcept>
#include <cmath>
using namespace xgc2_ros_display_relays;
template<class Predicate> bool until(Predicate predicate, double seconds = 3) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  do { if (predicate()) return true; std::this_thread::sleep_for(std::chrono::milliseconds(10)); }
  while (std::chrono::steady_clock::now() < deadline);
  return predicate();
}
template<class T> void lifecycle(const std::string& name, const std::string& type) {
  ros::NodeHandle node;
  using Wire = DeclaredWire<T>;
  // A real generated message publisher exercises ROS decoding at the relay.
  // Disable only this test publisher's seq rewrite so all bytes are known.
  ros::AdvertiseOptions source_options;
  source_options.template init<T>("/test_" + name, 1);
  source_options.has_header = false;
  auto source = node.advertise(source_options);
  DisplayRelays owner({{"/test_" + name, "/xgc/display/test_" + name, type, 5}});
  owner.start(node);
  // A declaration alone cannot activate even a subscriber-gated source.
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  EXPECT_EQ(source.getNumSubscribers(), 0u);
  std::atomic<unsigned> copies{0}, originals{0}, typed_copies{0};
  std::mutex mutex; std::vector<std::uint8_t> last;
  boost::function<void(const typename Wire::ConstPtr&)> receive = [&](const typename Wire::ConstPtr& message) {
    { std::lock_guard<std::mutex> guard(mutex); last = message->bytes; }
    ++copies;
  };
  auto display = node.subscribe<Wire>("/xgc/display/test_" + name, 1, receive);
  ASSERT_TRUE(until([&] { return source.getNumSubscribers() == 1; }));
  boost::function<void(const boost::shared_ptr<const T>&)> decoded = [&](const boost::shared_ptr<const T>& message) {
    EXPECT_EQ(message->header.seq, 0xdeadbeefu);
    EXPECT_EQ(message->header.stamp, ros::Time(123, 456));
    EXPECT_EQ(message->header.frame_id, "source_frame");
    ++typed_copies;
  };
  auto typed_display = node.subscribe<T>("/xgc/display/test_" + name, 1, decoded);
  boost::function<void(const typename Wire::ConstPtr&)> original = [&](const typename Wire::ConstPtr&) { ++originals; };
  auto recorder = node.subscribe<Wire>("/test_" + name, 1, original);
  // roscpp shares one node connection for callbacks on the same source.
  ASSERT_TRUE(until([&] { return source.getNumSubscribers() == 1; }));
  auto message = boost::shared_ptr<T>(new T);
  message->header.seq = 0xdeadbeefu;
  message->header.stamp = ros::Time(123, 456);
  message->header.frame_id = "source_frame";
  std::vector<std::uint8_t> expected(ros::serialization::serializationLength(*message));
  ros::serialization::OStream serialized(expected.data(), expected.size());
  ros::serialization::serialize(serialized, *message);
  const auto budget_begin = std::chrono::steady_clock::now();
  for (int i = 0; i < 80; ++i) { source.publish(message); std::this_thread::sleep_for(std::chrono::milliseconds(10)); }
  ASSERT_TRUE(until([&] { return copies.load() > 0; }));
  const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - budget_begin).count();
  EXPECT_LE(copies.load(), static_cast<unsigned>(std::ceil(elapsed * 5)) + 1);
  EXPECT_GT(originals.load(), copies.load()); // Original remains outside the display budget.
  EXPECT_GT(typed_copies.load(), 0u);
  { std::lock_guard<std::mutex> guard(mutex); EXPECT_EQ(last, expected); }
  display.shutdown();
  typed_display.shutdown();
  recorder.shutdown();
  ASSERT_TRUE(until([&] { return source.getNumSubscribers() == 0; }));
  // Stop while an upstream publisher is active: release only our subscription.
  display = node.subscribe<Wire>("/xgc/display/test_" + name, 1, receive);
  ASSERT_TRUE(until([&] { return source.getNumSubscribers() == 1; }));
  recorder = node.subscribe<Wire>("/test_" + name, 1, original);
  const auto originals_before_stop = originals.load();
  std::atomic<bool> sending{true};
  std::thread publishing([&] { while (sending.load()) { source.publish(message); std::this_thread::sleep_for(std::chrono::milliseconds(2)); } });
  const bool receiving_during_stop = until([&] { return originals.load() > originals_before_stop; });
  owner.stop(); owner.stop();
  sending.store(false); publishing.join();
  EXPECT_TRUE(receiving_during_stop);
  ASSERT_TRUE(until([&] { return source.getNumSubscribers() == 1; }));
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  const auto settled = copies.load();
  const auto originals_after_stop = originals.load();
  source.publish(message); std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(copies.load(), settled);
  EXPECT_GT(originals.load(), originals_after_stop); owner.rethrowFailure();
}
TEST(DisplayLifecycle, LazyReconnectStopAndCopiesOnlyBudgetForAllTypes) {
  lifecycle<sensor_msgs::PointCloud2>("points", "sensor_msgs/PointCloud2");
  lifecycle<nav_msgs::OccupancyGrid>("map", "nav_msgs/OccupancyGrid");
  lifecycle<nav_msgs::Path>("path", "nav_msgs/Path");
  lifecycle<geometry_msgs::PoseArray>("poses", "geometry_msgs/PoseArray");
}
TEST(DisplayLifecycle, StopBeforeStartCannotCreateNewROSOwnership) {
  ros::NodeHandle node;
  DisplayRelays owner({{"/late_source", "/xgc/display/late_source", "nav_msgs/Path", 5}});
  owner.stop();
  EXPECT_THROW(owner.start(node), std::logic_error);
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "display_lifecycle_test"); testing::InitGoogleTest(&argc, argv);
  ros::AsyncSpinner spinner(2); spinner.start(); const int result = RUN_ALL_TESTS();
  spinner.stop(); ros::shutdown(); return result;
}
