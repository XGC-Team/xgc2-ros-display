#include <xgc2_ros_display_relays/display_relays.hpp>
#include <xgc2_ros_display_relays/declared_wire.hpp>
#include <xgc2_ros_display_relays/rate_gate.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <gtest/gtest.h>
#include <stdexcept>
using namespace xgc2_ros_display_relays;
TEST(DisplaySpec, RejectsNonCanonicalAndNonNumericBudgets) {
  EXPECT_TRUE(parseRelaySpecs("[]").empty());
  for (const auto& json : {
      R"([{"source":"/points","topic":"/points","messageType":"sensor_msgs/PointCloud2","maxRateHz":10}])",
      R"([{"source":"/xgc/display/points","topic":"/xgc/display/xgc/display/points","messageType":"sensor_msgs/PointCloud2","maxRateHz":10}])",
      R"([{"source":"/points","topic":"/xgc/display/points","messageType":"sensor_msgs/PointCloud2","maxRateHz":true}])",
      R"([{"source":"/points","topic":"/xgc/display/points","messageType":"sensor_msgs/PointCloud2","maxRateHz":"10"}])",
      R"([{"source":"/points","topic":"/xgc/display/points","messageType":"sensor_msgs/LaserScan","maxRateHz":10}])"})
    EXPECT_THROW(parseRelaySpecs(json), std::invalid_argument);
  EXPECT_THROW(validateRelaySpecs({{"/points", "/xgc/display/points", "sensor_msgs/PointCloud2", 1},
                                  {"/points", "/xgc/display/points", "sensor_msgs/PointCloud2", 2}}), std::invalid_argument);
}
TEST(DisplayRate, NoBurstOrCatchupAndFirstSampleAfterReconnectIsImmediate) {
  detail::RateGate gate(10);
  const detail::RateGate::Clock::time_point zero{};
  EXPECT_TRUE(gate.admit(zero));
  EXPECT_FALSE(gate.admit(zero + std::chrono::milliseconds(99)));
  EXPECT_TRUE(gate.admit(zero + std::chrono::milliseconds(100)));
  EXPECT_TRUE(gate.admit(zero + std::chrono::seconds(10)));
  EXPECT_FALSE(gate.admit(zero + std::chrono::seconds(10)));
  gate.reset(); EXPECT_TRUE(gate.admit(zero + std::chrono::seconds(10)));
}
template<class T> void preservesWire() {
  DeclaredWire<T> input;
  // Includes arbitrary seq/stamp bytes, NaN payload bits and signed occupancy.
  input.bytes = {0xff, 0x00, 0x7f, 0x80, 0x01, 0x00, 0x00, 0x00, 0xfe, 0xcd};
  std::vector<std::uint8_t> output(input.bytes.size());
  ros::serialization::OStream writer(output.data(), output.size());
  ros::serialization::serialize(writer, input);
  EXPECT_EQ(input.bytes, output);
  DeclaredWire<T> received;
  ros::serialization::IStream reader(output.data(), output.size());
  ros::serialization::deserialize(reader, received);
  EXPECT_EQ(input.bytes, received.bytes);
  EXPECT_STREQ(ros::message_traits::DataType<T>::value(), ros::message_traits::DataType<DeclaredWire<T>>::value());
  EXPECT_STREQ(ros::message_traits::MD5Sum<T>::value(), ros::message_traits::MD5Sum<DeclaredWire<T>>::value());
  EXPECT_FALSE(ros::message_traits::HasHeader<DeclaredWire<T>>::value);
}
TEST(DisplayWire, AllFourDeclaredTypesPreserveSerializedBytes) {
  preservesWire<sensor_msgs::PointCloud2>(); preservesWire<nav_msgs::OccupancyGrid>();
  preservesWire<nav_msgs::Path>(); preservesWire<geometry_msgs::PoseArray>();
}
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
