#include <xgc2_ros_visualizer/config.hpp>
#include <xgc2_ros_visualizer/declared_wire.hpp>
#include <xgc2_ros_visualizer/source_history.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <gtest/gtest.h>
#include <stdexcept>
using namespace xgc2_ros_visualizer;
TEST(Rates, CompleteRoundTripAndKindIsolation) {
  auto original=defaultRates();auto json=original.json();json["scout"]["path"]=2.5;
  auto changed=parseRates(json,true);
  EXPECT_DOUBLE_EQ(changed.get(RateKind::Scout,Channel::Path),2.5);
  EXPECT_DOUBLE_EQ(changed.get(RateKind::Fs150,Channel::Path),10);
  EXPECT_DOUBLE_EQ(changed.get(RateKind::Mecanum,Channel::Path),10);
  EXPECT_DOUBLE_EQ(changed.get(RateKind::Global,Channel::JointTf),30);
  EXPECT_FALSE(applicable(RateKind::Global,Channel::PoseTf));
  EXPECT_EQ(changed.json(),json);
}
TEST(Rates, StartupOverlayAndStrictAtomicCandidateValidation) {
  auto empty=Json::Value(Json::objectValue);
  EXPECT_EQ(parseRates(empty,false).json(),defaultRates().json());
  EXPECT_THROW(parseRates(empty,true),std::invalid_argument);
  for(const auto& text:{"{\"scout\":{\"height_projection\":10}}","{\"global\":{\"pose_tf\":10}}","{\"fs150\":{\"path\":true}}","{\"fs150\":{\"path\":0}}","{\"fs150\":{\"path\":1001}}","{\"fs150\":{\"unknown\":10}}"})
    EXPECT_THROW(parseRates(parseJson(text),false),std::invalid_argument);
  auto original=defaultRates();auto bad=original.json();bad["scout"]["path"]=-1;
  EXPECT_THROW(parseRates(bad,true),std::invalid_argument);
  EXPECT_EQ(original.json(),defaultRates().json());
  EXPECT_THROW(parseJson("{\"x\":1,\"x\":2}"),std::invalid_argument);
}
TEST(RateOverrides, FollowTheKindTableUntilARobotOverridesAChannel) {
  const auto overrides=parseRateOverrides(RateKind::Fs150,parseJson("{\"path\":5,\"markers\":20}"));
  EXPECT_FALSE(overrides.empty());
  EXPECT_DOUBLE_EQ(overrides.get(Channel::Path,10),5);
  EXPECT_DOUBLE_EQ(overrides.get(Channel::Markers,30),20);
  EXPECT_DOUBLE_EQ(overrides.get(Channel::Scene,10),10);
  EXPECT_DOUBLE_EQ(overrides.maximum(),20);
  EXPECT_EQ(overrides.json(),parseJson("{\"path\":5.0,\"markers\":20.0}"));
  EXPECT_TRUE(RateOverrides().empty());
  EXPECT_NE(overrides,RateOverrides());
  EXPECT_EQ(parseRateOverrides(RateKind::Scout,Json::Value(Json::objectValue)),RateOverrides());
}
TEST(RateOverrides, OnlyTheChannelsOfTheRobotKindWithinTheRateRange) {
  // Height projection belongs to FS150 only, the root TF to the scene, relays to the instance.
  EXPECT_THROW(parseRateOverrides(RateKind::Scout,parseJson("{\"height_projection\":10}")),std::invalid_argument);
  EXPECT_THROW(parseRateOverrides(RateKind::Fs150,parseJson("{\"tf_root\":10}")),std::invalid_argument);
  EXPECT_THROW(parseRateOverrides(RateKind::Fs150,parseJson("{\"display_path\":10}")),std::invalid_argument);
  EXPECT_THROW(parseRateOverrides(RateKind::Global,parseJson("{\"pose_tf\":10}")),std::invalid_argument);
  EXPECT_NO_THROW(parseRateOverrides(RateKind::Global,parseJson("{\"joint_tf\":10}")));
  for(const auto& text:{"{\"path\":0}","{\"path\":1001}","{\"path\":\"fast\"}","{\"path\":true}","{\"unknown\":5}"})
    EXPECT_THROW(parseRateOverrides(RateKind::Fs150,parseJson(text)),std::invalid_argument) << text;
  EXPECT_THROW(parseRateOverrides(RateKind::Fs150,parseJson("[1]")),std::invalid_argument);
}
TEST(RelayConfig, KindRowOwnsTheRateAndOnlyFourFullStateTypesAreAccepted) {
  auto spec=relayConfig("/ugv8/path","/xgc/display/ugv8/path","nav_msgs/Path",RateKind::Scout);
  EXPECT_EQ(spec.kind,RateKind::Scout);EXPECT_EQ(spec.channel,Channel::DisplayPath);
  EXPECT_EQ(relayConfig("/map","/xgc/display/map","nav_msgs/OccupancyGrid",RateKind::Global).channel,Channel::DisplayGrid);
  EXPECT_EQ(relayConfig("/c","/xgc/display/c","sensor_msgs/PointCloud2",RateKind::Global).channel,Channel::DisplayPointcloud);
  EXPECT_EQ(relayConfig("/p","/xgc/display/p","geometry_msgs/PoseArray",RateKind::Global).channel,Channel::DisplayPoseArray);
  EXPECT_THROW(relayConfig("/ugv8/path","/xgc/display/ugv8/path","visualization_msgs/MarkerArray",RateKind::Scout),std::invalid_argument);
  EXPECT_THROW(relayConfig("/ugv8/path","/elsewhere/ugv8/path","nav_msgs/Path",RateKind::Scout),std::invalid_argument);
  EXPECT_THROW(relayConfig("/xgc/display/ugv8","/xgc/display/xgc/display/ugv8","nav_msgs/Path",RateKind::Scout),std::invalid_argument);
  EXPECT_THROW(relayConfig("ugv8/path","/xgc/displayugv8/path","nav_msgs/Path",RateKind::Scout),std::invalid_argument);
}
template<class T> void exactWire() {
  DeclaredWire<T> input;input.bytes={0xff,0x00,0x7f,0x80,0x01,0x00,0x00,0x00,0xfe,0xcd};
  std::vector<std::uint8_t> output(input.bytes.size());ros::serialization::OStream writer(output.data(),output.size());ros::serialization::serialize(writer,input);
  EXPECT_EQ(input.bytes,output);DeclaredWire<T> received;ros::serialization::IStream reader(output.data(),output.size());ros::serialization::deserialize(reader,received);EXPECT_EQ(input.bytes,received.bytes);
  EXPECT_STREQ(ros::message_traits::DataType<T>::value(),ros::message_traits::DataType<DeclaredWire<T>>::value());
  EXPECT_STREQ(ros::message_traits::MD5Sum<T>::value(),ros::message_traits::MD5Sum<DeclaredWire<T>>::value());
  EXPECT_FALSE(ros::message_traits::HasHeader<DeclaredWire<T>>::value);
}
TEST(RelayWire, AllFourTypesPreserveSerializedPayloadAndHeaderSequence) {
  exactWire<sensor_msgs::PointCloud2>();exactWire<nav_msgs::OccupancyGrid>();exactWire<nav_msgs::Path>();exactWire<geometry_msgs::PoseArray>();
}
TEST(SourceHistory, SamplingCapacityExpiryAndClockRollback) {
  SourceHistory history;
  geometry_msgs::Pose pose;
  pose.orientation.w=1;
  for(int i=0;i<70;++i) {
    pose.position.x=i;
    ASSERT_TRUE(history.append(ros::Time(100+i/10,(i%10)*100000000),pose));
    EXPECT_LE(history.size,61U);
  }
  ASSERT_EQ(history.size,61U);
  EXPECT_EQ(history.at(0).pose.position.x,9);
  EXPECT_EQ(history.at(60).pose.position.x,69);
  EXPECT_FALSE(history.append(ros::Time(),pose));
  EXPECT_FALSE(history.append(ros::Time(106,950000000),pose));
  EXPECT_FALSE(history.append(ros::Time(106,800000000),pose));
  ASSERT_TRUE(history.append(ros::Time(30),pose));
  ASSERT_EQ(history.size,1U);
  EXPECT_EQ(history.stamp,ros::Time(30));
  ASSERT_TRUE(history.expire(ros::Time(37)));
  EXPECT_EQ(history.size,0U);
  EXPECT_EQ(history.stamp,ros::Time(37));
}
int main(int argc,char** argv) {testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
