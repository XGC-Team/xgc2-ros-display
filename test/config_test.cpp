#include <xgc2_ros_visualizer/config.hpp>
#include <xgc2_ros_visualizer/declared_wire.hpp>
#include <xgc2_ros_visualizer/instance_input.hpp>
#include <xgc2_ros_visualizer/source_history.hpp>
#include <xgc2_robot_visualization/path_runtime.hpp>
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
TEST(RelayConfig, KindClassRatesReplaceIndividualBudgetsAndRejectIncrementalTypes) {
  auto value=parseJson("[{\"source\":\"/ugv8/path\",\"topic\":\"/xgc/display/ugv8/path\",\"messageType\":\"nav_msgs/Path\",\"robotKind\":\"scout\"}]");
  auto specs=parseRelays(value);ASSERT_EQ(specs.size(),1U);EXPECT_EQ(specs[0].kind,RateKind::Scout);EXPECT_EQ(specs[0].channel,Channel::DisplayPath);
  value[0]["maxRateHz"]=3;EXPECT_THROW(parseRelays(value),std::invalid_argument);value[0].removeMember("maxRateHz");
  value[0]["messageType"]="visualization_msgs/MarkerArray";EXPECT_THROW(parseRelays(value),std::invalid_argument);
  value[0]["messageType"]="nav_msgs/Path";value.append(value[0]);EXPECT_THROW(parseRelays(value),std::invalid_argument);
}
TEST(InstanceConfig, EmptyMembershipIsValidButPartialAndUnknownSettingsFail) {
  auto value=parseJson("{\"robots\":[],\"descriptions\":[],\"worldBoundary\":null,\"settings\":{},\"displayRelays\":[]}");
  auto config=parseInstance(value);EXPECT_TRUE(config.robots.empty());EXPECT_TRUE(config.descriptions.empty());
  value["settings"]["path_publish_rate"]=4;EXPECT_THROW(parseInstance(value),std::invalid_argument);value["settings"].removeMember("path_publish_rate");
  value.removeMember("descriptions");EXPECT_THROW(parseInstance(value),std::invalid_argument);
}
TEST(InstanceConfig, TrackGroundFlagRetainsFalseAndRequiresBoolean) {
  auto value=parseJson("{\"robots\":[],\"descriptions\":[],\"worldBoundary\":null,\"settings\":{},\"displayRelays\":[]}");
  EXPECT_TRUE(parseInstance(value).settings.track_ugv);
  value["settings"]["track_ugv"]=false;EXPECT_FALSE(parseInstance(value).settings.track_ugv);
  value["settings"]["track_ugv"]="false";EXPECT_THROW(parseInstance(value),std::invalid_argument);
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
Json::Value fullRobot(const std::string& name,const std::string& kind="px4_multirotor",const std::string& scene="fs150") {
  Json::Value result;result["namespace"]="/"+name;result["name"]="human label";result["kind"]=kind;
  auto& visual=result["visualization"];visual["sceneClass"]=scene;visual["descriptionPackage"]="fs150_description";
  visual["descriptionFile"]="urdf/fs150_visual.urdf";visual["robotStatePublisher"]=false;
  visual["jointStateTopic"]="joint_states";visual["pathTopic"]="path";
  result["hybridSource"]="simulation";result["profileId"]="px4-multirotor.physical.vrpn";
  result["px4"]["mocapRigidBodyName"]="measured_"+name;
  result["simulationPoseTopic"]="/frozen/simulation/"+name;
  return result;
}
Json::Value envelope() {
  return parseJson("{\"robots\":[],\"context\":{\"runMode\":\"simulation\",\"worldBoundary\":null,\"scene\":{\"simulator\":\"xsim\"},\"localizationOffset\":{\"x\":1,\"y\":2,\"z\":3}},\"settings\":{},\"displayRelays\":[]}");
}
TEST(InstanceInput, FrozenModeSourceOffsetAndNumericPaletteParity) {
  auto value=envelope();value["robots"].append(fullRobot("uav10"));value["robots"].append(fullRobot("uav2"));
  value["settings"]["uavPalette"]=parseJson("[\"#ABCDEF\",\"#123456\"]");
  for(const auto& mode:{"simulation","physical","hybrid"}) {
    value["context"]["runMode"]=mode;const auto projected=projectInstanceInput(value,true);
    const auto& rows=projected["robots"];
    ASSERT_EQ(rows.size(),2U);EXPECT_EQ(rows[0]["name"],"uav10");EXPECT_EQ(rows[0]["heightProjectionColor"],"#123456");
    EXPECT_EQ(rows[1]["heightProjectionColor"],"#abcdef");
    EXPECT_EQ(rows[1]["arPoseTopic"],std::string(mode)=="physical"?"/vrpn_client_node/measured_uav2/pose":"/frozen/simulation/uav2");
    EXPECT_DOUBLE_EQ(rows[1]["worldOffset"][2].asDouble(),std::string(mode)=="physical"?3:0);
    EXPECT_EQ(projected["descriptions"][1]["worldOffset"][2],0.0);
  }
  value["robots"][0].removeMember("hybridSource");EXPECT_THROW(projectInstanceInput(value,true),std::invalid_argument);
  value["robots"][0]["hybridSource"]="physical";
  auto physical=projectInstanceInput(value,true);EXPECT_EQ(physical["robots"][0]["arPoseTopic"],"/vrpn_client_node_physical/measured_uav10/pose");
  EXPECT_DOUBLE_EQ(physical["robots"][0]["worldOffset"][2].asDouble(),3);
  value["context"]["runMode"]="simulation";value["context"]["scene"]["simulator"]="gazebo";
  auto gazebo=projectInstanceInput(value,true);EXPECT_EQ(gazebo["robots"][0]["arPoseTopic"],"/vrpn_client_node/uav10/pose");
  EXPECT_DOUBLE_EQ(gazebo["robots"][0]["worldOffset"][2].asDouble(),0);
  value["context"]["runMode"]="physical";value["context"]["localizationOffset"]["z"]="invalid";
  EXPECT_THROW(projectInstanceInput(value,true),std::invalid_argument);
}
TEST(InstanceInput, HeightOffScoutBindingAndDescriptionOnlyArePreserved) {
  auto value=envelope();value["settings"]["uavHeightProjection"]=false;value["robots"].append(fullRobot("uav1"));
  auto a=fullRobot("ugv2","scout_mini","scout");a["scout"]["mocapRigidBodyName"]="different";value["robots"].append(a);
  auto b=fullRobot("uav3","scout_mini","scout");b["scout"]["mocapRigidBodyName"]="rigid";value["robots"].append(b);
  value["robots"].append(fullRobot("description_only","other",""));
  auto projected=projectInstanceInput(value,true);EXPECT_EQ(projected["descriptions"].size(),4U);EXPECT_EQ(projected["robots"].size(),3U);
  EXPECT_EQ(projected["robots"][1]["sceneModel"],"rigid");EXPECT_FALSE(projected["robots"][0].isMember("heightProjectionColor"));
  EXPECT_EQ(projected["robots"][2]["sceneModel"],"ugv2");
  // Existing canonical rosters may also include the non-scene description row.
  projected["robots"].append(projected["descriptions"][0]);EXPECT_NO_THROW(parseInstance(projected));
}
TEST(InstanceInput, RelayClassProjectionRejectsLegacyIndividualBudget) {
  auto value=envelope();value["robots"].append(fullRobot("uav1"));
  value["displayRelays"]=parseJson("[{\"source\":\"/uav1/path\",\"topic\":\"/xgc/display/uav1/path\",\"messageType\":\"nav_msgs/Path\"},{\"source\":\"/map\",\"topic\":\"/xgc/display/map\",\"messageType\":\"nav_msgs/OccupancyGrid\"}]");
  const auto result=projectInstanceInput(value,true);EXPECT_EQ(result["displayRelays"][0]["robotKind"],"fs150");
  EXPECT_EQ(result["displayRelays"][1]["robotKind"],"global");EXPECT_FALSE(result["displayRelays"][0].isMember("maxRateHz"));
  value["displayRelays"][0]["maxRateHz"]=3;EXPECT_THROW(projectInstanceInput(value,true),std::invalid_argument);value["displayRelays"][0].removeMember("maxRateHz");
  value["displayRelays"][0]["robotKind"]="scout";EXPECT_THROW(projectInstanceInput(value,true),std::invalid_argument);
  EXPECT_NO_THROW(projectInstanceInput(envelope(),true));EXPECT_FALSE(projectInstanceInput(envelope(),false)["settings"]["use_sim_time"].asBool());
}
TEST(InstanceInput, RejectsRetiredEnvelopeAndValidatesPublicationControls) {
  auto value=envelope();value["instanceId"]="old";
  EXPECT_THROW(projectInstanceInput(value,true),std::invalid_argument);value.removeMember("instanceId");
  value["descriptions"]=Json::Value(Json::arrayValue);
  EXPECT_THROW(projectInstanceInput(value,true),std::invalid_argument);value.removeMember("descriptions");
  value["settings"]["publication"]["scene"]=false;
  value["settings"]["publication"]["transforms"]=false;
  value["settings"]["publication"]["groundScene"]=false;
  auto projected=projectInstanceInput(value,true);
  EXPECT_FALSE(projected["settings"]["publish_scene_update"].asBool());
  EXPECT_FALSE(projected["settings"]["publish_transforms"].asBool());
  EXPECT_FALSE(projected["settings"]["track_ugv"].asBool());
  value["settings"]["publication"]["scene"]="false";
  EXPECT_THROW(projectInstanceInput(value,true),std::invalid_argument);
  value["settings"]["publication"]["scene"]=false;
  value["settings"]["publication"]["alias"]=false;
  EXPECT_THROW(projectInstanceInput(value,true),std::invalid_argument);
}
TEST(InstanceConfig, InvalidSceneLabelNamespaceFailsBeforeActivation) {
  auto value=envelope();value["robots"].append(fullRobot("uav1"));auto request=projectInstanceInput(value,true);
  request["robots"][0]["namespace"]="/bad_slot";EXPECT_THROW(parseInstance(request),std::invalid_argument);
  value["robots"][0]["namespace"]="/bad_slot";EXPECT_THROW(projectInstanceInput(value,true),std::invalid_argument);
}
TEST(SourceHistory, ExactOwningSdkSamplingRollbackAndExpirySemantics) {
  SourceHistory ring;xgc2_robot_visualization::BoundedPathRuntime sdk("world",{});geometry_msgs::Pose pose;pose.orientation.w=1;
  const auto compare=[&] {const auto& path=sdk.message();ASSERT_EQ(ring.size,path.poses.size());EXPECT_EQ(ring.stamp,path.header.stamp);for(std::size_t i=0;i<ring.size;++i)EXPECT_EQ(ring.at(i).stamp,path.poses[i].header.stamp);};
  for(int i=0;i<900;++i) {auto time=ros::Time(100)+ros::Duration(i*.02);pose.position.x=i;EXPECT_EQ(ring.append(time,pose),sdk.append(time,pose));compare();EXPECT_LE(ring.size,61U);}
  for(const auto& time:{ros::Time(118),ros::Time(117),ros::Time(103),ros::Time(120),ros::Time(127)}) {EXPECT_EQ(ring.expire(time),sdk.expire(time));compare();}
  EXPECT_EQ(ring.append(ros::Time(),pose),sdk.append(ros::Time(),pose));compare();
  EXPECT_EQ(ring.append(ros::Time(30),pose),sdk.append(ros::Time(30),pose));compare();
}
int main(int argc,char** argv) {testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
