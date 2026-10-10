#include "xgc2_ros_visualizer/robot_roster.hpp"

#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace xgc2_ros_visualizer {
namespace {

const char* kB2 = R"json({
  "name":"b21",
  "namespace":"/b21",
  "descriptionPackage":"b2arx_description",
  "descriptionFile":"urdf/b2arx_visual.urdf",
  "robotStatePublisher":true,
  "jointStateTopic":"joint_states",
  "sceneModel":"",
  "odometryTopic":"odom",
  "pathTopic":"path"
})json";

TEST(RobotDescriptionRuntime, AcceptsAndSortsMixedFrozenRoster) {
    const std::string raw = std::string("[") + R"json({
      "name":"uav1",
      "namespace":"/uav1",
      "descriptionPackage":"fs150_description",
      "descriptionFile":"urdf/fs150_visual.urdf",
      "robotStatePublisher":false,
      "jointStateTopic":"joint_states",
      "sceneModel":"uav1",
      "odometryTopic":"",
      "pathTopic":"path"
    },)json" + kB2 + "]";
    std::vector<RobotDescription> robots;
    std::string error;
    ASSERT_TRUE(readRobotVisualizationRoster(raw, &robots, &error)) << error;
    ASSERT_EQ(robots.size(), 2u);
    EXPECT_EQ(robots[0].name, "b21");
    EXPECT_EQ(robots[0].ros_namespace, "/b21");
    EXPECT_EQ(robots[0].description_package, "b2arx_description");
    EXPECT_TRUE(robots[0].robot_state_publisher);
    EXPECT_EQ(robots[1].name, "uav1");
    EXPECT_FALSE(robots[1].robot_state_publisher);
    EXPECT_EQ(robots[1].scene_model, "uav1");
    EXPECT_EQ(robots[1].path_topic, "path");
}

TEST(RobotDescriptionRuntime, ReadsARPoseAndWorldOffsetWithoutHistoryWindow) {
    std::string row(kB2);
    row.insert(row.rfind('}'), R"json(,"arPoseTopic":"/vrpn_client_node/body/pose","arPathTopic":"ar_path","worldOffset":[1,2,3])json");
    std::vector<RobotDescription> robots;
    std::string error;
    ASSERT_TRUE(readRobotVisualizationRoster("[" + row + "]", &robots, &error)) << error;
    ASSERT_EQ(robots.size(), 1u);
    EXPECT_EQ(robots[0].ar_pose_topic, "/vrpn_client_node/body/pose");
    EXPECT_EQ(robots[0].ar_path_topic, "ar_path");
    EXPECT_DOUBLE_EQ(robots[0].world_offset[2], 3);
    row.replace(row.find("[1,2,3]"), 7, "[1,2]");
    EXPECT_FALSE(readRobotVisualizationRoster("[" + row + "]", &robots, &error));
    EXPECT_DOUBLE_EQ(robots[0].world_offset[2], 3); // failed admission preserves the previous roster
}

TEST(RobotDescriptionRuntime, RejectsRetiredHistoryWindowSec) {
    std::string row(kB2);
    row.insert(row.rfind('}'), R"json(,"historyWindowSec":10)json");
    std::vector<RobotDescription> robots = {RobotDescription{"keep", "/keep", "pkg", "a.urdf", false, "joint_states"}};
    std::string error;
    EXPECT_FALSE(readRobotVisualizationRoster("[" + row + "]", &robots, &error));
    EXPECT_NE(error.find("unknown field historyWindowSec"), std::string::npos) << error;
    ASSERT_EQ(robots.size(), 1u);
    EXPECT_EQ(robots[0].name, "keep");
}

TEST(RobotDescriptionRuntime, AcceptsEmptyDescriptionCapabilityRoster) {
    std::vector<RobotDescription> robots = {
        RobotDescription{"stale", "/stale", "pkg", "stale.urdf", false,
                         "joint_states"}};
    std::string error;
    ASSERT_TRUE(readRobotVisualizationRoster("[]", &robots, &error)) << error;
    EXPECT_TRUE(robots.empty());
}

TEST(RobotDescriptionRuntime, ReadsOptionalCanonicalHeightProjectionColor) {
    std::vector<RobotDescription> robots;
    std::string error;
    ASSERT_TRUE(readRobotVisualizationRoster("[" + std::string(kB2) + "]", &robots, &error)) << error;
    EXPECT_TRUE(robots[0].height_projection_color.empty());
    std::string row(kB2);
    row.insert(row.rfind('}'), R"json(,"heightProjectionColor":"#123abc")json");
    ASSERT_TRUE(readRobotVisualizationRoster("[" + row + "]", &robots, &error)) << error;
    EXPECT_EQ(robots[0].height_projection_color, "#123abc");
    for (const auto& invalid : {"#ABCDEF", "#123abcff", "red"}) {
        std::string bad(row);
        bad.replace(bad.find("#123abc"), 7, invalid);
        EXPECT_FALSE(readRobotVisualizationRoster("[" + bad + "]", &robots, &error));
        EXPECT_EQ(robots[0].height_projection_color, "#123abc");
    }
}

TEST(RobotDescriptionRuntime, RejectsNonCanonicalOrIncompleteRoster) {
    struct Case {
        const char* name;
        std::string raw;
        const char* error;
    };
    const std::vector<Case> cases = {
        {"not array", "{}", "JSON array"},
        {"unknown field", std::string("[") +
             std::string(kB2).substr(0, std::string(kB2).size() - 1) +
             R"json(,"kind":"unitree_b2"}])json", "unknown field kind"},
        {"duplicate joint owners", R"json([{"name":"ugv1","namespace":"/ugv1","descriptionPackage":"scout_description","descriptionFile":"urdf/scout_visual.urdf","robotStatePublisher":true,"jointStateTopic":"joint_states","sceneModel":"ugv1","odometryTopic":"odom","pathTopic":"path"}])json", "not canonical"},
        {"namespace mismatch", R"json([{"name":"b21","namespace":"/dog","descriptionPackage":"b2arx_description","descriptionFile":"urdf/b2arx_visual.urdf","robotStatePublisher":true,"jointStateTopic":"joint_states","sceneModel":"","odometryTopic":"odom","pathTopic":"path"}])json", "not canonical"},
        {"absolute joint topic", R"json([{"name":"b21","namespace":"/b21","descriptionPackage":"b2arx_description","descriptionFile":"urdf/b2arx_visual.urdf","robotStatePublisher":true,"jointStateTopic":"/joint_states","sceneModel":"","odometryTopic":"odom","pathTopic":"path"}])json", "not canonical"},
        {"host description path", R"json([{"name":"b21","namespace":"/b21","descriptionPackage":"b2arx_description","descriptionFile":"/home/user/b2.urdf","robotStatePublisher":true,"jointStateTopic":"joint_states","sceneModel":"","odometryTopic":"odom","pathTopic":"path"}])json", "not canonical"},
        {"traversal description path", R"json([{"name":"b21","namespace":"/b21","descriptionPackage":"b2arx_description","descriptionFile":"../b2.urdf","robotStatePublisher":true,"jointStateTopic":"joint_states","sceneModel":"","odometryTopic":"odom","pathTopic":"path"}])json", "not canonical"},
        {"missing field", R"json([{"name":"b21","namespace":"/b21"}])json", "exact contract fields"},
        {"duplicate field", R"json([{"name":"b21","namespace":"/b21","descriptionPackage":"b2arx_description","descriptionFile":"urdf/b2arx_visual.urdf","robotStatePublisher":true,"robotStatePublisher":true,"jointStateTopic":"joint_states","sceneModel":"","odometryTopic":"odom","pathTopic":"path"}])json", "repeats field robotStatePublisher"},
        {"absolute odometry topic", R"json([{"name":"b21","namespace":"/b21","descriptionPackage":"b2arx_description","descriptionFile":"urdf/b2arx_visual.urdf","robotStatePublisher":true,"jointStateTopic":"joint_states","sceneModel":"","odometryTopic":"/odom","pathTopic":"path"}])json", "not canonical"},
        {"absolute path topic", R"json([{"name":"b21","namespace":"/b21","descriptionPackage":"b2arx_description","descriptionFile":"urdf/b2arx_visual.urdf","robotStatePublisher":true,"jointStateTopic":"joint_states","sceneModel":"","odometryTopic":"odom","pathTopic":"/path"}])json", "not canonical"},
        {"scene without path", R"json([{"name":"b21","namespace":"/b21","descriptionPackage":"b2arx_description","descriptionFile":"urdf/b2arx_visual.urdf","robotStatePublisher":true,"jointStateTopic":"joint_states","sceneModel":"b21","odometryTopic":"odom","pathTopic":""}])json", "not canonical"},
        {"duplicate", std::string("[") + kB2 + "," + kB2 + "]", "repeats model b21"},
    };
    for (const auto& test : cases) {
        SCOPED_TRACE(test.name);
        std::vector<RobotDescription> robots = {RobotDescription{"keep", "/keep", "pkg", "a.urdf", false, "joint_states"}};
        std::string error;
        EXPECT_FALSE(readRobotVisualizationRoster(test.raw, &robots, &error));
        EXPECT_NE(error.find(test.error), std::string::npos) << error;
        ASSERT_EQ(robots.size(), 1u);
        EXPECT_EQ(robots[0].name, "keep");
    }
}

TEST(RobotDescriptionRuntime, RejectsRosterAboveProductBound) {
    std::ostringstream raw;
    raw << '[';
    for (int index = 0; index < 257; ++index) {
        if (index != 0) {
            raw << ',';
        }
        raw << "{\"name\":\"r" << index << "\",\"namespace\":\"/r" << index
            << "\",\"descriptionPackage\":\"robot_description\","
               "\"descriptionFile\":\"urdf/robot.urdf\","
               "\"robotStatePublisher\":false,\"jointStateTopic\":\"joint_states\","
               "\"sceneModel\":\"\",\"odometryTopic\":\"odom\",\"pathTopic\":\"path\"}";
    }
    raw << ']';
    std::vector<RobotDescription> robots;
    std::string error;
    EXPECT_FALSE(readRobotVisualizationRoster(raw.str(), &robots, &error));
    EXPECT_NE(error.find("at most 256"), std::string::npos) << error;
}

TEST(RobotDescriptionRuntime, RejectsDuplicateSceneModelMapping) {
    const std::string raw = R"json([
      {"name":"uav7","namespace":"/uav7","descriptionPackage":"scout_description","descriptionFile":"urdf/scout_visual.urdf","robotStatePublisher":false,"jointStateTopic":"joint_states","sceneModel":"ugv1","odometryTopic":"odom","pathTopic":"path"},
      {"name":"uav8","namespace":"/uav8","descriptionPackage":"scout_description","descriptionFile":"urdf/scout_visual.urdf","robotStatePublisher":false,"jointStateTopic":"joint_states","sceneModel":"ugv1","odometryTopic":"odom","pathTopic":"path"}
    ])json";
    std::vector<RobotDescription> robots;
    std::string error;
    EXPECT_FALSE(readRobotVisualizationRoster(raw, &robots, &error));
    EXPECT_NE(error.find("repeats scene model ugv1"), std::string::npos) << error;
}

}  // namespace
}  // namespace xgc2_ros_visualizer

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
