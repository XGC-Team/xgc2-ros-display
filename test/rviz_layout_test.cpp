#include "rviz_layout.hpp"
#include <fstream>
#include <gtest/gtest.h>
#include <iterator>
#include <stdexcept>

namespace {
Json::Value input() {
  std::ifstream stream(std::string(RVIZ_TESTDATA) + "/rviz_layout_input.json");
  Json::Value value;
  stream >> value;
  return value;
}
} // namespace

TEST(RVizLayout, PreservesFrozenMixedRobotLayout) {
  const auto result = xgc2_ros_visualizer::prepareRvizLayout(input());
  std::ifstream expected(std::string(RVIZ_TESTDATA) +
                         "/rviz_layout_mixed.golden.rviz");
  ASSERT_TRUE(expected.good());
  EXPECT_EQ(result["config"].asString(),
            std::string(std::istreambuf_iterator<char>(expected), {}));
  EXPECT_EQ(result["fixedFrame"].asString(), "world");
  EXPECT_EQ(result["tfTopic"].asString(), "/xgc/tf");
  EXPECT_EQ(result["worldClock"].asString(), "simulation");
}

TEST(RVizLayout, RejectsDuplicateMembershipAndMixedTransformTrees) {
  auto value = input();
  value["robots"][1]["namespace"] = value["robots"][0]["namespace"];
  EXPECT_THROW(xgc2_ros_visualizer::prepareRvizLayout(value),
               std::invalid_argument);
  value = input();
  value["robots"][1]["visualization"]["sceneClass"] = "";
  EXPECT_THROW(xgc2_ros_visualizer::prepareRvizLayout(value),
               std::invalid_argument);
}

TEST(RVizLayout, ClockBelongsToNativeRunMode) {
  auto value = input();
  value["context"]["runMode"] = "hybrid";
  EXPECT_EQ(xgc2_ros_visualizer::prepareRvizLayout(value)["worldClock"].asString(), "wall");
  value["context"]["runMode"] = "physical";
  EXPECT_EQ(xgc2_ros_visualizer::prepareRvizLayout(value)["worldClock"].asString(), "wall");
}
