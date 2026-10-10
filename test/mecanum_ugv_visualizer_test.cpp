#include "render/robots/mecanum_ugv_visualizer.hpp"

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <visualization_msgs/Marker.h>

namespace xgc2_ros_visualizer {
namespace {

const visualization_msgs::Marker* findMarker(const visualization_msgs::MarkerArray& markers, const std::string& name) {
    for (const visualization_msgs::Marker& marker : markers.markers) {
        if (marker.ns == name) {
            return &marker;
        }
    }
    return nullptr;
}

double quaternionSimilarity(const geometry_msgs::Quaternion& lhs, const geometry_msgs::Quaternion& rhs) {
    return std::abs(lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w);
}

TEST(MecanumUgvVisualizer, UsesNexusMeshesAndMecanumWheelKinematics) {
    MecanumUgvVisualizer visualizer(MecanumUgvVisualizer::Config{});
    MecanumVisualState state;
    state.name = "ugv2";
    state.pose.orientation.w = 1.0;
    state.stamp = ros::Time(1, 0);

    visualization_msgs::MarkerArray initial_markers;
    std::vector<geometry_msgs::TransformStamped> initial_transforms;
    visualizer.append(state, &initial_markers, &initial_transforms);

    const visualization_msgs::Marker* body = findMarker(initial_markers, "ugv2_mecanum_base_link");
    const visualization_msgs::Marker* upper_left = findMarker(initial_markers, "ugv2_mecanum_upper_left_wheel");
    const visualization_msgs::Marker* upper_right = findMarker(initial_markers, "ugv2_mecanum_upper_right_wheel");
    const visualization_msgs::Marker* lower_left = findMarker(initial_markers, "ugv2_mecanum_lower_left_wheel");
    const visualization_msgs::Marker* lower_right = findMarker(initial_markers, "ugv2_mecanum_lower_right_wheel");
    const visualization_msgs::Marker* label = findMarker(initial_markers, "ugv2_label");
    ASSERT_NE(body, nullptr);
    ASSERT_NE(upper_left, nullptr);
    ASSERT_NE(upper_right, nullptr);
    ASSERT_NE(lower_left, nullptr);
    ASSERT_NE(lower_right, nullptr);
    ASSERT_NE(label, nullptr);
    EXPECT_EQ(label->text, "UGV 2");
    EXPECT_EQ(body->mesh_resource,
              "package://mecanum_description/meshes/lod10k/nexus_base_link.dae");
    EXPECT_DOUBLE_EQ(body->scale.x, 1.0);
    EXPECT_NE(upper_left->mesh_resource.find("mecanum_wheel_left.dae"), std::string::npos);
    EXPECT_NE(upper_right->mesh_resource.find("mecanum_wheel_right.dae"), std::string::npos);
    EXPECT_EQ(body->mesh_resource.find("scout_description"), std::string::npos);

    state.stamp = ros::Time(2, 0);
    state.has_motion_hint = true;
    state.lateral_velocity_m_s = 0.5;
    visualization_msgs::MarkerArray moving_markers;
    std::vector<geometry_msgs::TransformStamped> moving_transforms;
    visualizer.append(state, &moving_markers, &moving_transforms);

    const visualization_msgs::Marker* moving_upper_left =
        findMarker(moving_markers, "ugv2_mecanum_upper_left_wheel");
    const visualization_msgs::Marker* moving_lower_left =
        findMarker(moving_markers, "ugv2_mecanum_lower_left_wheel");
    const visualization_msgs::Marker* moving_upper_right =
        findMarker(moving_markers, "ugv2_mecanum_upper_right_wheel");
    const visualization_msgs::Marker* moving_lower_right =
        findMarker(moving_markers, "ugv2_mecanum_lower_right_wheel");
    ASSERT_NE(moving_upper_left, nullptr);
    ASSERT_NE(moving_lower_left, nullptr);
    ASSERT_NE(moving_upper_right, nullptr);
    ASSERT_NE(moving_lower_right, nullptr);
    EXPECT_GT(std::abs(moving_upper_left->pose.orientation.y), 0.1);
    EXPECT_GT(std::abs(moving_lower_left->pose.orientation.y), 0.1);
    EXPECT_LT(moving_upper_left->pose.orientation.y * moving_lower_left->pose.orientation.y, 0.0);
    EXPECT_LT(quaternionSimilarity(upper_left->pose.orientation, moving_upper_left->pose.orientation), 0.99);
    EXPECT_LT(quaternionSimilarity(upper_right->pose.orientation, moving_upper_right->pose.orientation), 0.99);
    EXPECT_LT(quaternionSimilarity(lower_left->pose.orientation, moving_lower_left->pose.orientation), 0.99);
    EXPECT_LT(quaternionSimilarity(lower_right->pose.orientation, moving_lower_right->pose.orientation), 0.99);
}

} // namespace
} // namespace xgc2_ros_visualizer

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
