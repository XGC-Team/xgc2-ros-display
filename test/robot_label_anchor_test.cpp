// A robot's name label is the one thing in the scene that is anchored rather
// than positioned: the viewer redraws it wherever its frame currently is. That
// only stays correct if the anchor frame refuses to inherit the vehicle's
// attitude. If it ever did, a banking aircraft would carry its own name off to
// one side and tilt the text with it -- which is exactly what publishing the
// label in the body frame would have done.

#include "render/robots/fs150_uav_visualizer.hpp"
#include "render/robots/mecanum_ugv_visualizer.hpp"
#include "render/robots/path_history.hpp"
#include "render/robots/robot_frames.hpp"
#include "render/robots/scout_ugv_visualizer.hpp"

#include <cmath>
#include <algorithm>
#include <string>
#include <set>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <gtest/gtest.h>
#include <ros/serialization.h>
#include <iostream>
#include <visualization_msgs/MarkerArray.h>

namespace xgc2_ros_visualizer {
namespace {

const geometry_msgs::TransformStamped* findTransform(
    const std::vector<geometry_msgs::TransformStamped>& transforms, const std::string& child) {
    for (const geometry_msgs::TransformStamped& transform : transforms) {
        if (transform.child_frame_id == child) {
            return &transform;
        }
    }
    return nullptr;
}

const visualization_msgs::Marker* findLabel(const visualization_msgs::MarkerArray& markers) {
    for (const visualization_msgs::Marker& marker : markers.markers) {
        if (marker.type == visualization_msgs::Marker::TEXT_VIEW_FACING) {
            return &marker;
        }
    }
    return nullptr;
}

// Roll 30 degrees and yaw 90: an attitude that would move the label visibly if
// the anchor inherited any of it.
geometry_msgs::Quaternion bankedAttitude() {
    const double roll = 30.0 * M_PI / 180.0;
    const double yaw = 90.0 * M_PI / 180.0;
    geometry_msgs::Quaternion attitude;
    attitude.w = std::cos(roll / 2.0) * std::cos(yaw / 2.0);
    attitude.x = std::sin(roll / 2.0) * std::cos(yaw / 2.0);
    attitude.y = std::sin(roll / 2.0) * std::sin(yaw / 2.0);
    attitude.z = std::cos(roll / 2.0) * std::sin(yaw / 2.0);
    return attitude;
}

void expectUprightAnchorAbove(const geometry_msgs::TransformStamped& anchor,
                              const geometry_msgs::Pose& pose, double height) {
    EXPECT_DOUBLE_EQ(anchor.transform.translation.x, pose.position.x);
    EXPECT_DOUBLE_EQ(anchor.transform.translation.y, pose.position.y);
    EXPECT_DOUBLE_EQ(anchor.transform.translation.z, pose.position.z + height);
    // Identity, not the vehicle's attitude. This is the whole guarantee.
    EXPECT_DOUBLE_EQ(anchor.transform.rotation.x, 0.0);
    EXPECT_DOUBLE_EQ(anchor.transform.rotation.y, 0.0);
    EXPECT_DOUBLE_EQ(anchor.transform.rotation.z, 0.0);
    EXPECT_DOUBLE_EQ(anchor.transform.rotation.w, 1.0);
}

TEST(Fs150Camera, OpticalAxesFollowTheNoseAndUseRobotScopedFrames) {
    Fs150UavVisualizer visualizer{Fs150UavVisualizer::Config{}};
    UavVisualState state;
    state.name = "uav2";
    state.pose.orientation = bankedAttitude();
    state.pose.position.x = 8;
    state.stamp = ros::Time(3);
    visualization_msgs::MarkerArray markers;
    std::vector<geometry_msgs::TransformStamped> transforms;
    visualizer.append(state, &markers, &transforms);
    const auto* camera = findTransform(transforms, "xgc/robots/uav2/camera_link");
    const auto* optical = findTransform(transforms, "xgc/robots/uav2/camera_optical_frame");
    ASSERT_NE(camera, nullptr);
    ASSERT_NE(optical, nullptr);
    EXPECT_EQ(camera->header.frame_id, "xgc/robots/uav2/base_link");
    EXPECT_EQ(optical->header.frame_id, camera->child_frame_id);
    EXPECT_DOUBLE_EQ(camera->transform.translation.x, 0.0488);
    EXPECT_DOUBLE_EQ(camera->transform.translation.y, 0);
    EXPECT_DOUBLE_EQ(camera->transform.translation.z, -0.016);
    const auto& q = optical->transform.rotation;
    // R * optical Z = camera X; R * optical X = camera -Y; R * optical Y = camera -Z.
    EXPECT_DOUBLE_EQ(2*(q.x*q.z+q.w*q.y), 1);
    EXPECT_DOUBLE_EQ(2*(q.x*q.y+q.w*q.z), -1);
    EXPECT_DOUBLE_EQ(2*(q.y*q.z+q.w*q.x), -1);
}

TEST(RobotLabelAnchor, MultirotorLabelStaysOverheadWhileBanking) {
    Fs150UavVisualizer visualizer{Fs150UavVisualizer::Config{}};
    UavVisualState state;
    state.name = "uav3";
    state.pose.position.x = 4.0;
    state.pose.position.y = -2.5;
    state.pose.position.z = 12.0;
    state.pose.orientation = bankedAttitude();
    state.stamp = ros::Time(7, 0);

    visualization_msgs::MarkerArray markers;
    std::vector<geometry_msgs::TransformStamped> transforms;
    visualizer.append(state, &markers, &transforms);

    const geometry_msgs::TransformStamped* anchor =
        findTransform(transforms, robotLabelFrame("uav3"));
    ASSERT_NE(anchor, nullptr) << "the label has no anchor to follow";
    EXPECT_EQ(anchor->header.frame_id, "world");
    expectUprightAnchorAbove(*anchor, state.pose, 0.55);

    // The body frame does carry the attitude; the two must not be confused.
    const geometry_msgs::TransformStamped* body =
        findTransform(transforms, robotBodyFrame("uav3"));
    ASSERT_NE(body, nullptr);
    EXPECT_DOUBLE_EQ(body->transform.rotation.w, state.pose.orientation.w);

    const visualization_msgs::Marker* label = findLabel(markers);
    ASSERT_NE(label, nullptr);
    EXPECT_EQ(label->header.frame_id, robotLabelFrame("uav3"));
    // The offset lives in the anchor now, so the marker sits at its origin.
    EXPECT_DOUBLE_EQ(label->pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(label->pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(label->pose.position.z, 0.0);
}

TEST(RobotLabelAnchor, ScoutLabelStaysOverheadOnASlope) {
    ScoutUgvVisualizer visualizer{ScoutUgvVisualizer::Config{}};
    UgvVisualState state;
    state.name = "ugv2";
    state.pose.position.x = -3.0;
    state.pose.position.y = 1.0;
    state.pose.position.z = 0.2;
    state.pose.orientation = bankedAttitude();
    state.stamp = ros::Time(7, 0);

    visualization_msgs::MarkerArray markers;
    std::vector<geometry_msgs::TransformStamped> transforms;
    visualizer.append(state, &markers, &transforms);

    const geometry_msgs::TransformStamped* anchor =
        findTransform(transforms, robotLabelFrame("ugv2"));
    ASSERT_NE(anchor, nullptr);
    geometry_msgs::Pose on_wheels = state.pose;
    on_wheels.position.z = scoutDisplayBodyZ();
    expectUprightAnchorAbove(*anchor, on_wheels, 0.65);
    const geometry_msgs::TransformStamped* body = findTransform(transforms, robotBodyFrame("ugv2"));
    ASSERT_NE(body, nullptr);
    EXPECT_DOUBLE_EQ(body->transform.translation.z, scoutDisplayBodyZ());

    const visualization_msgs::Marker* label = findLabel(markers);
    ASSERT_NE(label, nullptr);
    EXPECT_EQ(label->header.frame_id, robotLabelFrame("ugv2"));
}

TEST(RobotLabelAnchor, MecanumLabelStaysOverheadOnASlope) {
    MecanumUgvVisualizer visualizer{MecanumUgvVisualizer::Config{}};
    MecanumVisualState state;
    state.name = "mecanum1";
    state.pose.position.x = 0.5;
    state.pose.position.y = 6.0;
    state.pose.position.z = 0.05;
    state.pose.orientation = bankedAttitude();
    state.stamp = ros::Time(7, 0);

    visualization_msgs::MarkerArray markers;
    std::vector<geometry_msgs::TransformStamped> transforms;
    visualizer.append(state, &markers, &transforms);

    const geometry_msgs::TransformStamped* anchor =
        findTransform(transforms, robotLabelFrame("mecanum1"));
    ASSERT_NE(anchor, nullptr);
    geometry_msgs::Pose on_ground = state.pose;
    on_ground.position.z = 0.0;
    expectUprightAnchorAbove(*anchor, on_ground, 0.32);
    const geometry_msgs::TransformStamped* body = findTransform(transforms, robotBodyFrame("mecanum1"));
    ASSERT_NE(body, nullptr);
    EXPECT_DOUBLE_EQ(body->transform.translation.z, 0.0);

    const visualization_msgs::Marker* label = findLabel(markers);
    ASSERT_NE(label, nullptr);
    EXPECT_EQ(label->header.frame_id, robotLabelFrame("mecanum1"));
}

// The anchor is world-parented on purpose: the publisher forwards every
// world-parented transform at the pose rate and gates the rest down to the
// joint rate. A label parented to its own robot would animate at 5 Hz.
TEST(RobotLabelAnchor, AnchorIsWorldParentedSoItRidesThePoseRate) {
    Fs150UavVisualizer visualizer{Fs150UavVisualizer::Config{}};
    UavVisualState state;
    state.name = "uav1";
    state.pose.orientation.w = 1.0;
    state.stamp = ros::Time(7, 0);

    visualization_msgs::MarkerArray markers;
    std::vector<geometry_msgs::TransformStamped> transforms;
    visualizer.append(state, &markers, &transforms);

    const geometry_msgs::TransformStamped* anchor =
        findTransform(transforms, robotLabelFrame("uav1"));
    ASSERT_NE(anchor, nullptr);
    EXPECT_EQ(anchor->header.frame_id, "world");
    EXPECT_NE(anchor->header.frame_id, robotBodyFrame("uav1"));
}

template <typename Visualizer, typename State>
void expectIsolatedDisplayTree(const std::string& name) {
    Visualizer visualizer{typename Visualizer::Config{}};
    State state;
    state.name = name;
    state.pose.orientation.w = 1.0;
    state.stamp = ros::Time(7, 0);
    visualization_msgs::MarkerArray markers;
    std::vector<geometry_msgs::TransformStamped> transforms;
    visualizer.append(state, &markers, &transforms);
    const std::string prefix = "xgc/robots/" + name + "/";
    std::set<std::string> children;
    ASSERT_GT(transforms.size(), 2u);
    for (const auto& transform : transforms) {
        EXPECT_EQ(transform.child_frame_id.find(prefix), 0u);
        EXPECT_TRUE(children.insert(transform.child_frame_id).second);
        EXPECT_TRUE(transform.header.frame_id == "world" ||
                    transform.header.frame_id.find(prefix) == 0u);
        EXPECT_NE(transform.child_frame_id.find(name + "/"), 0u);
    }
    EXPECT_EQ(children.count(prefix + "base_link"), 1u);
    for (const auto& marker : markers.markers) {
        EXPECT_TRUE(marker.header.frame_id == "world" ||
                    marker.header.frame_id.find(prefix) == 0u);
    }
}

TEST(RobotDisplayTree, EveryBodyJointAndLabelIsIsolatedFromPlantAndOnboardFrames) {
    expectIsolatedDisplayTree<Fs150UavVisualizer, UavVisualState>("uav1");
    expectIsolatedDisplayTree<ScoutUgvVisualizer, UgvVisualState>("ugv1");
    expectIsolatedDisplayTree<MecanumUgvVisualizer, MecanumVisualState>("mecanum1");
}

template <typename Message>
std::vector<uint8_t> messageBytes(const Message& message) {
    std::vector<uint8_t> bytes(ros::serialization::serializationLength(message));
    ros::serialization::OStream stream(bytes.data(), bytes.size());
    ros::serialization::serialize(stream, message);
    return bytes;
}

template <typename Visualizer, typename State>
void expectSelectiveOutputs(const typename Visualizer::Config& config, State state) {
    Visualizer full(config), tf_only(config), suppressed(config);
    visualization_msgs::MarkerArray full_markers;
    std::vector<geometry_msgs::TransformStamped> full_tf, only_tf;
    const unsigned stamps[] = {1U, 2U, 3U, 1U, 2U};
    for (unsigned i = 0; i < 5; ++i) {
        state.stamp = ros::Time(stamps[i], 0);
        state.pose.position.x = 0.2 * i;
        state.pose.orientation.w = std::cos(0.05 * i);
        state.pose.orientation.z = std::sin(0.05 * i);
        full_markers.markers.clear(); full_tf.clear(); only_tf.clear();
        full.append(state, &full_markers, &full_tf);
        tf_only.append(state, nullptr, &only_tf, false, false, false);
        suppressed.append(state, nullptr, nullptr, false, false, false);
        ASSERT_EQ(full_tf.size(), only_tf.size());
        for (std::size_t j = 0; j < full_tf.size(); ++j)
            EXPECT_EQ(messageBytes(full_tf[j]), messageBytes(only_tf[j]));
    }
    visualization_msgs::MarkerArray selected;
    tf_only.append(state, &selected, nullptr, false, true, true);
    ASSERT_EQ(selected.markers.size(), 2U);
    for (const auto& marker : selected.markers) {
        auto found = std::find_if(full_markers.markers.begin(), full_markers.markers.end(),
            [&](const visualization_msgs::Marker& expected) { return marker.ns == expected.ns; });
        ASSERT_NE(found, full_markers.markers.end());
        EXPECT_EQ(messageBytes(marker), messageBytes(*found));
    }
    visualization_msgs::MarkerArray restored;
    std::vector<geometry_msgs::TransformStamped> restored_tf;
    suppressed.append(state, &restored, &restored_tf);
    EXPECT_EQ(messageBytes(restored), messageBytes(full_markers));
    ASSERT_EQ(restored_tf.size(), full_tf.size());
    for (std::size_t i = 0; i < full_tf.size(); ++i)
        EXPECT_EQ(messageBytes(restored_tf[i]), messageBytes(full_tf[i]));
    std::cout << "selective " << state.name << ": full=" << full_markers.markers.size()
              << " no_marker=0 label_path=" << selected.markers.size()
              << " meshes_skipped=" << full_markers.markers.size() - selected.markers.size()
              << " path_deep_copy=0 TF/history/phase bytes equal\n";
}

TEST(RobotLabelAnchor, NullableSelectiveOutputsPreserveFS150StateAndTF) {
    UavVisualState state; state.name = "uav-select"; state.rotors_active = true;
    expectSelectiveOutputs<Fs150UavVisualizer>(Fs150UavVisualizer::Config{}, state);
}
TEST(RobotLabelAnchor, NullableSelectiveOutputsPreserveScoutStateAndTF) {
    UgvVisualState state; state.name = "scout-select";
    expectSelectiveOutputs<ScoutUgvVisualizer>(ScoutUgvVisualizer::Config{}, state);
}
TEST(RobotLabelAnchor, NullableSelectiveOutputsPreserveMecanumStateAndTF) {
    MecanumVisualState state; state.name = "mecanum-select";
    expectSelectiveOutputs<MecanumUgvVisualizer>(MecanumUgvVisualizer::Config{}, state);
}

} // namespace
} // namespace xgc2_ros_visualizer

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
