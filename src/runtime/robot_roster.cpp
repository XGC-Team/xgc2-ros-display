#include "xgc2_ros_visualizer/robot_roster.hpp"

#include <cmath>
#include <algorithm>
#include <cctype>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace xgc2_ros_visualizer {
namespace {

bool canonicalROSIdentifier(const std::string& value) {
    static const std::regex pattern("^[A-Za-z_][A-Za-z0-9_]*$");
    return !value.empty() && value.size() <= 127 && std::regex_match(value, pattern);
}

bool canonicalROSPackage(const std::string& value) {
    static const std::regex pattern("^[a-z][a-z0-9_]*$");
    return std::regex_match(value, pattern);
}

bool canonicalRelativeROSName(const std::string& value) {
    if (value.empty() || value.front() == '/' || value.back() == '/') {
        return false;
    }
    std::stringstream stream(value);
    std::string segment;
    while (std::getline(stream, segment, '/')) {
        if (!canonicalROSIdentifier(segment)) {
            return false;
        }
    }
    return true;
}

bool canonicalDescriptionFile(const std::string& value) {
    static const std::regex pattern(
        "^[A-Za-z0-9_.-]+(/[A-Za-z0-9_.-]+)*\\.urdf$");
    if (!std::regex_match(value, pattern) || value.front() == '/') {
        return false;
    }
    std::stringstream stream(value);
    std::string segment;
    while (std::getline(stream, segment, '/')) {
        if (segment == "." || segment == "..") {
            return false;
        }
    }
    return true;
}

bool hasJSONArrayEnvelope(const std::string& raw) {
    const auto begin = std::find_if_not(raw.begin(), raw.end(), [](unsigned char value) {
        return std::isspace(value) != 0;
    });
    const auto end = std::find_if_not(raw.rbegin(), raw.rend(), [](unsigned char value) {
        return std::isspace(value) != 0;
    });
    return begin != raw.end() && end != raw.rend() && *begin == '[' && *end == ']';
}

}  // namespace

bool readRobotVisualizationRoster(const std::string& raw,
                                  std::vector<RobotDescription>* robots,
                                  std::string* error) {
    if (robots == nullptr || error == nullptr) {
        return false;
    }
    if (!hasJSONArrayEnvelope(raw)) {
        *error = "frozen Robot visualization roster must be a JSON array";
        return false;
    }

    boost::property_tree::ptree root;
    try {
        std::stringstream stream(raw);
        boost::property_tree::read_json(stream, root);
    } catch (const std::exception& exception) {
        *error = std::string("cannot decode frozen Robot visualization roster: ") +
                 exception.what();
        return false;
    }

    static const std::set<std::string> allowed_fields = {
        "name", "namespace", "descriptionPackage", "descriptionFile",
        "robotStatePublisher", "jointStateTopic", "sceneModel", "odometryTopic", "pathTopic",
    };
    const std::set<std::string> optional_fields = {"arPoseTopic", "arPathTopic", "worldOffset", "heightProjectionColor"};
    std::set<std::string> names;
    std::set<std::string> scene_models;
    std::vector<RobotDescription> prepared;
    for (const auto& item : root) {
        if (!item.first.empty()) {
            *error = "frozen Robot visualization roster must be a JSON array";
            return false;
        }
        std::set<std::string> entry_fields;
        for (const auto& field : item.second) {
            if (allowed_fields.count(field.first) == 0 && optional_fields.count(field.first) == 0) {
                *error = "frozen Robot visualization roster contains unknown field " + field.first;
                return false;
            }
            if (!entry_fields.insert(field.first).second) {
                *error = "frozen Robot visualization roster repeats field " + field.first;
                return false;
            }
        }
        for (const auto& optional : optional_fields) { entry_fields.erase(optional); }
        if (entry_fields != allowed_fields) {
            *error = "frozen Robot visualization roster entry does not have the exact contract fields";
            return false;
        }
        RobotDescription robot;
        try {
            robot.name = item.second.get<std::string>("name");
            robot.ros_namespace = item.second.get<std::string>("namespace");
            robot.description_package = item.second.get<std::string>("descriptionPackage");
            robot.description_file = item.second.get<std::string>("descriptionFile");
            robot.robot_state_publisher = item.second.get<bool>("robotStatePublisher");
            robot.joint_state_topic = item.second.get<std::string>("jointStateTopic");
            robot.scene_model = item.second.get<std::string>("sceneModel");
            robot.odometry_topic = item.second.get<std::string>("odometryTopic");
            robot.path_topic = item.second.get<std::string>("pathTopic");
            robot.height_projection_color = item.second.get<std::string>("heightProjectionColor", "");
            if (!robot.height_projection_color.empty() &&
                (robot.height_projection_color.size() != 7 || robot.height_projection_color[0] != '#' ||
                 robot.height_projection_color.find_first_not_of("0123456789abcdef", 1) != std::string::npos)) {
                throw std::runtime_error("heightProjectionColor must be canonical #rrggbb");
            }
            robot.ar_pose_topic = item.second.get<std::string>("arPoseTopic", "");
            robot.ar_path_topic = item.second.get<std::string>("arPathTopic", "");
            const auto offset = item.second.get_child_optional("worldOffset");
            if (offset) {
                if (offset->size() != 3) { throw std::runtime_error("worldOffset requires three coordinates"); }
                std::size_t index = 0;
                for (const auto& value : *offset) {
                    const double coordinate = value.second.get_value<double>();
                    if (!value.first.empty() || !std::isfinite(coordinate)) { throw std::runtime_error("worldOffset must be finite"); }
                    robot.world_offset[index++] = coordinate;
                }
            }
            if (robot.ar_pose_topic.empty() != robot.ar_path_topic.empty() ||
                (!robot.ar_path_topic.empty() && !canonicalRelativeROSName(robot.ar_path_topic)) ||
                (!robot.ar_pose_topic.empty() && (robot.ar_pose_topic.front() != '/' || !canonicalRelativeROSName(robot.ar_pose_topic.substr(1))))) {
                throw std::runtime_error("invalid AR history configuration");
            }
        } catch (const std::exception& exception) {
            *error = std::string("frozen Robot visualization roster entry is incomplete: ") +
                     exception.what();
            return false;
        }
        if (!canonicalROSIdentifier(robot.name) ||
            robot.ros_namespace != "/" + robot.name ||
            !canonicalROSPackage(robot.description_package) ||
            !canonicalDescriptionFile(robot.description_file) ||
            !canonicalRelativeROSName(robot.joint_state_topic) ||
            (!robot.scene_model.empty() && !canonicalROSIdentifier(robot.scene_model)) ||
            (!robot.odometry_topic.empty() && !canonicalRelativeROSName(robot.odometry_topic)) ||
            (!robot.path_topic.empty() && !canonicalRelativeROSName(robot.path_topic)) ||
            (!robot.scene_model.empty() && (robot.path_topic.empty() || robot.robot_state_publisher))) {
            *error = "frozen Robot visualization roster entry for " + robot.name +
                     " is not canonical";
            return false;
        }
        if (!names.insert(robot.name).second) {
            *error = "frozen Robot visualization roster repeats model " + robot.name;
            return false;
        }
        if (!robot.scene_model.empty() && !scene_models.insert(robot.scene_model).second) {
            *error = "frozen Robot visualization roster repeats scene model " + robot.scene_model;
            return false;
        }
        prepared.push_back(std::move(robot));
    }
    if (prepared.size() > 256) {
        *error = "frozen Robot visualization roster must contain at most 256 Robots";
        return false;
    }
    std::sort(prepared.begin(), prepared.end(),
              [](const RobotDescription& left, const RobotDescription& right) {
                  return left.name < right.name;
              });
    *robots = std::move(prepared);
    error->clear();
    return true;
}

}  // namespace xgc2_ros_visualizer
