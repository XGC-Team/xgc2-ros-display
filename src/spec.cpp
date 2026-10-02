#include <xgc2_ros_display_relays/display_relays.hpp>
#include <json/json.h>
#include <cmath>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
namespace xgc2_ros_display_relays {
void validateRelaySpecs(const std::vector<RelaySpec>& specs) {
  static const std::regex topic("^/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$");
  static const std::set<std::string> types{"sensor_msgs/PointCloud2", "nav_msgs/OccupancyGrid", "nav_msgs/Path", "geometry_msgs/PoseArray"};
  std::set<std::string> sources;
  if (specs.size() > 64) throw std::invalid_argument("display relays must be a list of at most 64 relays");
  for (const auto& spec : specs) {
    if (spec.source.size() + 12 > 511 || !std::regex_match(spec.source, topic) ||
        spec.source == "/xgc/display" || spec.source.compare(0, 13, "/xgc/display/") == 0 || !sources.insert(spec.source).second)
      throw std::invalid_argument("display relay source is invalid or repeated: " + spec.source);
    if (spec.topic != "/xgc/display" + spec.source || !types.count(spec.message_type))
      throw std::invalid_argument("display relay needs its canonical copy and a full-state message type");
    if (!std::isfinite(spec.max_rate_hz) || spec.max_rate_hz < 0.1 || spec.max_rate_hz > 100)
      throw std::invalid_argument("display relay rate is invalid");
  }
}
std::vector<RelaySpec> parseRelaySpecs(const std::string& text) {
  Json::CharReaderBuilder builder;
  builder["allowComments"] = false; builder["failIfExtra"] = true; builder["rejectDupKeys"] = true;
  Json::Value root; std::string errors; std::istringstream input(text);
  if (!Json::parseFromStream(builder, input, &root, &errors) || !root.isArray() || root.size() > 64)
    throw std::invalid_argument("display relays must be a list of at most 64 relays: " + errors);
  std::vector<RelaySpec> specs;
  const std::set<std::string> fields{"source", "topic", "messageType", "maxRateHz"};
  for (const auto& item : root) {
    if (!item.isObject()) throw std::invalid_argument("each display relay must be an object");
    const auto names = item.getMemberNames();
    if (std::set<std::string>(names.begin(), names.end()) != fields ||
        !item["source"].isString() || !item["topic"].isString() || !item["messageType"].isString() ||
        !item["maxRateHz"].isNumeric() || item["maxRateHz"].isBool())
      throw std::invalid_argument("each display relay needs source, topic, messageType and numeric maxRateHz");
    specs.push_back({item["source"].asString(), item["topic"].asString(), item["messageType"].asString(), item["maxRateHz"].asDouble()});
  }
  validateRelaySpecs(specs); return specs;
}
}
