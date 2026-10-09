#include <xgc2_ros_visualizer/instance_input.hpp>
#include <xgc2_ros_visualizer/config.hpp>
#include <algorithm>
#include <cerrno>
#include <climits>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <map>
#include <regex>
#include <set>
#include <stdexcept>
namespace xgc2_ros_visualizer {
namespace {
std::string text(const Json::Value& value,const std::string& name) {
  if(!value.isString())throw std::invalid_argument(name+" must be a string");return value.asString();
}
std::string optionalText(const Json::Value& value,const std::string& name) {return value.isNull()?"":text(value,name);}
bool flag(const Json::Value& value,bool fallback,const std::string& name) {
  if(value.isNull())return fallback;if(!value.isBool())throw std::invalid_argument(name+" must be boolean");return value.asBool();
}
std::string trim(std::string value) {
  const auto first=value.find_first_not_of(" \t\r\n");if(first==std::string::npos)return "";
  return value.substr(first,value.find_last_not_of(" \t\r\n")-first+1);
}
void arSource(Json::Value& description, const Json::Value& robot,
              const Json::Value& context) {
  const auto mode = text(context["runMode"], "context.runMode");
  const auto source = mode == "hybrid" ? text(robot["hybridSource"], "Robot hybridSource") : mode;
  if (source != "physical" && source != "simulation")
    throw std::invalid_argument("AR requires an explicit physical or simulation source");
  const auto space = robot["namespace"].asString();
  const bool rotor = robot["profileId"] == "px4.mocap-rotor.ros1.v1";
  const auto simulator = source == "simulation" && !rotor
      ? text(context["scene"]["simulator"], "context.scene.simulator") : "";
  if (source == "simulation" && !rotor && simulator != "xsim" && simulator != "gazebo")
    throw std::invalid_argument("AR requires an explicit supported simulator");
  const bool direct = simulator == "xsim";
  description["arPathTopic"] = "ar_path";
  if (direct) {
    const auto authored = optionalText(robot["simulationPoseTopic"], "simulationPoseTopic");
    description["arPoseTopic"] = authored.empty() ? space + "/pose" : authored;
    return;
  }
  const auto body = source == "physical" || rotor
      ? text(robot["px4"]["mocapRigidBodyName"], "px4.mocapRigidBodyName") : space.substr(1);
  if (body.empty() || body.find('/') != std::string::npos)
    throw std::invalid_argument("AR requires a single rigid body name");
  description["arPoseTopic"] = "/vrpn_client_node" +
      (mode == "hybrid" ? "_" + source : "") + "/" + body + "/pose";
  if (source == "physical" || rotor) {
    const auto& offset = context["localizationOffset"];
    for (int i = 0; i < 3; ++i) {
      const auto& v = offset[i == 0 ? "x" : i == 1 ? "y" : "z"];
      if (!v.isNumeric() || v.isBool() || !std::isfinite(v.asDouble()))
        throw std::invalid_argument("AR localization offset must be finite");
      description["worldOffset"][i] = v;
    }
  }
}
long slotNumber(const std::string& name) {
  std::size_t first=name.size();while(first>0&&std::isdigit(static_cast<unsigned char>(name[first-1])))--first;
  if(first==name.size())return 0;
  errno=0;char* end=nullptr;const long value=std::strtol(name.c_str()+first,&end,10);
  return errno||value>INT_MAX?0:value;
}
std::string heightColor(const Json::Value& settings,const Json::Value& robot,const std::vector<Json::Value>& ordered) {
  const auto kind=text(robot["kind"],"Robot kind");const char* key=kind=="scout_mini"?"scoutPalette":kind=="mecanum_ugv"?"mecanumPalette":"uavPalette";
  std::vector<std::string> palette;
  if(settings.isMember(key)) {
    const auto& colors=settings[key];if(!colors.isArray()||colors.empty()||colors.size()>32)throw std::invalid_argument(std::string(key)+" requires 1..32 colors");
    static const std::regex color("^#[0-9a-fA-F]{6}$");
    for(const auto& entry:colors) {auto value=text(entry,key);if(!std::regex_match(value,color))throw std::invalid_argument("invalid history palette color");std::transform(value.begin(),value.end(),value.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});palette.push_back(value);}
  } else if(kind=="px4_multirotor") palette={"#f2003c","#ff7043","#ab47bc","#ec407a","#7e57c2","#ef5350","#ffa726","#d4e157"};
  else if(kind=="scout_mini")palette={"#cbab01","#8bc34a","#ffca28","#66bb6a","#c0ca33","#26a69a","#d4a373","#a1887f"};
  else if(kind=="mecanum_ugv")palette={"#288f8c","#29b6f6","#5c6bc0","#26c6da","#42a5f5","#7e57c2","#80cbc4","#90caf9"};
  if(palette.empty())return kind=="scout_mini"?"#cbab01":kind=="mecanum_ugv"?"#288f8c":"#f2003c";
  std::vector<std::string> peers;
  for(const auto& candidate:ordered)if(candidate["kind"]==robot["kind"])peers.push_back(candidate["namespace"].asString().substr(1));
  std::sort(peers.begin(),peers.end(),[](const std::string& a,const std::string& b){auto x=slotNumber(a),y=slotNumber(b);return x==y?a<b:x<y;});
  const auto name=robot["namespace"].asString().substr(1);auto found=std::find(peers.begin(),peers.end(),name);
  return palette[static_cast<std::size_t>(found-peers.begin())%palette.size()];
}
}
Json::Value projectInstanceInput(const Json::Value& value) {
  if(!value.isObject())throw std::invalid_argument("instance input must be an object");
  const std::set<std::string> fields{"robots","context","settings","displayRelays"};auto keys=value.getMemberNames();
  if(std::set<std::string>(keys.begin(),keys.end())!=fields)throw std::invalid_argument("instance input requires exactly robots/context/settings/displayRelays");
  static const std::regex ns("^/[A-Za-z_][A-Za-z0-9_]{0,126}$");
  const auto& context=value["context"];const auto& panel=value["settings"];
  if(!context.isObject()||!panel.isObject()||!value["robots"].isArray()||value["robots"].size()>256)throw std::invalid_argument("instance context/settings/robots types are invalid");
  const auto mode=text(context["runMode"],"context.runMode"),clock=text(context["worldClock"],"context.worldClock");
  if(mode!="simulation"&&mode!="physical"&&mode!="hybrid")throw std::invalid_argument("invalid frozen runMode");
  if(clock!="simulation"&&clock!="wall")throw std::invalid_argument("invalid frozen worldClock");
  Json::Value request;request["robots"]=Json::Value(Json::arrayValue);request["descriptions"]=Json::Value(Json::arrayValue);request["displayRelays"]=Json::Value(Json::arrayValue);
  request["worldBoundary"]=context["worldBoundary"];auto& settings=request["settings"];
  settings["frame_id"]="world";settings["use_sim_time"]=clock=="simulation";
  settings["publish_markers"]=false;settings["publish_transforms"]=true;settings["publish_scene_update"]=true;settings["publish_scene_paths"]=false;settings["publish_paths"]=true;
  // Publication controls are product data, independent of the frozen Viewer
  // style/context fields. This preserves explicit relay-only and ground-scene
  // controls without accepting the retired projected wire document.
  if(panel.isMember("publication")) {
    const auto& publication=panel["publication"];
    if(!publication.isObject())throw std::invalid_argument("settings.publication must be an object");
    const std::map<std::string,std::string> fields{{"markers","publish_markers"},{"transforms","publish_transforms"},{"scene","publish_scene_update"},{"scenePaths","publish_scene_paths"},{"paths","publish_paths"},{"groundScene","track_ugv"}};
    for(const auto& field:publication.getMemberNames()) {
      const auto target=fields.find(field);
      if(target==fields.end()||!publication[field].isBool())throw std::invalid_argument("unknown or nonboolean publication control");
      settings[target->second]=publication[field];
    }
  }
  settings["scene_update_topic"]="/xgc/scene";settings["transform_topic"]="/xgc/tf";
  const std::pair<const char*,const char*> style[]={{"markerColor","marker_color"},{"worldBoundaryMode","world_boundary_mode"},{"labelScaleInvariant","label_scale_invariant"},{"labelFontSizeMeters","label_font_size_meters"},{"labelFontSizePixels","label_font_size_pixels"},{"markerOpacity","marker_opacity"},{"uavLabelOffset","uav_label_offset"},{"scoutLabelOffset","scout_label_offset"},{"mecanumLabelOffset","mecanum_label_offset"}};
  for(const auto& field:style)if(panel.isMember(field.first))settings[field.second]=panel[field.first];
  if(!settings.isMember("marker_color")||settings["marker_color"]=="")settings["marker_color"]="#00a2ff";
  std::vector<Json::Value> ordered;std::set<std::string> namespaces;
  for(const auto& robot:value["robots"]) {
    if(!robot.isObject())throw std::invalid_argument("frozen Robot must be an object");
    const auto name=text(robot["namespace"],"Robot namespace");text(robot["kind"],"Robot kind");
    if(!std::regex_match(name,ns)||!namespaces.insert(name).second)throw std::invalid_argument("invalid or repeated frozen Robot namespace");
    ordered.push_back(robot);
  }
  std::sort(ordered.begin(),ordered.end(),[](const Json::Value& a,const Json::Value& b){return a["namespace"].asString()<b["namespace"].asString();});
  std::array<std::vector<std::string>,3> models;
  std::map<std::string,std::string> classes;
  for(const auto& robot:ordered) {
    const auto space=robot["namespace"].asString(),name=space.substr(1);const auto& visual=robot["visualization"];
    if(!visual.isNull()&&!visual.isObject())throw std::invalid_argument("Robot visualization must be an object");
    const auto scene_class=optionalText(visual["sceneClass"],"sceneClass");
    if(!scene_class.empty()&&scene_class!="fs150"&&scene_class!="scout"&&scene_class!="mecanum")throw std::invalid_argument("unsupported concrete sceneClass");
    classes[space]=scene_class.empty()?"global":scene_class;
    const auto package=optionalText(visual["descriptionPackage"],"descriptionPackage");
    if(package.empty()) {if(!scene_class.empty())throw std::invalid_argument("sceneClass requires an installed description");continue;}
    Json::Value description;
    description["name"]=name;description["namespace"]=space;description["descriptionPackage"]=package;
    description["descriptionFile"]=text(visual["descriptionFile"],"descriptionFile");
    description["robotStatePublisher"]=flag(visual["robotStatePublisher"],false,"robotStatePublisher");
    description["jointStateTopic"]=text(visual["jointStateTopic"],"jointStateTopic");
    description["odometryTopic"]=optionalText(visual["odometryTopic"],"odometryTopic");description["pathTopic"]=optionalText(visual["pathTopic"],"pathTopic");
    description["worldOffset"]=Json::Value(Json::arrayValue);for(int i=0;i<3;++i)description["worldOffset"].append(0.0);
    auto model=optionalText(visual["sceneModel"],"sceneModel");
    if(robot["kind"]=="scout_mini"&&robot["scout"].isObject()) {
      const auto mocap=trim(optionalText(robot["scout"]["mocapRigidBodyName"],"mocapRigidBodyName"));
      model=(mocap.empty()||mocap==name||name.compare(0,3,"ugv")==0)?"":mocap;
    }
    description["sceneModel"]=scene_class.empty()?"":model.empty()?name:model;
    request["descriptions"].append(description);
    if(scene_class.empty())continue;
    const std::size_t index=scene_class=="fs150"?0:scene_class=="scout"?1:2;models[index].push_back(description["sceneModel"].asString());
    if(scene_class=="fs150") {
      arSource(description, robot, context);
      if(flag(panel["uavHeightProjection"],true,"uavHeightProjection"))description["heightProjectionColor"]=heightColor(panel,robot,ordered);
    }
    request["robots"].append(description);
  }
  const char* model_keys[]={"tracked_fs150_models","tracked_scout_models","tracked_mecanum_models"};
  for(std::size_t i=0;i<3;++i) {std::sort(models[i].begin(),models[i].end());std::string csv;for(const auto& model:models[i]){if(!csv.empty())csv+=',';csv+=model;}settings[model_keys[i]]=csv;}
  if(!value["displayRelays"].isArray())throw std::invalid_argument("displayRelays must be an array");
  for(const auto& relay:value["displayRelays"]) {
    if(!relay.isObject())throw std::invalid_argument("display relay must be an object");
    const std::set<std::string> allowed{"source","topic","messageType"};const auto fields=relay.getMemberNames();if(std::set<std::string>(fields.begin(),fields.end())!=allowed)throw std::invalid_argument("display relay requires exactly source/topic/messageType");
    Json::Value projected;projected["source"]=text(relay["source"],"relay source");projected["topic"]=text(relay["topic"],"relay topic");projected["messageType"]=text(relay["messageType"],"relay messageType");
    std::string kind="global";const auto source=projected["source"].asString();for(const auto& item:classes)if(source.compare(0,item.first.size()+1,item.first+"/")==0)kind=item.second;
    projected["robotKind"]=kind;request["displayRelays"].append(projected);
  }
  parseInstance(request);return request;
}
} // namespace xgc2_ros_visualizer
