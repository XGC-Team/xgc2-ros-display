#include "rpc.hpp"
#include <xgc2/xrpc/bounded_output.hpp>
#include <xgc2/xrpc/http.hpp>
#include <xgc2/xrpc/diagnostics.hpp>
#include <xgc2/xrpc/runtime_policy.hpp>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <regex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace xgc2_ros_visualizer {
namespace {
using namespace xgc2::xrpc;
RuntimePolicy resolvePolicy(const RpcOptions& options) {
  RuntimePolicyOptions input;
  input.environment = options.environment;
  input.capabilities.push_back("diagnostics");
  input.default_source = "ros-visualizer control host";
  input.defaults = {{"HOST_MAX_CONNECTIONS", "32"}, {"HOST_MAX_IN_FLIGHT", "32"},
      {"MAX_HEADER_BYTES", "16384"}, {"MAX_REQUEST_BYTES", "1048576"},
      {"MAX_RESPONSE_BYTES", "1048576"}, {"CALL_TIMEOUT_MS", "5000"},
      {"HEADER_TIMEOUT_MS", "5000"}, {"IDLE_TIMEOUT_MS", "5000"},
      {"SHUTDOWN_TIMEOUT_MS", "5000"}};
  // Product memory ceilings constrain the common policy, without another set
  // of defaults or silently clamping a deployment's requested value.
  input.ceilings = {{"HOST_MAX_CONNECTIONS", 32}, {"HOST_MAX_IN_FLIGHT", 32},
      {"MAX_HEADER_BYTES", 16384}, {"MAX_REQUEST_BYTES", 1048576},
      {"MAX_RESPONSE_BYTES", 1048576}, {"CALL_TIMEOUT_MS", 5000},
      {"HEADER_TIMEOUT_MS", 5000}, {"IDLE_TIMEOUT_MS", 5000},
      {"SHUTDOWN_TIMEOUT_MS", 5000}};
  return resolve_runtime_policy(input);
}
UnixOptions socketOptions(std::string path) {
  UnixOptions result;
  result.path = std::move(path);
  result.existing = ExistingPath::ReclaimUnreachable;
  return result;
}
Json::Value policyJson(const RuntimePolicySnapshot& policy) {
  Json::Value result;
  result["revision"] = Json::UInt64(policy.revision);
  for (const auto& field : policy.entries()) {
    auto& output = result["fields"][std::string(field.name)];
    if (const auto* integer = std::get_if<std::int64_t>(&field.value))
      output["value"] = Json::Int64(*integer);
    else output["value"] = std::get<std::string>(field.value);
    output["source"] = std::string(field.source);
    output["sourceDetail"] = field.source_detail;
    output["dynamic"] = field.dynamic;
    output["unit"] = std::string(field.unit);
    if (field.ceiling) output["ceiling"] = Json::Int64(*field.ceiling);
  }
  return result;
}
Json::Value parseRequest(const std::string& text) {
  if (text.empty()) return Json::Value();
  Json::CharReaderBuilder builder;
  builder["allowComments"] = false;
  builder["collectComments"] = false;
  builder["allowTrailingCommas"] = false;
  builder["rejectDupKeys"] = true;
  builder["failIfExtra"] = true;
  builder["allowSpecialFloats"] = false;
  builder["stackLimit"] = 64;
  std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
  Json::Value result;
  std::string detail;
  if (!reader->parse(text.data(), text.data() + text.size(), &result, &detail))
    throw std::invalid_argument("invalid JSON request");
  return result;
}
void complete(const HttpReply& reply, RpcReply result, std::size_t limit) {
  BoundedOutput output(limit);
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "";
  std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
  writer->write(result.body, &output.stream());
  if (!output.good()) {
    reply.complete(http_error(500, "resource_exhausted", "JSON response exceeds limit"));
    return;
  }
  HttpResponse response;
  response.status = result.status;
  response.headers.emplace_back("Content-Type", "application/json");
  response.body = output.value();
  reply.complete(std::move(response));
}
}

class RpcServer::Impl {
 public:
  Impl(std::string path, RpcHandler handler, RpcOptions options, std::function<void()> quiesce_native)
      : policy_(resolvePolicy(options)), limits_(http_limits(policy_)),
        handler_(std::move(handler)), slots_(limits_.inflight),
        diagnostics_(policy_),
        quiesce_native_(std::move(quiesce_native)),
        host_(socketOptions(path), [this](HttpRequest request, HttpReply reply) {
          dispatch(std::move(request), std::move(reply));
        }, limits_, HttpIdentity{options.instance_id, {"/v1/describe"}}, options.retained_parent_fd) {
    if (!handler_) throw std::invalid_argument("RPC handler is required");
    static const std::regex target("^[A-Za-z0-9_.:-]{1,128}$");
    if (!std::regex_match(options.target_id, target))
      throw std::invalid_argument("target_id requires 1..128 ASCII identity characters");
    if (options.instance_id.empty()) throw std::invalid_argument("instance_id is required");
    auto& ref = description_["service_ref"];
    ref["target_id"] = options.target_id;
    ref["service"] = "xgc2.visualization";
    ref["api_version"] = "1";
    ref["instance_id"] = options.instance_id;
    ref["profile"] = "http.v1";
    ref["endpoint"]["kind"] = "unix";
    ref["endpoint"]["address"] = path;
    description_["domain"]["schema"] = "visualization-v1";
    description_["domain"]["maxInstances"] = 64;
    description_["domain"]["rates"]["revisionChecked"] = true;
    description_["domain"]["rates"]["persistence"] = false;
    description_["domain"]["instances"]["immutableConfiguration"] = true;
    description_["domain"]["instances"]["inputSchema"] = "frozen-visualization-input-v1";
    description_["domain"]["instances"]["persistence"] = false;
    description_["resources"]["domainWorkers"] = 1;
    description_["resources"]["domainQueueCapacity"] = Json::UInt64(slots_.size());
    description_["policy"] = effectivePolicy();
    storage_["configuration"]["backend"] = "memory";
    storage_["configuration"]["durable"] = false;
    storage_["runtime"]["socket"] = path;
    storage_["runtime"]["lock"] = path + ".xrpc.lock";
    storage_["runtime"]["writer"] = "xgc2-xrpc";
    storage_["rosCache"]["root"] = options.ros_home;
    storage_["rosCache"]["writer"] = "ROS libraries";
    storage_["rosLogs"]["root"] = options.ros_log_dir;
    storage_["rosLogs"]["writer"] = "ROS libraries";
    storage_["rosLogs"]["quotaAndRotationOwner"] = "process supervisor";
    description_["storage"] = storage_;
    host_.set_diagnostics(&diagnostics_, "xgc2.visualization");
    worker_ = std::thread([this] { work(); });
  }
  ~Impl() { stop(); join(); quiesce(); }
  Json::Value effectivePolicy() const {
    auto result=policyJson(policy_.effective());
    const auto logging=policyJson(diagnostics_.effective_policy());
    result["revision"]=logging["revision"];
    for(const auto& key:logging["fields"].getMemberNames())result["fields"][key]=logging["fields"][key];
    return result;
  }

  void dispatch(HttpRequest request, HttpReply reply) {
    if (request.target == "/v1/describe" || request.target == "/v1/xrpc/policy" ||
        request.target == "/v1/xrpc/status" || request.target == "/v1/xrpc/storage" ||
        request.target == "/v1/xrpc/diagnostics" || request.target == "/v1/xrpc/log-level") {
      if(request.target=="/v1/xrpc/log-level"&&request.method=="PUT") {
        try {
          if(request.body.size()>256)throw std::invalid_argument("log-level update exceeds 256 bytes");
          const auto body=parseRequest(request.body);
          if(!body.isObject())throw std::invalid_argument("log-level update must be an object");
          const auto names=body.getMemberNames();
          if(!body.isObject()||names.size()!=2||!body.isMember("expectedRevision")||!body.isMember("level")||
              (body["expectedRevision"].type()!=Json::intValue&&body["expectedRevision"].type()!=Json::uintValue)||
              !body["expectedRevision"].isUInt64()||!body["expectedRevision"].asUInt64()||!body["level"].isString())
            throw std::invalid_argument("log-level update requires positive expectedRevision and level");
          DiagnosticPolicyUpdate update;
          const auto level=body["level"].asString();
          if(level=="error")update.level=LogSeverity::Error;
          else if(level=="warn")update.level=LogSeverity::Warn;
          else if(level=="info")update.level=LogSeverity::Info;
          else if(level=="debug")update.level=LogSeverity::Debug;
          else if(level=="trace")update.level=LogSeverity::Trace;
          else throw std::invalid_argument("invalid log level");
          const auto changed=diagnostics_.update(body["expectedRevision"].asUInt64(),update);
          if(changed==DiagnosticUpdateResult::RevisionConflict) {
            reply.complete(http_error(409,"conflict","diagnostic policy revision conflict"));
          } else if(changed!=DiagnosticUpdateResult::Applied) {
            reply.complete(http_error(400,"invalid_argument","unsupported diagnostic policy update"));
          } else complete(reply,{200,effectivePolicy()},limits_.response_bytes);
        } catch(const std::invalid_argument& error) {
          reply.complete(http_error(400,"invalid_argument",error.what()));
        }
        return;
      }
      if (request.method != "GET") {
        reply.complete(http_error(405, "invalid_argument", "resource supports GET"));
      } else if (!request.body.empty()) {
        reply.complete(http_error(400, "invalid_argument", "GET resource requires an empty body"));
      } else if (request.target == "/v1/describe") {
        auto description=description_;description["policy"]=effectivePolicy();
        complete(reply, {200, std::move(description)}, limits_.response_bytes);
      } else if (request.target == "/v1/xrpc/policy") {
        complete(reply, {200, effectivePolicy()}, limits_.response_bytes);
      } else if (request.target == "/v1/xrpc/storage") {
        complete(reply, {200, storage_}, limits_.response_bytes);
      } else if (request.target == "/v1/xrpc/log-level") {
        complete(reply, {200, policyJson(diagnostics_.effective_policy())}, limits_.response_bytes);
      } else if (request.target == "/v1/xrpc/diagnostics") {
        // Draining is explicit through this private endpoint. No file sink,
        // unbounded diagnostic queue, IO-thread stderr write or payload capture.
        Json::Value result;
        result["records"]=Json::Value(Json::arrayValue);
        const auto maximum=std::min<std::size_t>(16,(limits_.response_bytes-256)/(2*diagnostic_output_capacity));
        diagnostics_.drain(maximum,[](void* state,std::string_view record){
          static_cast<Json::Value*>(state)->append(std::string(record));return true;
        },&result["records"]);
        const auto stats=diagnostics_.stats();
        result["dropped"]=Json::UInt64(stats.dropped());result["queued"]=Json::UInt64(stats.queued);
        result["capacity"]=Json::UInt64(stats.capacity);
        complete(reply,{200,std::move(result)},limits_.response_bytes);
      } else {
        const auto stats = host_.stats();
        Json::Value result;
        result["sourceSteadyNs"] = Json::Int64(std::chrono::duration_cast<std::chrono::nanoseconds>(
            stats.source_time.time_since_epoch()).count());
        result["stopping"] = stats.stopping;
        result["activeConnections"] = Json::UInt64(stats.active_connections);
        result["inflightCalls"] = Json::UInt64(stats.inflight_calls);
        result["acceptedConnections"] = Json::UInt64(stats.accepted_connections);
        result["rejectedConnections"] = Json::UInt64(stats.rejected_connections);
        result["admittedCalls"] = Json::UInt64(stats.admitted_calls);
        result["rejectedCalls"] = Json::UInt64(stats.rejected_calls);
        result["completedCalls"] = Json::UInt64(stats.completed_calls);
        result["deadlineExceeded"] = Json::UInt64(stats.deadline_exceeded);
        result["idleTimeouts"] = Json::UInt64(stats.idle_timeouts);
        result["cancelledCalls"] = Json::UInt64(stats.cancelled_calls);
        result["malformedRequests"] = Json::UInt64(stats.malformed_requests);
        result["peerErrors"] = Json::UInt64(stats.peer_errors);
        const auto logging=diagnostics_.stats();
        result["diagnosticQueueCapacity"]=Json::UInt64(logging.capacity);
        result["diagnosticsQueued"]=Json::UInt64(logging.queued);
        result["diagnosticsDropped"]=Json::UInt64(logging.dropped());
        {
          std::lock_guard<std::mutex> lock(mutex_);
          result["queuedDomainCalls"] = Json::UInt64(size_);
          result["activeDomainCalls"] = busy_ ? 1 : 0;
        }
        complete(reply, {200, result}, limits_.response_bytes);
      }
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_.load()) {
        reply.complete(http_error(503, "unavailable", "visualizer is stopping"));
        return;
      }
      if (size_ == slots_.size()) {
        reply.complete(http_error(503, "resource_exhausted", "domain handoff is full"));
        return;
      }
      slots_[tail_].emplace(Work{std::move(request), std::move(reply)});
      tail_ = (tail_ + 1) % slots_.size();
      ++size_;
    }
    wake_.notify_one();
  }
  void work() noexcept {
    for (;;) {
      std::optional<Work> current;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock, [this] { return size_ || stopping_.load(); });
        if (!size_) return;
        current = std::move(slots_[head_]);
        slots_[head_].reset();
        head_ = (head_ + 1) % slots_.size();
        --size_;
        busy_ = true;
      }
      try {
        if (stopping_.load()) {
          current->reply.complete(http_error(503, "unavailable", "visualizer is stopping"));
        } else if (!current->reply.cancelled() && Clock::now() < current->request.deadline) {
          if ((current->request.method == "GET" || current->request.method == "DELETE") &&
              !current->request.body.empty())
            throw std::invalid_argument("GET and DELETE require an empty body");
          const auto body = parseRequest(current->request.body);
          // Admission does not assert native completion. Once started, native
          // work remains owned even when the peer/deadline cancels its reply.
          if(!current->reply.cancelled()&&Clock::now()<current->request.deadline)
            complete(current->reply, handler_(current->request.method, current->request.target, body),
                     limits_.response_bytes);
        }
      } catch (const std::invalid_argument& error) {
        current->reply.complete(http_error(400, "invalid_argument", error.what()));
      } catch (...) {
        current->reply.complete(http_error(500, "internal", "internal RPC handler failure"));
      }
      current.reset();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        busy_ = false;
      }
    }
  }
  void stop() noexcept {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_.store(true);
    }
    // Wake the owner without releasing the endpoint. Its native quiescence
    // phase still owns the lease; only the later SDK drain closes it.
    host_.wake();
    wake_.notify_one();
  }
  void join() { if (worker_.joinable()) worker_.join(); }
  void quiesce() {
    if(!native_quiesced_) {
      native_quiesced_=true;
      if(quiesce_native_)quiesce_native_();
    }
  }
  void run(const std::atomic<bool>& external_stop) {
    while (!external_stop.load() && !stopping_.load())
      host_.poll(std::chrono::milliseconds(50));
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_.store(true);
    }
    wake_.notify_one();
    join();
    // No new RPC is dispatched while this owner is outside poll(). Complete
    // native workers and publication fences before initiating SDK lease close.
    quiesce();
    host_.drain();
  }
  const std::string& socket_path() const noexcept { return host_.socket_path(); }
 private:
  struct Work { HttpRequest request; HttpReply reply; };
  RuntimePolicy policy_;
  HttpLimits limits_;
  RpcHandler handler_;
  std::vector<std::optional<Work>> slots_;
  Json::Value description_, storage_;
  Diagnostics diagnostics_;
  std::function<void()> quiesce_native_;
  bool native_quiesced_{false};
  std::mutex mutex_;
  std::condition_variable wake_;
  std::size_t head_{0}, tail_{0}, size_{0};
  bool busy_{false};
  std::atomic<bool> stopping_{false};
  HttpServer host_;
  std::thread worker_;
};

RpcServer::RpcServer(std::string path, RpcHandler handler, RpcOptions options, std::function<void()> quiesce_native)
    : impl_(new Impl(std::move(path), std::move(handler), std::move(options),std::move(quiesce_native))) {}
RpcServer::~RpcServer() = default;
std::string RpcServer::newInstanceId() { return xgc2::xrpc::new_instance_id(); }
void RpcServer::run(const std::atomic<bool>& stopping) { impl_->run(stopping); }
void RpcServer::stop() noexcept { impl_->stop(); }
const std::string& RpcServer::socket_path() const noexcept { return impl_->socket_path(); }
} // namespace xgc2_ros_visualizer
