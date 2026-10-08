#include "../src/server/rpc.hpp"
#include <xgc2/xrpc/http.hpp>
#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <future>
#include <fcntl.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
namespace xgc2_ros_visualizer { namespace {
using namespace std::chrono;
class RpcTransportTest : public ::testing::Test {
 protected:
  void SetUp() override {
    char pattern[] = "/tmp/xgc2-viz-xrpc-XXXXXX";
    const char* directory = ::mkdtemp(pattern); ASSERT_NE(nullptr, directory);
    directory_ = directory; path_ = directory_ + "/control.sock";
    directory_fd_=::open(directory_.c_str(),O_RDONLY|O_DIRECTORY|O_CLOEXEC); ASSERT_GE(directory_fd_,0);
  }
  void TearDown() override {
    if (server_) server_->stop();
    if (worker_.joinable()) worker_.join();
    client_.reset(); server_.reset();
    ::unlink((path_ + ".xrpc.lock").c_str()); if(directory_fd_>=0)::close(directory_fd_); EXPECT_EQ(0, ::rmdir(directory_.c_str()));
  }
  void start(RpcHandler handler = {}, std::vector<std::pair<std::string,std::string>> environment = {}, std::function<void()> quiesce_native = {}) {
    if (!handler) handler = [this](const std::string& method, const std::string& path, const Json::Value& body) {
      ++calls_; Json::Value result; result["method"] = method; result["path"] = path; result["body"] = body;
      return RpcReply{path == "/missing" ? 404 : 200, result};
    };
    RpcOptions options;options.retained_parent_fd=directory_fd_; options.target_id="test-target"; options.instance_id="test-incarnation"; options.environment=std::move(environment);
    server_.reset(new RpcServer(path_, std::move(handler), std::move(options),std::move(quiesce_native)));
    client_.reset(new xgc2::xrpc::HttpClient(path_, {}, "test-incarnation"));
    worker_ = std::thread([this] { server_->run(stopping_); });
  }
  xgc2::xrpc::HttpResponse exchange(const std::string& method, const std::string& path, const std::string& body = {}) {
    xgc2::xrpc::HttpRequest request; request.method = method; request.target = path; request.body = body;
    return client_->call(std::move(request), xgc2::xrpc::Clock::now() + seconds(2));
  }
  Json::Value json(const xgc2::xrpc::HttpResponse& response) {
    Json::Value result; Json::CharReaderBuilder builder; std::string error;
    const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    EXPECT_TRUE(reader->parse(response.body.data(), response.body.data() + response.body.size(), &result, &error)); return result;
  }
  std::string directory_, path_; int directory_fd_{-1};
  std::atomic<bool> stopping_{false}; std::atomic<int> calls_{0};
  std::unique_ptr<RpcServer> server_; std::unique_ptr<xgc2::xrpc::HttpClient> client_; std::thread worker_;
};
TEST_F(RpcTransportTest, PreservesJsonAndRoutingAcrossReusedConnection) {
  start(); EXPECT_EQ(path_, server_->socket_path());
  EXPECT_TRUE(json(exchange("GET", "/v1/health"))["body"].isNull());
  const auto response = exchange("PUT", "/v1/instances/a", "{\"robots\":[],\"id\":17}");
  EXPECT_EQ(200, response.status); EXPECT_EQ("PUT", json(response)["method"].asString()); EXPECT_EQ(17, json(response)["body"]["id"].asInt());
  EXPECT_EQ(404, exchange("GET", "/missing").status); EXPECT_EQ(3, calls_.load());
}
TEST_F(RpcTransportTest, RejectsAmbiguousJsonBeforeBusinessHandler) {
  start();
  for (const auto* body : {"{\"a\":1,\"a\":2}", "{}{}", "{\"a\":NaN}", "{\"a\":1,}", "/*comment*/{}"})
    EXPECT_EQ(400, exchange("POST", "/entities", body).status) << body;
  EXPECT_EQ(0, calls_.load()); EXPECT_EQ(200, exchange("POST", "/entities", "{}").status);
}
TEST_F(RpcTransportTest, MapsHandlerFailureWithoutPrivateDetails) {
  start([](const std::string&, const std::string&, const Json::Value&) -> RpcReply { throw std::runtime_error("private-password"); });
  const auto response = exchange("GET", "/v1/health"); EXPECT_EQ(500, response.status);
  EXPECT_EQ(std::string::npos, response.body.find("private-password"));
}
TEST_F(RpcTransportTest, BoundsSerializedBusinessResponse) {
  start([](const std::string&, const std::string&, const Json::Value&) { Json::Value result; result["large"] = std::string(1048576, 'x'); return RpcReply{200, result}; });
  const auto response = exchange("GET", "/large"); EXPECT_EQ(500, response.status);
  EXPECT_LT(response.body.size(), 256u); EXPECT_NE(std::string::npos, response.body.find("resource_exhausted"));
}
TEST_F(RpcTransportTest, DiscoversActualBindingAndFencesEveryDomainRequest) {
  start();
  xgc2::xrpc::HttpClient unbound(path_);
  xgc2::xrpc::HttpRequest discovery; discovery.method="GET"; discovery.target="/v1/describe";
  const auto response=unbound.call(discovery,xgc2::xrpc::Clock::now()+seconds(2));
  ASSERT_EQ(200,response.status); const auto reference=json(response)["service_ref"];
  EXPECT_EQ("test-incarnation",reference["instance_id"].asString());
  EXPECT_EQ("test-target",reference["target_id"].asString());
  EXPECT_EQ(path_,reference["endpoint"]["address"].asString());
  discovery.target="/v1/health";
  EXPECT_EQ(409,unbound.call(discovery,xgc2::xrpc::Clock::now()+seconds(2)).status);
  xgc2::xrpc::HttpClient stale(path_,{},"old-incarnation");
  EXPECT_THROW(stale.call(discovery,xgc2::xrpc::Clock::now()+seconds(2)),xgc2::xrpc::HttpCallError);
  EXPECT_EQ(0,calls_.load());
  EXPECT_EQ(200,exchange("GET","/v1/health").status);
}
TEST_F(RpcTransportTest, AppliesOneResolvedPolicyAndRejectsUnsupportedSettings) {
  start({},{{"XGC2_XRPC_HOST_MAX_CONNECTIONS","3"},{"XGC2_XRPC_MAX_REQUEST_BYTES","256"}});
  const auto policy=json(exchange("GET","/v1/xrpc/policy"));
  EXPECT_EQ(3,policy["fields"]["HOST_MAX_CONNECTIONS"]["value"].asInt());
  EXPECT_EQ("environment",policy["fields"]["HOST_MAX_CONNECTIONS"]["source"].asString());
  EXPECT_EQ(32,policy["fields"]["HOST_MAX_CONNECTIONS"]["ceiling"].asInt());
  const auto large=exchange("PUT","/entities",std::string(257,'x'));
  EXPECT_EQ(413,large.status); EXPECT_EQ(0,calls_.load());
  const auto stats=json(exchange("GET","/v1/xrpc/status"));
  EXPECT_GE(stats["admittedCalls"].asUInt64(),2u);
  EXPECT_EQ(0,stats["activeDomainCalls"].asInt());
}
TEST_F(RpcTransportTest, RejectsInvalidRuntimePolicyBeforeEndpointAdmission) {
  for(const auto& setting:std::vector<std::pair<std::string,std::string>>{
      {"XGC2_XRPC_HOST_MAX_CONNECTIONS","33"}, {"XGC2_XRPC_HOST_MAX_CONNECTIONS",""},
      {"XGC2_XRPC_LOG_QUEUE_BYTES","1024"}, {"XGC2_XRPC_UNDECLARED","1"}}) {
    RpcOptions options;options.retained_parent_fd=directory_fd_;options.target_id="test-target";options.instance_id="test-instance";options.environment.push_back(setting);
    EXPECT_THROW(RpcServer(path_,[](const std::string&,const std::string&,const Json::Value&){return RpcReply{200,{}};},options),std::exception);
    EXPECT_NE(0,::access(path_.c_str(),F_OK));
  }
}
TEST_F(RpcTransportTest, DomainWorkDoesNotBlockPolicyAndCancelledQueuedWorkNeverStarts) {
  std::promise<void> entered,release;auto unlocked=release.get_future().share();
  std::atomic<int> mutations{0};
  start([&,unlocked](const std::string&,const std::string&,const Json::Value&){
    if(++mutations==1){entered.set_value();unlocked.wait_for(seconds(3));}
    return RpcReply{200,Json::Value(Json::objectValue)};
  });
  auto first=std::async(std::launch::async,[&]{return exchange("PUT","/entities","{}");});
  EXPECT_EQ(std::future_status::ready,entered.get_future().wait_for(seconds(1)));
  xgc2::xrpc::HttpClient queued(path_,{},"test-incarnation");
  xgc2::xrpc::HttpRequest mutation;mutation.method="PUT";mutation.target="/entities";mutation.body="{}";
  EXPECT_THROW(queued.call(mutation,xgc2::xrpc::Clock::now()+milliseconds(80)),xgc2::xrpc::HttpCallError);
  xgc2::xrpc::HttpClient observer(path_,{},"test-incarnation");
  xgc2::xrpc::HttpRequest observation;observation.method="GET";observation.target="/v1/xrpc/status";
  const auto status=json(observer.call(observation,xgc2::xrpc::Clock::now()+seconds(1)));
  EXPECT_EQ(1,status["activeDomainCalls"].asInt());
  release.set_value();EXPECT_EQ(200,first.get().status);
  // A subsequent domain request is an ordering barrier on the single worker.
  EXPECT_EQ(200,exchange("GET","/v1/health").status);EXPECT_EQ(2,mutations.load());
}
TEST_F(RpcTransportTest, StopRetainsLeaseUntilNativeWorkReallyEnds) {
  std::promise<void> entered,release;auto unlocked=release.get_future().share();
  start([&,unlocked](const std::string&,const std::string&,const Json::Value&){entered.set_value();unlocked.wait_for(seconds(3));return RpcReply{200,{}};});
  auto active=std::async(std::launch::async,[&]{try{exchange("PUT","/entities","{}");}catch(const std::exception&){};});
  EXPECT_EQ(std::future_status::ready,entered.get_future().wait_for(seconds(1)));
  server_->stop();
  RpcOptions options;options.retained_parent_fd=directory_fd_;options.target_id="test-target";options.instance_id="replacement";
  EXPECT_THROW(RpcServer(path_,[](const std::string&,const std::string&,const Json::Value&){return RpcReply{200,{}};},options),std::exception);
  release.set_value();active.get();worker_.join();server_.reset();
  EXPECT_NE(0,::access(path_.c_str(),F_OK));
}
TEST_F(RpcTransportTest, SharedDiagnosticsHaveBoundedRedactedRecordsAndLevelCas) {
  start({},{{"XGC2_XRPC_LOG_LEVEL","debug"}});
  EXPECT_EQ(200,exchange("PUT","/entities","{\"privatePassword\":\"never-log-this\"}").status);
  const auto before=json(exchange("GET","/v1/xrpc/policy"));
  EXPECT_EQ("debug",before["fields"]["LOG_LEVEL"]["value"].asString());
  EXPECT_EQ(200,exchange("PUT","/v1/xrpc/log-level","{\"expectedRevision\":1,\"level\":\"warn\"}").status);
  EXPECT_EQ(409,exchange("PUT","/v1/xrpc/log-level","{\"expectedRevision\":1,\"level\":\"trace\"}").status);
  const auto after=json(exchange("GET","/v1/xrpc/policy"));
  EXPECT_EQ(2,after["revision"].asUInt64());EXPECT_EQ("warn",after["fields"]["LOG_LEVEL"]["value"].asString());
  EXPECT_EQ(400,exchange("PUT","/v1/xrpc/log-level","{\"expectedRevision\":2,\"level\":\"warn\",\"format\":\"text\"}").status);
  const auto records=exchange("GET","/v1/xrpc/diagnostics");
  EXPECT_EQ(200,records.status);EXPECT_EQ(std::string::npos,records.body.find("never-log-this"));
  const auto buffered=json(records);EXPECT_LE(buffered["records"].size(),16u);EXPECT_GT(buffered["records"].size(),0u);
  EXPECT_EQ(256,buffered["capacity"].asUInt64());
}
TEST_F(RpcTransportTest, CancelledNativeWorkStillConsumesSharedAdmission) {
  std::promise<void> entered,release;auto unlocked=release.get_future().share();
  start([&,unlocked](const std::string&,const std::string&,const Json::Value&){entered.set_value();unlocked.wait_for(seconds(3));return RpcReply{200,{}};},
        {{"XGC2_XRPC_HOST_MAX_IN_FLIGHT","1"}});
  xgc2::xrpc::HttpClient limited(path_,{},"test-incarnation");
  auto active=std::async(std::launch::async,[&]{
    xgc2::xrpc::HttpRequest request;request.method="PUT";request.target="/entities";request.body="{}";
    EXPECT_THROW(limited.call(request,xgc2::xrpc::Clock::now()+milliseconds(100)),xgc2::xrpc::HttpCallError);
  });
  EXPECT_EQ(std::future_status::ready,entered.get_future().wait_for(seconds(1)));
  active.get();
  // Cancellation is an outcome, not release of an executing native handler.
  EXPECT_EQ(503,exchange("GET","/v1/xrpc/status").status);
  release.set_value();server_->stop();worker_.join();
}
TEST_F(RpcTransportTest, LeaseAlsoFencesNativeOwnerQuiescenceAfterCallsEnd) {
  std::promise<void> entered,release;auto unlocked=release.get_future().share();
  std::atomic<int> native_stops{0};
  start({}, {}, [&,unlocked]{++native_stops;entered.set_value();unlocked.wait_for(seconds(3));});
  server_->stop();
  EXPECT_EQ(std::future_status::ready,entered.get_future().wait_for(seconds(1)));
  RpcOptions options;options.retained_parent_fd=directory_fd_;options.target_id="test-target";options.instance_id="replacement";
  EXPECT_THROW(RpcServer(path_,[](const std::string&,const std::string&,const Json::Value&){return RpcReply{200,{}};},options),std::exception);
  release.set_value();worker_.join();server_.reset();
  EXPECT_EQ(1,native_stops.load());EXPECT_NE(0,::access(path_.c_str(),F_OK));
}
}} // namespace
