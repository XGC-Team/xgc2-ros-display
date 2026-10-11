#include "../src/server/rpc.hpp"
#include <xgc2/xrpc/http.hpp>
#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <future>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <cstdio>
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
  }
  void TearDown() override {
    if (server_) server_->stop();
    if (worker_.joinable()) worker_.join();
    client_.reset(); server_.reset();
    ::unlink((path_ + ".xrpc.lock").c_str()); EXPECT_EQ(0, ::rmdir(directory_.c_str()));
  }
  Json::Value document() const {
    Json::Value result;
    result["service"] = "xgc2.visualization"; result["api_version"] = "1"; result["instance_id"] = "test-incarnation";
    result["ready"] = ready_.load(); result["facts"]["marker"] = 7;
    return result;
  }
  void start(RpcHandler handler = {}, std::function<void()> quiesce_native = {}) {
    if (!handler) handler = [this](const std::string& method, const std::string& path, const Json::Value& body) {
      ++calls_; Json::Value result; result["method"] = method; result["path"] = path; result["body"] = body;
      return RpcReply{path == "/missing" ? 404 : 200, result};
    };
    RpcOptions options; options.instance_id = "test-incarnation";
    server_.reset(new RpcServer(path_, std::move(handler), [this] { return document(); }, std::move(options), std::move(quiesce_native)));
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
  std::string directory_, path_;
  std::atomic<bool> stopping_{false}, ready_{true}; std::atomic<int> calls_{0};
  std::unique_ptr<RpcServer> server_; std::unique_ptr<xgc2::xrpc::HttpClient> client_; std::thread worker_;
};
TEST_F(RpcTransportTest, PreservesJsonAndRoutingAcrossReusedConnection) {
  start(); EXPECT_EQ(path_, server_->socket_path());
  const auto response = exchange("PUT", "/v1/instances/a", "{\"robots\":[],\"id\":17}");
  EXPECT_EQ(200, response.status); EXPECT_EQ("PUT", json(response)["method"].asString()); EXPECT_EQ(17, json(response)["body"]["id"].asInt());
  EXPECT_EQ("/v1/instances/a", json(response)["path"].asString());
  EXPECT_EQ(404, exchange("GET", "/missing").status); EXPECT_EQ(2, calls_.load());
  // Only describe takes a query; the handler never sees one.
  EXPECT_EQ(400, exchange("GET", "/v1/status?verbose=1").status); EXPECT_EQ(2, calls_.load());
}
TEST_F(RpcTransportTest, RejectsAmbiguousJsonBeforeBusinessHandler) {
  start();
  for (const auto* body : {"{\"a\":1,\"a\":2}", "{}{}", "{\"a\":NaN}", "{\"a\":1,}", "/*comment*/{}"})
    EXPECT_EQ(400, exchange("POST", "/entities", body).status) << body;
  EXPECT_EQ(0, calls_.load()); EXPECT_EQ(200, exchange("POST", "/entities", "{}").status);
}
TEST_F(RpcTransportTest, MapsHandlerFailureWithoutPrivateDetails) {
  start([](const std::string&, const std::string&, const Json::Value&) -> RpcReply { throw std::runtime_error("private-password"); });
  const auto response = exchange("GET", "/v1/status"); EXPECT_EQ(500, response.status);
  EXPECT_EQ(std::string::npos, response.body.find("private-password"));
}
TEST_F(RpcTransportTest, BoundsSerializedBusinessResponse) {
  start([](const std::string&, const std::string&, const Json::Value&) { Json::Value result; result["large"] = std::string(1048576, 'x'); return RpcReply{200, result}; });
  const auto response = exchange("GET", "/large"); EXPECT_EQ(500, response.status);
  EXPECT_LT(response.body.size(), 256u); EXPECT_NE(std::string::npos, response.body.find("resource_exhausted"));
}
TEST_F(RpcTransportTest, DiscoversTheProviderAndFencesEveryDomainRequest) {
  start();
  xgc2::xrpc::HttpClient unbound(path_);
  xgc2::xrpc::HttpRequest discovery; discovery.method = "GET"; discovery.target = "/v1/describe";
  const auto response = unbound.call(discovery, xgc2::xrpc::Clock::now() + seconds(2));
  ASSERT_EQ(200, response.status); const auto described = json(response);
  EXPECT_EQ("test-incarnation", described["instance_id"].asString());
  EXPECT_EQ("xgc2.visualization", described["service"].asString());
  EXPECT_TRUE(described["ready"].asBool()); EXPECT_EQ(7, described["facts"]["marker"].asInt());
  discovery.target = "/v1/status";
  EXPECT_EQ(409, unbound.call(discovery, xgc2::xrpc::Clock::now() + seconds(2)).status);
  xgc2::xrpc::HttpClient stale(path_, {}, "old-incarnation");
  EXPECT_THROW(stale.call(discovery, xgc2::xrpc::Clock::now() + seconds(2)), xgc2::xrpc::HttpCallError);
  EXPECT_EQ(0, calls_.load());
  EXPECT_EQ(200, exchange("GET", "/v1/status").status);
}
TEST_F(RpcTransportTest, DescribeRejectsMalformedWaitsAndOtherMethods) {
  start();
  for (const auto* target : {"/v1/describe?wait_ready_ms=", "/v1/describe?wait_ready_ms=abc", "/v1/describe?wait_ready_ms=30001",
                             "/v1/describe?wait_ready_ms=-1", "/v1/describe?wait_ready_ms=05", "/v1/describe?other=1"})
    EXPECT_EQ(400, exchange("GET", target).status) << target;
  EXPECT_EQ(405, exchange("PUT", "/v1/describe").status);
  EXPECT_EQ(200, exchange("GET", "/v1/describe?wait_ready_ms=0").status);
  EXPECT_EQ(0, calls_.load());
}
TEST_F(RpcTransportTest, DescribeWaitsForReadinessWithoutOccupyingTheDomainWorker) {
  ready_ = false; start();
  // Not ready, no wait: answered at once.
  auto now = steady_clock::now();
  auto immediate = json(exchange("GET", "/v1/describe"));
  EXPECT_FALSE(immediate["ready"].asBool()); EXPECT_LT(steady_clock::now() - now, milliseconds(300));
  // The held call returns when readiness arrives, not at the end of its wait.
  now = steady_clock::now();
  auto held = std::async(std::launch::async, [&] {
    xgc2::xrpc::HttpClient waiter(path_, {}, "test-incarnation");
    xgc2::xrpc::HttpRequest request; request.method = "GET"; request.target = "/v1/describe?wait_ready_ms=5000";
    return waiter.call(request, xgc2::xrpc::Clock::now() + seconds(8));
  });
  std::this_thread::sleep_for(milliseconds(150));
  // Domain calls proceed while a describe is held.
  EXPECT_EQ(200, exchange("GET", "/v1/status").status);
  EXPECT_EQ(std::future_status::timeout, held.wait_for(milliseconds(1)));
  ready_ = true;
  ASSERT_EQ(std::future_status::ready, held.wait_for(seconds(2)));
  const auto elapsed = steady_clock::now() - now;
  EXPECT_GE(elapsed, milliseconds(140)); EXPECT_LT(elapsed, milliseconds(1500));
  EXPECT_TRUE(json(held.get())["ready"].asBool());
  // A wait that elapses answers with the current document.
  ready_ = false; now = steady_clock::now();
  const auto expired = json(exchange("GET", "/v1/describe?wait_ready_ms=200"));
  EXPECT_FALSE(expired["ready"].asBool());
  EXPECT_GE(steady_clock::now() - now, milliseconds(190)); EXPECT_LT(steady_clock::now() - now, milliseconds(1500));
}
TEST_F(RpcTransportTest, DescribeWaitIsBoundedByTheCallDeadline) {
  ready_ = false; start();
  xgc2::xrpc::HttpRequest request; request.method = "GET"; request.target = "/v1/describe?wait_ready_ms=30000";
  // The client sets its own deadline; the answer arrives before it, not-ready, rather than as a timeout.
  const auto response = client_->call(request, xgc2::xrpc::Clock::now() + milliseconds(400));
  EXPECT_EQ(200, response.status); EXPECT_FALSE(json(response)["ready"].asBool());
}
TEST_F(RpcTransportTest, StopAnswersHeldDescribeAndEndsTheEndpoint) {
  ready_ = false; start();
  auto held = std::async(std::launch::async, [&] {
    xgc2::xrpc::HttpClient waiter(path_, {}, "test-incarnation");
    xgc2::xrpc::HttpRequest request; request.method = "GET"; request.target = "/v1/describe?wait_ready_ms=20000";
    return waiter.call(request, xgc2::xrpc::Clock::now() + seconds(8));
  });
  std::this_thread::sleep_for(milliseconds(150));
  server_->stop();
  ASSERT_EQ(std::future_status::ready, held.wait_for(seconds(3)));
  EXPECT_EQ(200, held.get().status);
  worker_.join(); server_.reset();
  EXPECT_NE(0, ::access(path_.c_str(), F_OK));
}
TEST_F(RpcTransportTest, DomainWorkDoesNotBlockDescribeAndCancelledQueuedWorkNeverStarts) {
  std::promise<void> entered, release; auto unlocked = release.get_future().share();
  std::atomic<int> mutations{0};
  start([&, unlocked](const std::string&, const std::string&, const Json::Value&) {
    if (++mutations == 1) { entered.set_value(); unlocked.wait_for(seconds(3)); }
    return RpcReply{200, Json::Value(Json::objectValue)};
  });
  auto first = std::async(std::launch::async, [&] { return exchange("PUT", "/entities", "{}"); });
  EXPECT_EQ(std::future_status::ready, entered.get_future().wait_for(seconds(1)));
  xgc2::xrpc::HttpClient queued(path_, {}, "test-incarnation");
  xgc2::xrpc::HttpRequest mutation; mutation.method = "PUT"; mutation.target = "/entities"; mutation.body = "{}";
  EXPECT_THROW(queued.call(mutation, xgc2::xrpc::Clock::now() + milliseconds(80)), xgc2::xrpc::HttpCallError);
  xgc2::xrpc::HttpClient observer(path_, {}, "test-incarnation");
  xgc2::xrpc::HttpRequest observation; observation.method = "GET"; observation.target = "/v1/describe";
  EXPECT_EQ(200, observer.call(observation, xgc2::xrpc::Clock::now() + seconds(1)).status);
  release.set_value(); EXPECT_EQ(200, first.get().status);
  // A subsequent domain request is an ordering barrier on the single worker.
  EXPECT_EQ(200, exchange("GET", "/v1/status").status); EXPECT_EQ(2, mutations.load());
}
TEST_F(RpcTransportTest, StopRetainsLeaseUntilNativeWorkReallyEnds) {
  std::promise<void> entered, release; auto unlocked = release.get_future().share();
  start([&, unlocked](const std::string&, const std::string&, const Json::Value&) { entered.set_value(); unlocked.wait_for(seconds(3)); return RpcReply{200, {}}; });
  auto active = std::async(std::launch::async, [&] { try { exchange("PUT", "/entities", "{}"); } catch (const std::exception&) {} });
  EXPECT_EQ(std::future_status::ready, entered.get_future().wait_for(seconds(1)));
  server_->stop();
  RpcOptions options; options.instance_id = "replacement";
  EXPECT_THROW(RpcServer(path_, [](const std::string&, const std::string&, const Json::Value&) { return RpcReply{200, {}}; },
                         [] { return Json::Value(); }, options), std::exception);
  release.set_value(); active.get(); worker_.join(); server_.reset();
  EXPECT_NE(0, ::access(path_.c_str(), F_OK));
}
TEST_F(RpcTransportTest, LeaseAlsoFencesNativeOwnerQuiescenceAfterCallsEnd) {
  std::promise<void> entered, release; auto unlocked = release.get_future().share();
  std::atomic<int> native_stops{0};
  start({}, [&, unlocked] { ++native_stops; entered.set_value(); unlocked.wait_for(seconds(3)); });
  server_->stop();
  EXPECT_EQ(std::future_status::ready, entered.get_future().wait_for(seconds(1)));
  RpcOptions options; options.instance_id = "replacement";
  EXPECT_THROW(RpcServer(path_, [](const std::string&, const std::string&, const Json::Value&) { return RpcReply{200, {}}; },
                         [] { return Json::Value(); }, options), std::exception);
  release.set_value(); worker_.join(); server_.reset();
  EXPECT_EQ(1, native_stops.load()); EXPECT_NE(0, ::access(path_.c_str(), F_OK));
}
TEST_F(RpcTransportTest, RefusesASecondOwnerOfALiveEndpoint) {
  start();
  RpcOptions options; options.instance_id = "contender";
  EXPECT_THROW(RpcServer(path_, [](const std::string&, const std::string&, const Json::Value&) { return RpcReply{200, {}}; },
                         [] { return Json::Value(); }, options), std::exception);
  EXPECT_EQ(200, exchange("GET", "/v1/status").status);
}
TEST_F(RpcTransportTest, ReclaimsTheSocketAnOwnerLeftBehind) {
  // A killed owner leaves its socket inode; nothing listens on it.
  const int stale = ::socket(AF_UNIX, SOCK_STREAM, 0); ASSERT_GE(stale, 0);
  sockaddr_un address{}; address.sun_family = AF_UNIX; std::snprintf(address.sun_path, sizeof(address.sun_path), "%s", path_.c_str());
  ASSERT_EQ(0, ::bind(stale, reinterpret_cast<sockaddr*>(&address), sizeof(address))); ::close(stale);
  ASSERT_EQ(0, ::access(path_.c_str(), F_OK));
  start();
  EXPECT_EQ(200, exchange("GET", "/v1/status").status);
}
}} // namespace
