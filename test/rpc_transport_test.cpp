#include "../src/server/rpc.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <exception>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace xgc2_ros_visualizer {
namespace {
using Clock = std::chrono::steady_clock;

struct Socket {
  int fd = -1;
  Socket() = default;
  explicit Socket(int value) : fd(value) {}
  Socket(const Socket&) = delete;
  Socket& operator=(const Socket&) = delete;
  Socket(Socket&& other) noexcept : fd(other.fd) { other.fd = -1; }
  Socket& operator=(Socket&& other) noexcept {
    if (fd >= 0) ::close(fd);
    fd = other.fd;
    other.fd = -1;
    return *this;
  }
  ~Socket() { if (fd >= 0) ::close(fd); }
};

Socket connect_to(const std::string& path) {
  Socket socket(::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0));
  if (socket.fd < 0) throw std::runtime_error("test socket failed");
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
  if (::connect(socket.fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
    throw std::runtime_error("test connect failed");
  timeval timeout{3, 0};
  ::setsockopt(socket.fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
  return socket;
}

Socket bind_foreign(const std::string& path) {
  Socket socket(::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0));
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
  if (socket.fd < 0 ||
      ::bind(socket.fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0 ||
      ::listen(socket.fd, 1) != 0)
    throw std::runtime_error("foreign test socket failed");
  return socket;
}

void send_all(int fd, const std::string& data) {
  std::size_t sent = 0;
  while (sent < data.size()) {
    const auto count = ::send(fd, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
    if (count > 0) sent += static_cast<std::size_t>(count);
    else if (count < 0 && errno == EINTR) continue;
    else throw std::runtime_error("test send failed");
  }
}

std::string receive_all(int fd, int timeout_ms = 3000) {
  std::string result;
  const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
  while (Clock::now() < deadline) {
    pollfd descriptor{fd, POLLIN, 0};
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - Clock::now()).count();
    const int count = ::poll(&descriptor, 1, static_cast<int>(left));
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) throw std::runtime_error("test receive deadline");
    char buffer[16384];
    const auto bytes = ::recv(fd, buffer, sizeof(buffer), MSG_DONTWAIT);
    if (bytes > 0) result.append(buffer, static_cast<std::size_t>(bytes));
    else if (bytes == 0 || (bytes < 0 && errno == ECONNRESET)) return result;
    else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
      throw std::runtime_error("test receive failed");
  }
  throw std::runtime_error("test receive deadline");
}

std::string request(const std::string& method, const std::string& path,
                    const std::string& body = {}) {
  return method + " " + path + " HTTP/1.1\r\nHost: localhost\r\n"
      "Content-Type: application/json\r\nContent-Length: " +
      std::to_string(body.size()) + "\r\n\r\n" + body;
}

struct HttpReply {
  int status = 0;
  Json::Value body;
  std::string bytes;
};

HttpReply decode(const std::string& bytes) {
  HttpReply reply;
  reply.bytes = bytes;
  if (std::sscanf(bytes.c_str(), "HTTP/1.1 %d", &reply.status) != 1)
    throw std::runtime_error("invalid HTTP reply");
  const auto split = bytes.find("\r\n\r\n");
  const auto length = bytes.find("Content-Length: ");
  if (split == std::string::npos || length == std::string::npos ||
      bytes.find("Connection: close\r\n") == std::string::npos)
    throw std::runtime_error("incomplete HTTP reply");
  const auto body = bytes.substr(split + 4);
  if (std::stoul(bytes.substr(length + 16)) != body.size())
    throw std::runtime_error("incorrect HTTP reply length");
  if (!body.empty()) {
    Json::CharReaderBuilder builder;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    std::string errors;
    if (!reader->parse(body.data(), body.data() + body.size(), &reply.body, &errors))
      throw std::runtime_error("invalid JSON reply");
  }
  return reply;
}

class RpcTransportTest : public ::testing::Test {
 protected:
  void SetUp() override {
    char pattern[] = "/tmp/xgc2-rpc-test-XXXXXX";
    const char* directory = ::mkdtemp(pattern);
    ASSERT_NE(nullptr, directory);
    directory_ = directory;
    path_ = directory_ + "/control.sock";
  }

  void TearDown() override {
    stop_and_join();
    ::unlink(path_.c_str());
    ::unlink((directory_ + "/saved.sock").c_str());
    ::unlink((directory_ + "/file").c_str());
    ::rmdir(directory_.c_str());
  }

  void start(RpcHandler handler = {}) {
    if (!handler) handler = [this](const std::string& method, const std::string& path,
                                   const Json::Value& body) {
      ++calls_;
      Json::Value result(Json::objectValue);
      result["ok"] = true;
      result["method"] = method;
      result["path"] = path;
      result["body"] = body;
      return RpcReply{200, result};
    };
    server_.reset(new RpcServer(path_, std::move(handler)));
    worker_ = std::thread([this] {
      try { server_->run(stopping_); }
      catch (...) { failure_ = std::current_exception(); }
    });
  }

  void stop_and_join() {
    if (server_) server_->stop();
    if (worker_.joinable()) worker_.join();
    server_.reset();
    if (failure_) {
      try { std::rethrow_exception(failure_); }
      catch (const std::exception& error) { ADD_FAILURE() << error.what(); }
      catch (...) { ADD_FAILURE() << "unknown server failure"; }
      failure_ = {};
    }
  }

  HttpReply exchange(const std::string& raw) {
    auto socket = connect_to(path_);
    send_all(socket.fd, raw);
    return decode(receive_all(socket.fd));
  }

  void healthy() { EXPECT_EQ(200, exchange(request("GET", "/health")).status); }

  std::string directory_;
  std::string path_;
  std::atomic<bool> stopping_{false};
  std::atomic<int> calls_{0};
  std::unique_ptr<RpcServer> server_;
  std::thread worker_;
  std::exception_ptr failure_;
};

TEST_F(RpcTransportTest, RoutesJsonAndEmptyBodyOnOneControlThread) {
  const auto caller = std::this_thread::get_id();
  std::atomic<bool> ran_on_caller{false};
  start([&](const std::string& method, const std::string& path, const Json::Value& body) {
    if (std::this_thread::get_id() == caller) ran_on_caller.store(true);
    Json::Value result(Json::objectValue);
    result["ok"] = true;
    result["method"] = method;
    result["path"] = path;
    result["body"] = body;
    return RpcReply{path == "/missing" ? 404 : 200, result};
  });
  struct stat bound{};
  ASSERT_EQ(0, ::lstat(path_.c_str(), &bound));
  EXPECT_TRUE(S_ISSOCK(bound.st_mode));
  EXPECT_EQ(0600u, bound.st_mode & 0777u);
  EXPECT_EQ(path_, server_->socket_path());
  auto reply = exchange(request("GET", "/health"));
  EXPECT_EQ(200, reply.status);
  EXPECT_TRUE(reply.body["body"].isNull());
  reply = exchange(request("PUT", "/v1/instances/a", "{\"robots\":[],\"id\":17}"));
  EXPECT_EQ("PUT", reply.body["method"].asString());
  EXPECT_EQ("/v1/instances/a", reply.body["path"].asString());
  EXPECT_EQ(17, reply.body["body"]["id"].asInt());
  EXPECT_EQ(404, exchange(request("GET", "/missing")).status);
  EXPECT_EQ(200, exchange(request("DELETE", "/v1/instances/a")).status);
  EXPECT_FALSE(ran_on_caller.load());
}

TEST_F(RpcTransportTest, PartialHeadersAndBodyDoNotBlockAnotherClient) {
  start();
  auto slow = connect_to(path_);
  send_all(slow.fd, "PUT /v1/instances/a HTTP/1.1\r\nContent-Len");
  const auto began = Clock::now();
  healthy();
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - began).count(), 1000);
  send_all(slow.fd, "gth: 7\r\n\r\n{\"a\":");
  healthy();
  EXPECT_EQ(2, calls_.load());
  send_all(slow.fd, "1}");
  auto reply = decode(receive_all(slow.fd));
  EXPECT_EQ(200, reply.status);
  EXPECT_EQ(1, reply.body["body"]["a"].asInt());
  EXPECT_EQ(3, calls_.load());
}

TEST_F(RpcTransportTest, RejectsMalformedJsonWithoutKillingServer) {
  start();
  const std::vector<std::string> invalid = {
      "{", "{\"a\":1,\"a\":2}", "{\"a\":1,}", "/*comment*/{}", "{} {}",
      "{\"a\":NaN}", std::string(80, '[') + "0" + std::string(80, ']')};
  for (const auto& body : invalid) {
    SCOPED_TRACE(body.substr(0, 80));
    const auto before = calls_.load();
    auto reply = exchange(request("PUT", "/v1/rates", body));
    EXPECT_EQ(400, reply.status);
    EXPECT_FALSE(reply.body["ok"].asBool());
    EXPECT_EQ(before, calls_.load());
    healthy();
  }
}

TEST_F(RpcTransportTest, RejectsAmbiguousFramingAndInvalidHeaders) {
  start();
  const std::vector<std::string> invalid = {
      "PUT /a HTTP/1.1\r\nContent-Length: 2\r\nContent-Length: 2\r\n\r\n{}",
      "PUT /a HTTP/1.1\r\nContent-Length: +2\r\n\r\n{}",
      "PUT /a HTTP/1.1\r\nTransfer-Encoding: chunked\r\n\r\n0\r\n\r\n",
      "PUT /a HTTP/1.1\r\nExpect: 100-continue\r\n\r\n",
      "GET /a HTTP/1.1\r\n folded: header\r\n\r\n",
      "GET /a HTTP/1.1\r\nNoColon\r\n\r\n",
      "GET /a HTTP/2\r\n\r\n",
      "GET /a extra HTTP/1.1\r\n\r\n",
      "GET /a#fragment HTTP/1.1\r\n\r\n",
      "PUT /a HTTP/1.1\r\nContent-Type: application/json\r\nContent-Type: application/json\r\nContent-Length: 2\r\n\r\n{}",
      std::string("G\0T /a HTTP/1.1\r\n\r\n", 19),
      request("PUT", "/a", "{}") + "GET /b HTTP/1.1\r\n\r\n"};
  for (const auto& raw : invalid) {
    SCOPED_TRACE(raw.substr(0, 80));
    const auto before = calls_.load();
    EXPECT_EQ(400, exchange(raw).status);
    EXPECT_EQ(before, calls_.load());
  }
  EXPECT_EQ(415, exchange("PUT /a HTTP/1.1\r\nContent-Type: text/plain\r\nContent-Length: 2\r\n\r\n{}").status);
  healthy();
}

TEST_F(RpcTransportTest, EnforcesHeaderBodyAndSerializedReplyBounds) {
  start([](const std::string&, const std::string& path, const Json::Value& body) {
    if (path == "/large") return RpcReply{200, Json::Value(std::string(1024 * 1024, 'x'))};
    return RpcReply{200, body};
  });
  EXPECT_EQ(431, exchange("GET /a HTTP/1.1\r\nX: " + std::string(16384, 'x') + "\r\n\r\n").status);
  EXPECT_EQ(413, exchange("PUT /a HTTP/1.1\r\nContent-Length: 1048577\r\n\r\n").status);
  EXPECT_EQ(413, exchange("PUT /a HTTP/1.1\r\nContent-Length: 9999999999999999999999999999999999\r\n\r\n").status);
  const std::string exact = "\"" + std::string(1024 * 1024 - 2, 'x') + "\"";
  const auto reply = exchange(request("PUT", "/echo", exact));
  EXPECT_EQ(200, reply.status);
  EXPECT_EQ(1024u * 1024u - 2u, reply.body.asString().size());
  auto overflow = exchange(request("GET", "/large"));
  EXPECT_EQ(500, overflow.status);
  EXPECT_EQ("response exceeds limit", overflow.body["error"].asString());
  EXPECT_LT(overflow.bytes.size(), 512u);
  healthy();
}

TEST_F(RpcTransportTest, HandlerFailureAndInvalidStatusAreBoundedJsonErrors) {
  start([](const std::string&, const std::string& path, const Json::Value&) -> RpcReply {
    if (path == "/throw") throw std::runtime_error("private internal detail");
    return RpcReply{path == "/invalid" ? 12345 : 409, Json::Value(Json::objectValue)};
  });
  auto reply = exchange(request("GET", "/throw"));
  EXPECT_EQ(500, reply.status);
  EXPECT_EQ(std::string::npos, reply.bytes.find("private internal detail"));
  EXPECT_EQ(500, exchange(request("GET", "/invalid")).status);
  EXPECT_EQ(409, exchange(request("GET", "/conflict")).status);
}

TEST_F(RpcTransportTest, ClientLimitAndAbsoluteDeadlineBoundSlowClients) {
  start();
  std::vector<Socket> slow;
  const auto began = Clock::now();
  for (int i = 0; i < 32; ++i) {
    slow.push_back(connect_to(path_));
    send_all(slow.back().fd, "GET /health HTTP/1.1\r\n");
  }
  // Admission is asynchronous; allow the fixed loop to accept the 32 sockets.
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  EXPECT_EQ(503, exchange(request("GET", "/health")).status);
  EXPECT_EQ(0, calls_.load());
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  // Progress does not extend the original five-second connection lifetime.
  send_all(slow.front().fd, "X-Progress: yes\r\n");
  EXPECT_TRUE(receive_all(slow.front().fd, 4000).empty());
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - began).count();
  EXPECT_GE(elapsed, 4900);
  EXPECT_LT(elapsed, 5700);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  healthy();
}

TEST_F(RpcTransportTest, PeerEofAndDisconnectDoNotKillControlLoop) {
  start();
  auto truncated = connect_to(path_);
  send_all(truncated.fd, "PUT /a HTTP/1.1\r\nContent-Length: 10\r\n\r\n{}");
  ASSERT_EQ(0, ::shutdown(truncated.fd, SHUT_WR));
  EXPECT_EQ(400, decode(receive_all(truncated.fd)).status);
  {
    auto disconnected = connect_to(path_);
    send_all(disconnected.fd, request("GET", "/health"));
  }
  healthy();
}

TEST_F(RpcTransportTest, SlowResponseReaderCannotBlockOtherRequestsOrShutdown) {
  start([](const std::string&, const std::string& path, const Json::Value&) {
    if (path == "/large")
      return RpcReply{200, Json::Value(std::string(1024 * 1024 - 2, 'x'))};
    Json::Value value(Json::objectValue);
    value["ok"] = true;
    return RpcReply{200, value};
  });
  auto stalled = connect_to(path_);
  const int receive_buffer = 1024;
  ASSERT_EQ(0, ::setsockopt(stalled.fd, SOL_SOCKET, SO_RCVBUF,
      &receive_buffer, sizeof(receive_buffer)));
  send_all(stalled.fd, request("GET", "/large"));
  pollfd waiting{stalled.fd, POLLIN, 0};
  ASSERT_GT(::poll(&waiting, 1, 1000), 0);
  const auto began = Clock::now();
  healthy();
  stop_and_join();
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - began).count(), 500);
  EXPECT_FALSE(receive_all(stalled.fd).empty());
}

TEST_F(RpcTransportTest, StopIsIdempotentClosesClientsAndUnlinksOnlyOwnSocket) {
  start();
  auto unfinished = connect_to(path_);
  send_all(unfinished.fd, "GET /health HTTP/1.1\r\n");
  const auto began = Clock::now();
  server_->stop();
  server_->stop();
  worker_.join();
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - began).count(), 500);
  EXPECT_TRUE(receive_all(unfinished.fd).empty());
  struct stat removed{};
  EXPECT_EQ(-1, ::lstat(path_.c_str(), &removed));
  EXPECT_EQ(ENOENT, errno);
  EXPECT_THROW(server_->run(stopping_), std::logic_error);
}

TEST_F(RpcTransportTest, ExternalStoppingFlagAlsoCleansUp) {
  start();
  healthy();
  stopping_.store(true);
  worker_.join();
  struct stat removed{};
  EXPECT_EQ(-1, ::lstat(path_.c_str(), &removed));
  EXPECT_EQ(ENOENT, errno);
}

TEST_F(RpcTransportTest, ExistingFileSocketAndSymlinkAreNeverUnlinked) {
  RpcHandler handler = [](const std::string&, const std::string&, const Json::Value&) {
    return RpcReply{200, Json::Value()};
  };
  {
    std::ofstream file(path_);
    file << "foreign data";
  }
  struct stat original{};
  ASSERT_EQ(0, ::lstat(path_.c_str(), &original));
  EXPECT_THROW(RpcServer server(path_, handler), std::runtime_error);
  struct stat retained{};
  ASSERT_EQ(0, ::lstat(path_.c_str(), &retained));
  EXPECT_EQ(original.st_ino, retained.st_ino);
  ::unlink(path_.c_str());
  auto foreign = bind_foreign(path_);
  ASSERT_EQ(0, ::lstat(path_.c_str(), &original));
  EXPECT_THROW(RpcServer server(path_, handler), std::runtime_error);
  ASSERT_EQ(0, ::lstat(path_.c_str(), &retained));
  EXPECT_EQ(original.st_ino, retained.st_ino);
  ::unlink(path_.c_str());
  ASSERT_EQ(0, ::symlink("missing", path_.c_str()));
  EXPECT_THROW(RpcServer server(path_, handler), std::runtime_error);
  ASSERT_EQ(0, ::lstat(path_.c_str(), &retained));
  EXPECT_TRUE(S_ISLNK(retained.st_mode));
}

TEST_F(RpcTransportTest, PathReplacementDoesNotLetShutdownRemoveForeignSocket) {
  start();
  healthy();
  ASSERT_EQ(0, ::rename(path_.c_str(), (directory_ + "/saved.sock").c_str()));
  auto foreign = bind_foreign(path_);
  struct stat expected{};
  ASSERT_EQ(0, ::lstat(path_.c_str(), &expected));
  stop_and_join();
  struct stat retained{};
  ASSERT_EQ(0, ::lstat(path_.c_str(), &retained));
  EXPECT_EQ(expected.st_ino, retained.st_ino);
  auto probe = connect_to(path_);
  EXPECT_GE(probe.fd, 0);
}

TEST_F(RpcTransportTest, DestructorBeforeRunCleansItsSocketAndRejectsBadPaths) {
  RpcHandler handler = [](const std::string&, const std::string&, const Json::Value&) {
    return RpcReply{200, Json::Value()};
  };
  {
    RpcServer server(path_, handler);
    server.stop();
    server.stop();
  }
  struct stat removed{};
  EXPECT_EQ(-1, ::lstat(path_.c_str(), &removed));
  EXPECT_THROW(RpcServer server("relative.sock", handler), std::invalid_argument);
  EXPECT_THROW(RpcServer server("/" + std::string(200, 'x'), handler), std::invalid_argument);
  EXPECT_THROW(RpcServer server(std::string("/tmp/a\0b", 8), handler), std::invalid_argument);
  EXPECT_THROW(RpcServer server(path_, RpcHandler{}), std::invalid_argument);
}

}  // namespace
}  // namespace xgc2_ros_visualizer
