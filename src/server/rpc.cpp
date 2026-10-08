#include "rpc.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <streambuf>
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
constexpr std::size_t kMaxClients = 32;
constexpr std::size_t kMaxHeaderBytes = 16 * 1024;
constexpr std::size_t kMaxBodyBytes = 1024 * 1024;
constexpr std::size_t kIoBudget = 64 * 1024;
constexpr int kPollMs = 100;
constexpr int kClientLifetimeMs = 5000;

Json::Value error_body(const char* message) {
  Json::Value value(Json::objectValue);
  value["ok"] = false;
  value["error"] = message;
  return value;
}

bool token_char(unsigned char c) {
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
         (c >= '0' && c <= '9') || (c != 0 && std::strchr("!#$%&'*+-.^_`|~", c));
}

std::string lower_ascii(std::string value) {
  for (char& c : value) {
    if (c >= 'A' && c <= 'Z') c = static_cast<char>(c + ('a' - 'A'));
  }
  return value;
}

std::string trim_ows(const std::string& value) {
  const auto first = value.find_first_not_of(" \t");
  if (first == std::string::npos) return {};
  const auto last = value.find_last_not_of(" \t");
  return value.substr(first, last - first + 1);
}

const char* reason(int status) {
  switch (status) {
    case 200: return "OK";
    case 201: return "Created";
    case 202: return "Accepted";
    case 204: return "No Content";
    case 304: return "Not Modified";
    case 400: return "Bad Request";
    case 404: return "Not Found";
    case 405: return "Method Not Allowed";
    case 409: return "Conflict";
    case 413: return "Payload Too Large";
    case 415: return "Unsupported Media Type";
    case 422: return "Unprocessable Entity";
    case 431: return "Request Header Fields Too Large";
    case 500: return "Internal Server Error";
    case 503: return "Service Unavailable";
    default: return "Response";
  }
}

// JsonCpp writes directly into a bounded stream instead of first allocating an
// unbounded serialized response string supplied by the domain handler.
class JsonBuffer : public std::streambuf {
 public:
  std::string value;

 protected:
  std::streamsize xsputn(const char* data, std::streamsize length) override {
    const auto available = kMaxBodyBytes - value.size();
    const auto count = std::min<std::size_t>(available,
        static_cast<std::size_t>(length));
    value.append(data, count);
    return static_cast<std::streamsize>(count);
  }
  int_type overflow(int_type c) override {
    if (traits_type::eq_int_type(c, traits_type::eof()))
      return traits_type::not_eof(c);
    if (value.size() == kMaxBodyBytes) return traits_type::eof();
    value.push_back(traits_type::to_char_type(c));
    return c;
  }
};

std::string response(RpcReply reply) {
  if (reply.status < 200 || reply.status > 599)
    reply = {500, error_body("invalid response status")};
  JsonBuffer buffer;
  if (reply.status != 204 && reply.status != 304) {
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    std::ostream output(&buffer);
    writer->write(reply.body, &output);
    if (!output.good()) {
      reply.status = 500;
      buffer.value = "{\"ok\":false,\"error\":\"response exceeds limit\"}";
    }
  }
  return "HTTP/1.1 " + std::to_string(reply.status) + " " + reason(reply.status) +
      "\r\nContent-Type: application/json\r\nConnection: close\r\nContent-Length: " +
      std::to_string(buffer.value.size()) + "\r\n\r\n" + buffer.value;
}
}  // namespace

class RpcServer::Impl {
 public:
  struct Client {
    int fd;
    Clock::time_point deadline;
    std::string input;
    std::string output;
    std::string method;
    std::string path;
    std::size_t header_bytes = 0;
    std::size_t body_bytes = 0;
    std::size_t sent = 0;
    bool headers_ready = false;
    bool responding = false;
  };

  Impl(std::string path, RpcHandler handler)
      : path_(std::move(path)), handler_(std::move(handler)) {
    if (!handler_) throw std::invalid_argument("RPC handler is required");
    if (path_.empty() || path_[0] != '/' ||
        path_.size() >= sizeof(sockaddr_un::sun_path) ||
        path_.find('\0') != std::string::npos)
      throw std::invalid_argument("RPC socket must be a short absolute path");
    struct stat existing{};
    if (::lstat(path_.c_str(), &existing) == 0)
      throw std::runtime_error("RPC socket path already exists");
    if (errno != ENOENT) throw std::runtime_error("RPC socket path is unavailable");
    listener_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (listener_ < 0) throw std::runtime_error("RPC socket creation failed");
    try {
      sockaddr_un address{};
      address.sun_family = AF_UNIX;
      std::memcpy(address.sun_path, path_.c_str(), path_.size() + 1);
      if (::bind(listener_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
        throw std::runtime_error("RPC socket bind failed; path must be absent");
      struct stat bound{};
      if (::lstat(path_.c_str(), &bound) != 0 || !S_ISSOCK(bound.st_mode))
        throw std::runtime_error("RPC socket identity is unavailable");
      device_ = bound.st_dev;
      inode_ = bound.st_ino;
      owns_path_ = true;
      // Listen only after restricting permissions, so there is no connectable
      // interval with permissions inherited from the process umask.
      if (::chmod(path_.c_str(), 0600) != 0 || !owns_current_path())
        throw std::runtime_error("RPC socket permissions or identity changed");
      if (::listen(listener_, static_cast<int>(kMaxClients)) != 0)
        throw std::runtime_error("RPC socket listen failed");
      clients_.reserve(kMaxClients);
    } catch (...) {
      cleanup();
      throw;
    }
  }

  ~Impl() { cleanup(); }

  void run(const std::atomic<bool>& stopping) {
    if (run_started_) throw std::logic_error("RPC control loop may run only once");
    run_started_ = true;
    try {
      while (!stopping.load(std::memory_order_relaxed) &&
             !stop_.load(std::memory_order_relaxed)) {
        const auto now = Clock::now();
        for (auto& client : clients_) {
          if (now >= client.deadline) close_client(client);
        }
        discard_closed();
        std::vector<pollfd> descriptors;
        descriptors.reserve(kMaxClients + 1);
        descriptors.push_back({listener_, POLLIN, 0});
        for (const auto& client : clients_)
          descriptors.push_back({client.fd,
              static_cast<short>(client.responding ? POLLOUT : POLLIN), 0});
        const int count = ::poll(descriptors.data(), descriptors.size(), kPollMs);
        if (count < 0) {
          if (errno == EINTR) continue;
          throw std::runtime_error("RPC poll failed");
        }
        if (stopping.load(std::memory_order_relaxed) ||
            stop_.load(std::memory_order_relaxed)) break;
        if (descriptors[0].revents & (POLLERR | POLLHUP | POLLNVAL))
          throw std::runtime_error("RPC listener failed");
        // Indexes still refer to the old clients if accept appends/reallocates.
        if (descriptors[0].revents & POLLIN) accept_clients();
        for (std::size_t i = 1; i < descriptors.size(); ++i) {
          if (stopping.load(std::memory_order_relaxed) ||
              stop_.load(std::memory_order_relaxed)) break;
          auto& client = clients_[i - 1];
          const auto events = descriptors[i].revents;
          if (events & (POLLERR | POLLNVAL)) {
            close_client(client);
            continue;
          }
          if (Clock::now() >= client.deadline) {
            close_client(client);
            continue;
          }
          if (!client.responding && (events & (POLLIN | POLLHUP))) read_client(client);
          if (client.fd >= 0 && client.responding &&
              (events & (POLLIN | POLLOUT | POLLHUP))) write_client(client);
        }
        discard_closed();
      }
    } catch (...) {
      cleanup();
      throw;
    }
    cleanup();
  }

  void stop() noexcept { stop_.store(true, std::memory_order_relaxed); }
  const std::string& path() const noexcept { return path_; }

 private:
  bool owns_current_path() const noexcept {
    struct stat current{};
    return owns_path_ && ::lstat(path_.c_str(), &current) == 0 &&
        S_ISSOCK(current.st_mode) && current.st_dev == device_ && current.st_ino == inode_;
  }

  void cleanup() noexcept {
    for (auto& client : clients_) close_client(client);
    clients_.clear();
    if (listener_ >= 0) {
      ::close(listener_);
      listener_ = -1;
    }
    if (owns_current_path()) ::unlink(path_.c_str());
    owns_path_ = false;
  }

  static void close_client(Client& client) noexcept {
    if (client.fd >= 0) ::close(client.fd);
    client.fd = -1;
  }

  void discard_closed() {
    clients_.erase(std::remove_if(clients_.begin(), clients_.end(),
        [](const Client& client) { return client.fd < 0; }), clients_.end());
  }

  void accept_clients() {
    for (int i = 0; i < 8; ++i) {
      const int fd = ::accept4(listener_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
      if (fd < 0) {
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) return;
        throw std::runtime_error("RPC accept failed");
      }
      if (clients_.size() == kMaxClients) {
        const std::string denied = response({503, error_body("client limit reached")});
        ::send(fd, denied.data(), denied.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
        ::close(fd);
        continue;
      }
      Client client{};
      client.fd = fd;
      client.deadline = Clock::now() + std::chrono::milliseconds(kClientLifetimeMs);
      clients_.push_back(std::move(client));
    }
  }

  static void reply(Client& client, int status, const char* error) {
    client.output = response({status, error_body(error)});
    client.input.clear();
    client.responding = true;
  }

  bool parse_headers(Client& client) {
    const auto end = client.input.find("\r\n\r\n");
    if (end == std::string::npos) {
      if (client.input.size() >= kMaxHeaderBytes)
        reply(client, 431, "headers exceed limit");
      return false;
    }
    client.header_bytes = end + 4;
    if (client.header_bytes > kMaxHeaderBytes) {
      reply(client, 431, "headers exceed limit");
      return false;
    }
    const auto first_line = client.input.find("\r\n");
    const std::string request = client.input.substr(0, first_line);
    const auto first_space = request.find(' ');
    const auto last_space = request.rfind(' ');
    if (first_space == std::string::npos || first_space == last_space ||
        first_space == 0 || last_space <= first_space + 1 ||
        request.find(' ', first_space + 1) != last_space ||
        (request.substr(last_space + 1) != "HTTP/1.1" &&
         request.substr(last_space + 1) != "HTTP/1.0")) {
      reply(client, 400, "invalid request line");
      return false;
    }
    client.method = request.substr(0, first_space);
    client.path = request.substr(first_space + 1, last_space - first_space - 1);
    if (!std::all_of(client.method.begin(), client.method.end(), token_char) ||
        client.path[0] != '/' ||
        !std::all_of(client.path.begin(), client.path.end(),
            [](unsigned char c) { return c > 32 && c < 127 && c != '#'; })) {
      reply(client, 400, "invalid method or path");
      return false;
    }
    bool length_seen = false;
    std::string content_type;
    bool content_type_seen = false;
    std::size_t position = first_line + 2;
    while (position < end) {
      const auto line_end = client.input.find("\r\n", position);
      const std::string line = client.input.substr(position, line_end - position);
      const auto colon = line.find(':');
      if (colon == std::string::npos || colon == 0 ||
          !std::all_of(line.begin(), line.begin() + colon, token_char) ||
          !std::all_of(line.begin() + colon + 1, line.end(),
              [](unsigned char c) { return c == '\t' || (c >= 32 && c != 127); })) {
        reply(client, 400, "invalid header");
        return false;
      }
      const auto name = lower_ascii(line.substr(0, colon));
      const auto value = trim_ows(line.substr(colon + 1));
      if (name == "transfer-encoding" || name == "expect") {
        reply(client, 400, "unsupported request framing");
        return false;
      }
      if (name == "content-length") {
        if (length_seen || value.empty() ||
            !std::all_of(value.begin(), value.end(),
                [](unsigned char c) { return c >= '0' && c <= '9'; })) {
          reply(client, 400, "invalid content length");
          return false;
        }
        length_seen = true;
        for (const char digit : value) {
          const auto next = static_cast<std::size_t>(digit - '0');
          if (client.body_bytes > (kMaxBodyBytes - next) / 10) {
            reply(client, 413, "body exceeds limit");
            return false;
          }
          client.body_bytes = client.body_bytes * 10 + next;
        }
      }
      if (name == "content-type") {
        if (content_type_seen) {
          reply(client, 400, "duplicate content type");
          return false;
        }
        content_type_seen = true;
        content_type = lower_ascii(trim_ows(value.substr(0, value.find(';'))));
      }
      position = line_end + 2;
    }
    if (client.body_bytes && content_type_seen && content_type != "application/json") {
      reply(client, 415, "body must be application/json");
      return false;
    }
    client.headers_ready = true;
    return true;
  }

  void process_input(Client& client) {
    if (!client.headers_ready && !parse_headers(client)) return;
    const auto expected = client.header_bytes + client.body_bytes;
    if (client.input.size() < expected) return;
    if (client.input.size() != expected) {
      reply(client, 400, "one framed request is required");
      return;
    }
    Json::Value body;
    if (client.body_bytes != 0) {
      Json::CharReaderBuilder builder;
      builder["allowComments"] = false;
      builder["collectComments"] = false;
      builder["allowTrailingCommas"] = false;
      builder["strictRoot"] = false;
      builder["failIfExtra"] = true;
      builder["rejectDupKeys"] = true;
      builder["allowSpecialFloats"] = false;
      builder["stackLimit"] = 64;
      std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
      std::string errors;
      const char* begin = client.input.data() + client.header_bytes;
      bool parsed = false;
      try {
        parsed = reader->parse(begin, begin + client.body_bytes, &body, &errors);
      } catch (...) {
        // JsonCpp also throws when its nesting limit is exceeded. A malformed
        // client must not terminate the shared server/control loop.
      }
      if (!parsed) {
        reply(client, 400, "invalid JSON body");
        return;
      }
    }
    try {
      client.output = response(handler_(client.method, client.path, body));
    } catch (...) {
      client.output = response({500, error_body("request handler failed")});
    }
    client.input.clear();
    client.responding = true;
  }

  void read_client(Client& client) {
    char buffer[16 * 1024];
    std::size_t received = 0;
    while (!client.responding && received < kIoBudget) {
      const auto count = ::recv(client.fd, buffer,
          std::min(sizeof(buffer), kIoBudget - received), MSG_DONTWAIT);
      if (count > 0) {
        received += static_cast<std::size_t>(count);
        client.input.append(buffer, static_cast<std::size_t>(count));
        process_input(client);
      } else if (count == 0) {
        reply(client, 400, "incomplete request");
      } else {
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) return;
        close_client(client);
        return;
      }
    }
  }

  static void write_client(Client& client) {
    std::size_t written = 0;
    while (client.sent < client.output.size() && written < kIoBudget) {
      const auto count = ::send(client.fd, client.output.data() + client.sent,
          std::min(client.output.size() - client.sent, kIoBudget - written), MSG_NOSIGNAL);
      if (count > 0) {
        client.sent += static_cast<std::size_t>(count);
        written += static_cast<std::size_t>(count);
      } else if (count < 0 && errno == EINTR) {
        continue;
      } else if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        return;
      } else {
        close_client(client);
        return;
      }
    }
    if (client.sent == client.output.size()) close_client(client);
  }

  std::string path_;
  RpcHandler handler_;
  int listener_ = -1;
  dev_t device_ = 0;
  ino_t inode_ = 0;
  bool owns_path_ = false;
  bool run_started_ = false;
  std::atomic<bool> stop_{false};
  std::vector<Client> clients_;
};

RpcServer::RpcServer(std::string socket_path, RpcHandler handler)
    : impl_(new Impl(std::move(socket_path), std::move(handler))) {}
RpcServer::~RpcServer() = default;
void RpcServer::run(const std::atomic<bool>& stopping) { impl_->run(stopping); }
void RpcServer::stop() noexcept { impl_->stop(); }
const std::string& RpcServer::socket_path() const noexcept { return impl_->path(); }

}  // namespace xgc2_ros_visualizer
