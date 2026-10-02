#include <xgc2_ros_display_relays/display_relays.hpp>
#include <xgc2_ros_display_relays/declared_wire.hpp>
#include <xgc2_ros_display_relays/rate_gate.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <boost/make_shared.hpp>
#include <boost/weak_ptr.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <atomic>
#include <exception>
#include <mutex>
#include <thread>
#include <stdexcept>
#include <utility>
namespace xgc2_ros_display_relays {
class DisplayRelays::Impl {
 public:
  explicit Impl(std::vector<RelaySpec> config) : specs(std::move(config)) { validateRelaySpecs(specs); }
  struct Relay {
    virtual ~Relay() = default;
    virtual void close() noexcept = 0;
  };
  template<class Message> struct TypedRelay final : Relay {
    using Wire = DeclaredWire<Message>;
    Impl& owner;
    ros::NodeHandle node;
    RelaySpec spec;
    ros::Publisher publisher;
    ros::Subscriber subscriber;
    detail::RateGate rate;
    std::uint64_t generation = 0;
    TypedRelay(Impl& impl, const ros::NodeHandle& nh, const RelaySpec& configured)
        : owner(impl), node(nh), spec(configured), rate(configured.max_rate_hz) {}
    static boost::shared_ptr<TypedRelay> create(Impl& owner, const ros::NodeHandle& node, const RelaySpec& spec) {
      auto relay = boost::make_shared<TypedRelay>(owner, node, spec);
      boost::weak_ptr<TypedRelay> weak(relay);
      ros::SubscriberStatusCallback status = [weak](const ros::SingleSubscriberPublisher&) {
        if (auto self = weak.lock()) {
          try { self->reconcile(); } catch (...) { self->owner.fail(std::current_exception()); }
        }
      };
      auto options = ros::AdvertiseOptions::create<Wire>(spec.topic, 1, status, status, relay, &owner.queue);
      options.latch = false;
      options.has_header = false; // Preserve even the source's serialized seq.
      relay->publisher = relay->node.advertise(options);
      if (!relay->publisher) throw std::runtime_error("cannot advertise display copy: " + spec.topic);
      return relay;
    }
    void reconcile() {
      if (owner.stopping.load()) return;
      const bool watched = publisher.getNumSubscribers() != 0;
      if (watched && !subscriber) {
        rate.reset();
        const auto current_generation = ++generation;
        boost::weak_ptr<TypedRelay> weak(this_shared);
        boost::function<void(const typename Wire::ConstPtr&)> callback = [weak, current_generation](const typename Wire::ConstPtr& message) {
          if (auto self = weak.lock()) {
            try {
              if (self->owner.stopping.load() || current_generation != self->generation || self->publisher.getNumSubscribers() == 0) return;
              if (self->rate.admit(detail::RateGate::Clock::now())) self->publisher.publish(message);
            } catch (...) { self->owner.fail(std::current_exception()); }
          }
        };
        subscriber = node.subscribe<Wire>(spec.source, 1, callback, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
        if (!subscriber) throw std::runtime_error("cannot subscribe display source: " + spec.source);
      } else if (!watched && subscriber) {
        ++generation;
        subscriber.shutdown();
      }
    }
    // Weak only: ROS callbacks must not create an owning cycle.
    boost::weak_ptr<TypedRelay> this_shared;
    void close() noexcept override {
      ++generation;
      subscriber.shutdown(); publisher.shutdown();
    }
  };
  template<class Message> void add(const ros::NodeHandle& node, const RelaySpec& spec) {
    auto relay = TypedRelay<Message>::create(*this, node, spec);
    relay->this_shared = relay;
    relays.push_back(relay);
  }
  void fail(std::exception_ptr error) noexcept {
    { std::lock_guard<std::mutex> guard(failure_mutex); if (!failure) failure = error; }
    stopping.store(true);
  }
  void start(const ros::NodeHandle& owner_node) {
    std::lock_guard<std::mutex> guard(control_mutex);
    if (started || stopping.load()) throw std::logic_error("display relay owner can only start once and cannot restart after Stop");
    started = true;
    if (specs.empty()) return;
    ros::NodeHandle node(owner_node); node.setCallbackQueue(&queue);
    try {
      for (const auto& spec : specs) {
        if (node.resolveName(spec.source) != spec.source || node.resolveName(spec.topic) != spec.topic)
          throw std::invalid_argument("ROS remapping changed a canonical display relay topic");
        if (spec.message_type == "sensor_msgs/PointCloud2") add<sensor_msgs::PointCloud2>(node, spec);
        else if (spec.message_type == "nav_msgs/OccupancyGrid") add<nav_msgs::OccupancyGrid>(node, spec);
        else if (spec.message_type == "nav_msgs/Path") add<nav_msgs::Path>(node, spec);
        else add<geometry_msgs::PoseArray>(node, spec);
      }
      worker = std::thread([this] {
        try { while (!stopping.load()) queue.callAvailable(ros::WallDuration(0.1)); }
        catch (...) { fail(std::current_exception()); }
      });
    } catch (...) {
      stopping.store(true); queue.disable();
      for (auto& relay : relays) relay->close();
      relays.clear(); queue.clear(); throw;
    }
  }
  void stop() noexcept {
    std::lock_guard<std::mutex> guard(control_mutex);
    stopping.store(true); queue.disable();
    if (worker.joinable()) worker.join();
    for (auto& relay : relays) relay->close();
    relays.clear(); queue.clear();
  }
  std::vector<RelaySpec> specs;
  ros::CallbackQueue queue;
  std::vector<boost::shared_ptr<Relay>> relays;
  std::thread worker;
  std::atomic<bool> stopping{false};
  bool started = false;
  std::mutex control_mutex;
  mutable std::mutex failure_mutex;
  std::exception_ptr failure;
};
DisplayRelays::DisplayRelays(std::vector<RelaySpec> specs) : impl_(new Impl(std::move(specs))) {}
DisplayRelays::~DisplayRelays() { stop(); }
void DisplayRelays::start(const ros::NodeHandle& node) { impl_->start(node); }
void DisplayRelays::stop() noexcept { impl_->stop(); }
void DisplayRelays::rethrowFailure() const {
  std::lock_guard<std::mutex> guard(impl_->failure_mutex);
  if (impl_->failure) std::rethrow_exception(impl_->failure);
}
}
