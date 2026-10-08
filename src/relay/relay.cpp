#include "relay.hpp"
#include <xgc2_ros_visualizer/declared_wire.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <stdexcept>
namespace xgc2_ros_visualizer {
namespace {
template<class Message> class TypedRelay final : public Relay {
  using Wire=DeclaredWire<Message>;
  struct Latest {
    std::mutex mutex;
    typename Wire::ConstPtr message;
    std::uint64_t generation{0};
  };
 public:
  TypedRelay(InputPool& pool,const RelayConfig& spec) : spec_(spec),latest_(new Latest) {
    auto node=pool.node(spec.source);
    if (node.resolveName(spec.source)!=spec.source || node.resolveName(spec.topic)!=spec.topic)
      throw std::invalid_argument("remapping changed a canonical relay topic");
    ros::AdvertiseOptions options;
    options.template init<Wire>(spec.topic,1);
    options.has_header=false; options.latch=false;
    publisher_=node.advertise(options);
    auto latest=latest_;
    boost::function<void(const typename Wire::ConstPtr&)> callback=[latest](const typename Wire::ConstPtr& message) {
      std::lock_guard<std::mutex> lock(latest->mutex);
      latest->message=message; ++latest->generation;
    };
    subscriber_=node.subscribe<Wire>(spec.source,1,callback,ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
    if (!publisher_ || !subscriber_) throw std::runtime_error("cannot activate relay: "+spec.source);
  }
  bool publish(const Rates& rates,std::chrono::steady_clock::time_point now) override {
    if (sent_ && std::chrono::duration<double>(now-last_).count()<1.0/rates.get(spec_.kind,spec_.channel)) return false;
    typename Wire::ConstPtr message; std::uint64_t generation;
    { std::lock_guard<std::mutex> lock(latest_->mutex); message=latest_->message; generation=latest_->generation; }
    if (!message || generation==published_generation_) return false;
    publisher_.publish(message); // Same immutable source shared_ptr; no payload copy/replay.
    published_generation_=generation; last_=now; sent_=true; ++count_;return true;
  }
  void stop() override { subscriber_.shutdown(); publisher_.shutdown(); }
  std::uint64_t count() const override { return count_.load(); }
 private:
  RelayConfig spec_;
  std::shared_ptr<Latest> latest_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  std::uint64_t published_generation_{0};
  std::atomic<std::uint64_t> count_{0};
  std::chrono::steady_clock::time_point last_{};
  bool sent_{false};
};
}
std::unique_ptr<Relay> makeRelay(InputPool& pool,const RelayConfig& spec) {
  if (spec.message_type=="sensor_msgs/PointCloud2") return std::unique_ptr<Relay>(new TypedRelay<sensor_msgs::PointCloud2>(pool,spec));
  if (spec.message_type=="nav_msgs/OccupancyGrid") return std::unique_ptr<Relay>(new TypedRelay<nav_msgs::OccupancyGrid>(pool,spec));
  if (spec.message_type=="nav_msgs/Path") return std::unique_ptr<Relay>(new TypedRelay<nav_msgs::Path>(pool,spec));
  if (spec.message_type=="geometry_msgs/PoseArray") return std::unique_ptr<Relay>(new TypedRelay<geometry_msgs::PoseArray>(pool,spec));
  throw std::invalid_argument("unsupported relay type");
}
} // namespace xgc2_ros_visualizer
