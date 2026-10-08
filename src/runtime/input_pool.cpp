#include "input_pool.hpp"
#include <stdexcept>
namespace xgc2_ros_visualizer {
InputPool::InputPool(std::size_t count) {
  if (count<1 || count>32) throw std::invalid_argument("callback_workers must be within 1..32");
  queues_.reserve(count); workers_.reserve(count);
  for (std::size_t i=0;i<count;++i) queues_.emplace_back(new ros::CallbackQueue);
  try { for (auto& queue:queues_) {
    auto* selected=queue.get();
    workers_.emplace_back([this,selected] {
      try { while (!stopping_.load()) selected->callAvailable(ros::WallDuration(.01)); }
      catch(...) {std::lock_guard<std::mutex> lock(failure_mutex_);if(!failure_)failure_=std::current_exception();stopping_.store(true);}
    });
  } } catch(...) {stop();throw;}
}
InputPool::~InputPool() { stop(); }
ros::NodeHandle InputPool::node(const std::string& source) const {
  // Stable FNV-1a sharding, independent of process/library hash randomization.
  std::uint64_t hash=14695981039346656037ULL;
  for (unsigned char value:source) { hash^=value; hash*=1099511628211ULL; }
  ros::NodeHandle result; result.setCallbackQueue(queues_[hash%queues_.size()].get()); return result;
}
void InputPool::stop() {
  stopping_.store(true);
  for (auto& queue:queues_) queue->disable();
  for (auto& worker:workers_) if (worker.joinable()) worker.join();
  for (auto& queue:queues_) queue->clear();
}
void InputPool::rethrowFailure() {std::lock_guard<std::mutex> lock(failure_mutex_);if(failure_)std::rethrow_exception(failure_);}
} // namespace xgc2_ros_visualizer
