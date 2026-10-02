#pragma once
#include <chrono>
namespace xgc2_ros_display_relays { namespace detail {
// Accessed only on the private callback thread. Steady time, never ROS /clock.
class RateGate {
 public:
  using Clock = std::chrono::steady_clock;
  explicit RateGate(double hz) : interval_(1.0 / hz) {}
  void reset() noexcept { sent_ = false; }
  bool admit(Clock::time_point now) noexcept {
    if (sent_ && now - last_ < interval_) return false;
    sent_ = true; last_ = now; return true;
  }
 private:
  std::chrono::duration<double> interval_;
  bool sent_ = false;
  Clock::time_point last_{};
};
}}
