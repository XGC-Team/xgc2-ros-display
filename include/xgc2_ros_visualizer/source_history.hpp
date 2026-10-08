#pragma once

#include <array>
#include <cstdint>
#include <geometry_msgs/Pose.h>
#include <ros/time.h>

namespace xgc2_ros_visualizer {
// Fixed storage, sampled by source stamps. Publishing cadence never samples
// the scientific stream. These rules match the owning SDK's 6s/10Hz history.
struct HistoryPoint {
  ros::Time stamp;
  geometry_msgs::Pose pose;
};
struct SourceHistory {
  std::array<HistoryPoint,61> points;
  std::size_t head{0},size{0};
  std::uint64_t revision{0};
  ros::Time stamp;
  const HistoryPoint& at(std::size_t index) const { return points[(head+index)%points.size()]; }
  void drop() { head=(head+1)%points.size();--size; }
  bool append(const ros::Time& time,const geometry_msgs::Pose& pose) {
    if(time.isZero())return false;
    if(size) {
      const auto previous=at(size-1).stamp;
      if(time<previous) {
        if((previous-time).toSec()>6.0) {head=0;size=0;}
        else return false;
      } else if(time==previous||(time-previous).toSec()<0.1)return false;
    }
    while(size>=2&&(time-at(0).stamp).toSec()>6.0)drop();
    if(size==points.size())drop();
    points[(head+size)%points.size()]={time,pose};++size;stamp=time;++revision;return true;
  }
  bool expire(const ros::Time& now) {
    if(now.isZero()||!size)return false;
    if(now<at(size-1).stamp&&(at(size-1).stamp-now).toSec()>6.0) {
      head=0;size=0;stamp=now;++revision;return true;
    }
    const auto previous=size;
    while(size&&(now-at(0).stamp).toSec()>6.0)drop();
    if(previous==size)return false;
    stamp=size?at(size-1).stamp:now;++revision;return true;
  }
};
} // namespace xgc2_ros_visualizer
