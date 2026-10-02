#pragma once
#include <boost/shared_ptr.hpp>
#include <ros/message_traits.h>
#include <ros/serialization.h>
#include <cstdint>
#include <cstring>
#include <vector>
namespace xgc2_ros_display_relays {
// Declared type/MD5 is known before subscribing. No ShapeShifter discovery,
// field decoding, conversion, cache, or header reconstruction is performed.
template<class Declared> struct DeclaredWire {
  using ConstPtr = boost::shared_ptr<const DeclaredWire<Declared>>;
  std::vector<std::uint8_t> bytes;
};
}
namespace ros { namespace message_traits {
template<class T> struct IsMessage<xgc2_ros_display_relays::DeclaredWire<T>> : TrueType {};
template<class T> struct IsMessage<const xgc2_ros_display_relays::DeclaredWire<T>> : TrueType {};
template<class T> struct IsFixedSize<xgc2_ros_display_relays::DeclaredWire<T>> : FalseType {};
// roscpp rewrites header.seq when has_header=true. Raw wire must opt out.
template<class T> struct HasHeader<xgc2_ros_display_relays::DeclaredWire<T>> : FalseType {};
template<class T> struct MD5Sum<xgc2_ros_display_relays::DeclaredWire<T>> {
  static const char* value() { return MD5Sum<T>::value(); }
  static const char* value(const xgc2_ros_display_relays::DeclaredWire<T>&) { return value(); }
};
template<class T> struct DataType<xgc2_ros_display_relays::DeclaredWire<T>> {
  static const char* value() { return DataType<T>::value(); }
  static const char* value(const xgc2_ros_display_relays::DeclaredWire<T>&) { return value(); }
};
template<class T> struct Definition<xgc2_ros_display_relays::DeclaredWire<T>> {
  static const char* value() { return Definition<T>::value(); }
  static const char* value(const xgc2_ros_display_relays::DeclaredWire<T>&) { return value(); }
};
} namespace serialization {
template<class T> struct Serializer<xgc2_ros_display_relays::DeclaredWire<T>> {
  using Wire = xgc2_ros_display_relays::DeclaredWire<T>;
  template<class Stream> static void read(Stream& stream, Wire& message) {
    const auto size = stream.getLength();
    const auto* data = stream.advance(size);
    message.bytes.assign(data, data + size);
  }
  template<class Stream> static void write(Stream& stream, const Wire& message) {
    auto* target = stream.advance(serializedLength(message));
    if (!message.bytes.empty()) std::memcpy(target, message.bytes.data(), message.bytes.size());
  }
  static std::uint32_t serializedLength(const Wire& message) {
    return static_cast<std::uint32_t>(message.bytes.size());
  }
};
}}
