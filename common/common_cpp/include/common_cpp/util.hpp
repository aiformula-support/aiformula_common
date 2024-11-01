#ifndef UTIL_HPP
#define UTIL_HPP

#include <cxxabi.h>

#include <memory>
#include <typeinfo>

// ROS
#include <tf2/impl/utils.h>

// ROS msg
#include <geometry_msgs/msg/quaternion.hpp>

namespace aiformula {

inline double toTimeStampDouble(const builtin_interfaces::msg::Time& msg_stamp) {
    return msg_stamp.sec + static_cast<double>(msg_stamp.nanosec) / 1e9;
}

inline double getYaw(const geometry_msgs::msg::Quaternion& quat_msg) {
    return tf2::impl::getYaw(tf2::impl::toQuaternion(quat_msg));
}

inline std::string toExceptionTypeString(const std::exception& e) {
    int status = 0;
    const std::unique_ptr<char, void (*)(void*)> demangled_name(
        abi::__cxa_demangle(typeid(e).name(), nullptr, nullptr, &status), std::free);

    const std::string type_name = (status == 0) ? demangled_name.get() : typeid(e).name();
    return type_name;
}

}  // namespace aiformula

#endif  // UTIL_HPP
