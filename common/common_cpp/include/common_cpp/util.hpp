#ifndef UTIL_HPP
#define UTIL_HPP

// ROS
#include <tf2/impl/utils.h>

// ROS msg
#include <geometry_msgs/msg/quaternion.hpp>

namespace aiformula {

class BasicException : public std::exception {
public:
    BasicException(const std::string& error_message) : error_message_(error_message) {}
    const char* what() const noexcept override { return error_message_.c_str(); }

protected:
    std::string error_message_;
};

inline double toTimeStampDouble(const builtin_interfaces::msg::Time& msg_stamp) {
    return msg_stamp.sec + static_cast<double>(msg_stamp.nanosec) / 1e9;
}

inline double getYaw(const geometry_msgs::msg::Quaternion& quat_msg) {
    return tf2::impl::getYaw(tf2::impl::toQuaternion(quat_msg));
}

}  // namespace aiformula

#endif  // UTIL_HPP
