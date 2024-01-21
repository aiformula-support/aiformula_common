#ifndef UTIL_HPP
#define UTIL_HPP

// ROS
#include <rclcpp/rclcpp.hpp>

// C++
#include <type_traits>

namespace aiformula {

template <typename T>
inline void checkVariable(const rclcpp::Node* node_ptr, const T& variable, const T& false_value,
                          const std::string& error_statement);
}  // namespace aiformula

#endif  // UTIL_HPP
