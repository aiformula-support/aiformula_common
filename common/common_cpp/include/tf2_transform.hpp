#ifndef TF2_TRANSFORM_HPP
#define TF2_TRANSFORM_HPP

// ROS
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>

namespace aiformula {

tf2::Transform getTf2Transform(rclcpp::Node* node_ptr, const std::string& dst_frame_id,
                               const std::string& src_frame_id);

}  // namespace aiformula

#endif  // TF2_TRANSFORM_HPP
