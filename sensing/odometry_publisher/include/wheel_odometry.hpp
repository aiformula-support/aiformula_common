#ifndef WHEEL_ODOMETRY_HPP
#define WHEEL_ODOMETRY_HPP

// ROS
#include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/rclcpp.hpp>

// ROS msg
#include <can_msgs/msg/frame.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Original
#include "get_ros_parameter.hpp"  // common_cpp librtary
#include "odometry.hpp"
#include "to_geometry_msgs.hpp"  // common_cpp librtary
#include "wheel.hpp"

namespace aiformula {

class WheelOdometry : public Odometry {
public:
    WheelOdometry();
    ~WheelOdometry() = default;

private:
    void getRosParams();
    void initValues();
    void printParam() const;
    void canFrameCallback(const can_msgs::msg::Frame::SharedPtr msg);

    double tire_diameter_;
    double tire_tread_;

    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_sub_;
};

}  // namespace aiformula

#endif  // WHEEL_ODOMETRY_HPP
