#ifndef GYRO_ODOMETRY_HPP
#define GYRO_ODOMETRY_HPP

// ROS
#include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/rclcpp.hpp>

// ROS msg
#include <can_msgs/msg/frame.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

// Original
#include "get_ros_parameter.hpp"  // common_cpp librtary
#include "odometry.hpp"
#include "util.hpp"  // common_cpp librtary
#include "wheel.hpp"

namespace aiformula {

class GyroOdometry : public Odometry {
public:
    GyroOdometry();
    ~GyroOdometry() = default;

private:
    void getRosParams();
    void initValues();
    void printParam() const;
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void canFrameCallback(const can_msgs::msg::Frame::SharedPtr msg);

    double tire_diameter_;
    double tire_tread_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_sub_;
};

}  // namespace aiformula

#endif  // GYRO_ODOMETRY_HPP
