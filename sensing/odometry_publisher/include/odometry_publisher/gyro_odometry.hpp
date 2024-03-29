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
#include "common_cpp/get_ros_parameter.hpp"
#include "common_cpp/util.hpp"
#include "odometry_publisher/odometry.hpp"
#include "odometry_publisher/wheel.hpp"

namespace aiformula {

class GyroOdometryPublisher : public OdometryPublisher {
public:
    explicit GyroOdometryPublisher();
    ~GyroOdometryPublisher() = default;

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
