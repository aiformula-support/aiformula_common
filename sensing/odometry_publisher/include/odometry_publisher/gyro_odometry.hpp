#ifndef GYRO_ODOMETRY_PUBLISHER_HPP
#define GYRO_ODOMETRY_PUBLISHER_HPP

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
    void publishOdometryCallback();
    void initializeVariablesForLinearInterpolation(const size_t& prev_imu_idx, double& prev_imu_time,
                                                   double& current_imu_time, tf2::Quaternion& prev_orientation,
                                                   tf2::Quaternion& current_orientation, double& yaw_rate_lerp_a,
                                                   double& yaw_rate_lerp_b) const;
    void linearlyInterpolateAngleAndRate(const double& current_can_time, const double& prev_imu_time,
                                         const double& current_imu_time, const tf2::Quaternion& prev_orientation,
                                         const tf2::Quaternion& current_orientation, const double& yaw_angle_offset,
                                         const double& yaw_rate_lerp_a, const double& yaw_rate_lerp_b);

    int publish_timer_loop_duration_;
    double wheel_diameter_;
    double wheel_tread_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_msgs_;
    std::vector<can_msgs::msg::Frame::SharedPtr> can_msgs_;
};

}  // namespace aiformula

#endif  // GYRO_ODOMETRY_PUBLISHER_HPP
