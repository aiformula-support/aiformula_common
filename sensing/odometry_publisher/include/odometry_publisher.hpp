#ifndef ODOMETRY_PUBLISHER_HPP
#define ODOMETRY_PUBLISHER_HPP

// ROS
#include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/rclcpp.hpp>

// ROS msg
#include <can_msgs/msg/frame.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Original
#include "get_ros_parameter.hpp"  // common_cpp librtary
#include "to_geometry_msgs.hpp"   // common_cpp librtary

namespace aiformula {

class OdometryPublisher : public rclcpp::Node {
public:
    OdometryPublisher();
    ~OdometryPublisher() = default;

private:
    template <typename T>
    struct Wheel {
        Wheel(const T& left, const T& right) : left(left), right(right) {}
        Wheel<double> operator*(const double& rhs) const { return {left * rhs, right * rhs}; }
        T left, right;
    };
    const int RPM_ID = 1809;
    const double MINUTE_TO_SECOND = 0.016667;  // = 1/60
    const double DEGREE_TO_RADIAN = M_PI / 180.0;
    const double RADIAN_TO_DEGREE = 180.0 / M_PI;

    void getRosParams();
    void initValues();
    void printParam() const;
    void canFrameCallback(const can_msgs::msg::Frame::SharedPtr msg);
    void broadcastTf(const nav_msgs::msg::Odometry& odom) const;

    std::string odom_frame_id_;
    std::string robot_frame_id_;
    double tire_diameter_;
    double tire_tread_;

    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odometry_br_;
};

}  // namespace aiformula

#endif  // ODOMETRY_PUBLISHER_HPP
