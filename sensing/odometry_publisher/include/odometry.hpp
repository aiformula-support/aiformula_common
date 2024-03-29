#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

// ROS
#include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/rclcpp.hpp>

// ROS msg
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Original
#include "get_ros_parameter.hpp"  // common_cpp librtary
#include "to_geometry_msgs.hpp"   // common_cpp librtary
#include "util.hpp"               // common_cpp librtary

namespace aiformula {

class Odometry : public rclcpp::Node {
public:
    Odometry(const std::string& node_name);
    ~Odometry() = default;

protected:
    double calcTimeDelta(const builtin_interfaces::msg::Time& msg_stamp);
    void updatePosition(const double& dt, const double& linear_velocity_ave);
    nav_msgs::msg::Odometry createOdometryMsg(const builtin_interfaces::msg::Time& msg_stamp) const;
    void broadcastTf(const nav_msgs::msg::Odometry& odom);

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    double yaw_angle_;
    double yaw_rate_;

private:
    void getRosParams();
    void initValues();
    void printParam() const;

    std::string odom_frame_id_;
    std::string robot_frame_id_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> odometry_br_;
    double stamp_prev_;
    geometry_msgs::msg::Point pos_;
    geometry_msgs::msg::Vector3 linear_velocity_;
};

}  // namespace aiformula

#endif  // ODOMETRY_HPP
