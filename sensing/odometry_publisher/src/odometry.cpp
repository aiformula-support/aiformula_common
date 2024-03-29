#include "odometry.hpp"

namespace aiformula {

Odometry::Odometry(const std::string& node_name)
    : Node(node_name),
      stamp_prev_(0.0),
      pos_(toPointMsg(0.0, 0.0, 0.0)),
      yaw_angle_(0.0),
      linear_velocity_(toVector3Msg(0.0, 0.0, 0.0)),
      yaw_rate_(0.0) {
    getRosParams();
    initValues();
    printParam();
}

void Odometry::getRosParams() {
    // launch
    odom_frame_id_ = getRosParameter<std::string>(this, "odom_frame_id");
    robot_frame_id_ = getRosParameter<std::string>(this, "robot_frame_id");
}

void Odometry::initValues() {
    const int buffer_size = 10;
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("pub_odometry", buffer_size);
    odometry_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
}

void Odometry::printParam() const {
    RCLCPP_INFO(this->get_logger(), "[%s] ===============", __func__);
    RCLCPP_INFO(this->get_logger(), "(launch)");
    RCLCPP_INFO(this->get_logger(), "  odom_frame_id_  = %s", odom_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  robot_frame_id_ = %s", robot_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "============================\n");
}

double Odometry::calcTimeDelta(const builtin_interfaces::msg::Time& msg_stamp) {
    if (!stamp_prev_) {
        stamp_prev_ = toTimeStampDouble(msg_stamp);
        return 0.0;
    }
    const double stamp_now = toTimeStampDouble(msg_stamp);
    const double dt = stamp_now - stamp_prev_;
    stamp_prev_ = stamp_now;
    return dt;
}

void Odometry::updatePosition(const double& dt, const double& linear_velocity_ave) {
    linear_velocity_.x = linear_velocity_ave * cos(yaw_angle_);
    linear_velocity_.y = linear_velocity_ave * sin(yaw_angle_);
    pos_.x += linear_velocity_.x * dt;
    pos_.y += linear_velocity_.y * dt;
}

nav_msgs::msg::Odometry Odometry::createOdometryMsg(const builtin_interfaces::msg::Time& msg_stamp) const {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = msg_stamp;
    odometry.header.frame_id = odom_frame_id_;
    odometry.child_frame_id = robot_frame_id_;
    odometry.pose.pose.position = pos_;
    odometry.pose.pose.orientation = toQuaternionMsg(0.0, 0.0, yaw_angle_);
    odometry.twist.twist.linear = linear_velocity_;
    odometry.twist.twist.angular = toVector3Msg(0.0, 0.0, yaw_rate_);
    return odometry;
}

void Odometry::broadcastTf(const nav_msgs::msg::Odometry& odom) {
    geometry_msgs::msg::TransformStamped ts;
    ts.header = odom.header;
    ts.child_frame_id = odom.child_frame_id;
    ts.transform.translation = toVector3Msg(odom.pose.pose.position);
    ts.transform.rotation = odom.pose.pose.orientation;
    odometry_br_->sendTransform(ts);
}

}  // namespace aiformula
