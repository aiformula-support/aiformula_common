#include "odometry_publisher.hpp"

namespace aiformula {

OdometryPublisher::OdometryPublisher() : Node("odometry_publisher") {
    getRosParams();
    initValues();
    printParam();
}

void OdometryPublisher::getRosParams() {
    // launch
    odom_frame_id_ = getRosParameter<std::string>(this, "odom_frame_id");
    robot_frame_id_ = getRosParameter<std::string>(this, "robot_frame_id");

    // odometry_publisher.yaml
    tire_diameter_ = getRosParameter<double>(this, "tire.diameter");
    tire_tread_ = getRosParameter<double>(this, "tire.tread");
}

void OdometryPublisher::initValues() {
    const int buffer_size = 10;
    can_frame_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "sub_can_frame_topic_name", buffer_size,
        std::bind(&OdometryPublisher::canFrameCallback, this, std::placeholders::_1));
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("pub_odometry_topic_name", buffer_size);

    odometry_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
}

void OdometryPublisher::printParam() const {
    RCLCPP_INFO(this->get_logger(), "[%s] ===============", __func__);

    RCLCPP_INFO(this->get_logger(), "(launch)");
    RCLCPP_INFO(this->get_logger(), "  odom_frame_id_  = %s", odom_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  robot_frame_id_ = %s", robot_frame_id_.c_str());

    RCLCPP_INFO(this->get_logger(), "(odometry_publisher.yaml)");
    RCLCPP_INFO(this->get_logger(), "  tire_diameter_ = %.3lf [m]", tire_diameter_);
    RCLCPP_INFO(this->get_logger(), "  tire_tread_    = %.3lf [m]", tire_tread_);
}

void OdometryPublisher::canFrameCallback(const can_msgs::msg::Frame::SharedPtr msg) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Subscribe Can Frame !");
    if (msg->id != RPM_ID) return;

    static bool is_first = true;
    static double stamp_old = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
    static double yaw = 0.0;
    static double pos_x = 0.0, pos_y = 0.0;

    const Wheel<unsigned int> rpm(msg->data[4], msg->data[0]);
    const Wheel<double> vel = rpm * MINUTE_TO_SECOND * tire_diameter_ * M_PI;
    const double vel_ave = (vel.left + vel.right) * 0.5;
    const double w = (vel.right - vel.left) / tire_tread_;

    const double stamp_now = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nanosec) / 1e9;
    const double dt = is_first ? 0.0 : (stamp_now - stamp_old);
    is_first = false;

    yaw += w * dt;
    const double vel_x = vel_ave * cos(yaw);
    const double vel_y = vel_ave * sin(yaw);
    pos_x += vel_x * dt;
    pos_y += vel_y * dt;
    stamp_old = stamp_now;

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = msg->header.stamp;
    odometry.header.frame_id = odom_frame_id_;
    odometry.child_frame_id = robot_frame_id_;
    odometry.pose.pose.position = toPointMsg(pos_x, pos_y, 0.0);
    odometry.pose.pose.orientation = toQuaternionMsg(0.0, 0.0, yaw);
    odometry.twist.twist.linear = toVector3Msg(vel_x, vel_y, 0.0);
    odometry.twist.twist.angular = toVector3Msg(0.0, 0.0, w);
    odometry_pub_->publish(odometry);
    broadcastTf(odometry);
    RCLCPP_INFO_ONCE(this->get_logger(), "Pubscribe Odometry !");
}

void OdometryPublisher::broadcastTf(const nav_msgs::msg::Odometry& odom) const {
    geometry_msgs::msg::TransformStamped ts;
    ts.header = odom.header;
    ts.child_frame_id = odom.child_frame_id;
    ts.transform.translation = toVector3Msg(odom.pose.pose.position);
    ts.transform.rotation = odom.pose.pose.orientation;
    odometry_br_->sendTransform(ts);
}

}  // namespace aiformula
