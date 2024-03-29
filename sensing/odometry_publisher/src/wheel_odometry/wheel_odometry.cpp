#include "odometry_publisher/wheel_odometry.hpp"

namespace aiformula {

WheelOdometryPublisher::WheelOdometryPublisher() : OdometryPublisher("wheel_odometry") {
    getRosParams();
    initValues();
    printParam();
}

void WheelOdometryPublisher::getRosParams() {
    // wheel.yaml
    tire_diameter_ = getRosParameter<double>(this, "wheel.diameter");
    tire_tread_ = getRosParameter<double>(this, "wheel.tread");
}

void WheelOdometryPublisher::initValues() {
    const int buffer_size = 10;
    can_frame_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "sub_can_frame", buffer_size,
        std::bind(&WheelOdometryPublisher::canFrameCallback, this, std::placeholders::_1));
}

void WheelOdometryPublisher::printParam() const {
    RCLCPP_INFO(this->get_logger(), "[%s] ===============", __func__);
    RCLCPP_INFO(this->get_logger(), "(wheel.yaml)");
    RCLCPP_INFO(this->get_logger(), "  tire_diameter_ = %.3lf [m]", tire_diameter_);
    RCLCPP_INFO(this->get_logger(), "  tire_tread_    = %.3lf [m]", tire_tread_);
    RCLCPP_INFO(this->get_logger(), "============================\n");
}

void WheelOdometryPublisher::canFrameCallback(const can_msgs::msg::Frame::SharedPtr msg) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Subscribe Can Frame !");
    if (msg->id != odometry_publisher::RPM_ID) return;

    const odometry_publisher::WheelRates<unsigned int> rpm(msg->data[4], msg->data[0]);
    const odometry_publisher::WheelRates<double> vel =
        rpm * odometry_publisher::MINUTE_TO_SECOND * tire_diameter_ * M_PI;
    const double vehicle_linear_velocity = (vel.left + vel.right) * 0.5;
    yaw_rate_ = (vel.right - vel.left) / tire_tread_;

    const double dt = calcTimeDelta(msg->header.stamp);
    yaw_angle_ += yaw_rate_ * dt;
    updatePosition(dt, vehicle_linear_velocity);
    nav_msgs::msg::Odometry odometry = createOdometryMsg(msg->header.stamp);
    odometry_pub_->publish(odometry);
    broadcastTf(odometry);
    RCLCPP_INFO_ONCE(this->get_logger(), "Publish Odometry !");
}

}  // namespace aiformula
