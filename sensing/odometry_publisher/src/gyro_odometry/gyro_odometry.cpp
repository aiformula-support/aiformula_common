#include "odometry_publisher/gyro_odometry.hpp"

namespace aiformula {

GyroOdometryPublisher::GyroOdometryPublisher() : OdometryPublisher("gyro_odometry") {
    getRosParams();
    initValues();
    printParam();
}

void GyroOdometryPublisher::getRosParams() {
    // From 'gyro_odometry.yaml'
    publish_timer_loop_duration_ = getRosParameter<int>(this, "publish_timer_loop_duration");

    // From 'wheel.yaml'
    wheel_diameter_ = getRosParameter<double>(this, "wheel.diameter");
    wheel_tread_ = getRosParameter<double>(this, "wheel.tread");
}

void GyroOdometryPublisher::initValues() {
    const int buffer_size = 10;
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "sub_imu", buffer_size, std::bind(&GyroOdometryPublisher::imuCallback, this, std::placeholders::_1));
    can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "sub_can", buffer_size, std::bind(&GyroOdometryPublisher::canFrameCallback, this, std::placeholders::_1));

    publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(publish_timer_loop_duration_),
                                             std::bind(&GyroOdometryPublisher::publishOdometryCallback, this));
}

void GyroOdometryPublisher::printParam() const {
    RCLCPP_INFO(this->get_logger(), "[%s] ===============", __func__);
    RCLCPP_INFO(this->get_logger(), "(gyro_odometry.yaml)");
    RCLCPP_INFO(this->get_logger(), "  publish_timer_loop_duration_ = %d [msec]", publish_timer_loop_duration_);

    RCLCPP_INFO(this->get_logger(), "(wheel.yaml)");
    RCLCPP_INFO(this->get_logger(), "  wheel_diameter_ = %.3lf [m]", wheel_diameter_);
    RCLCPP_INFO(this->get_logger(), "  wheel_tread_    = %.3lf [m]", wheel_tread_);
    RCLCPP_INFO(this->get_logger(), "============================\n");
}

void GyroOdometryPublisher::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Subscribe Imu !");
    imu_msgs_.emplace_back(msg);
}

void GyroOdometryPublisher::canFrameCallback(const can_msgs::msg::Frame::SharedPtr msg) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Subscribe Can Frame !");
    if (msg->id != odometry_publisher::RPM_ID) return;
    can_msgs_.emplace_back(msg);
}

void GyroOdometryPublisher::publishOdometryCallback() {
    if (imu_msgs_.size() < 2 || !can_msgs_.size()) return;
    static const double yaw_angle_offset = getYaw(imu_msgs_[0]->orientation);

    static double prev_can_time = 0.0;
    size_t prev_imu_idx = 0, can_idx = 0;
    for (; prev_imu_idx < imu_msgs_.size() - 1; ++prev_imu_idx) {
        // Variables for Linear interpolation of 'yaw angle' and 'yaw rate'
        double prev_imu_time, current_imu_time;
        tf2::Quaternion prev_orientation, current_orientation;
        double yaw_rate_lerp_a, yaw_rate_lerp_b;
        initializeVariablesForLinearInterpolation(prev_imu_idx, prev_imu_time, current_imu_time, prev_orientation,
                                                  current_orientation, yaw_rate_lerp_a, yaw_rate_lerp_b);

        while (rclcpp::ok()) {
            if (can_idx == can_msgs_.size()) break;  // Finish when all can messages have been processed.

            const double current_can_time = toTimeStampDouble(can_msgs_[can_idx]->header.stamp);
            if (current_imu_time < current_can_time)
                break;  // Increment imu index because interpolation is not possible.

            if (prev_can_time) {  // Other than the first can message
                const double vehicle_linear_velocity =
                    odometry_publisher::calcVehicleLinearVelocity(can_msgs_[can_idx], wheel_diameter_);
                linearlyInterpolateAngleAndRate(current_can_time, prev_imu_time, current_imu_time, prev_orientation,
                                                current_orientation, yaw_angle_offset, yaw_rate_lerp_a,
                                                yaw_rate_lerp_b);
                const double dt = current_can_time - prev_can_time;
                updatePosition(vehicle_linear_velocity, dt);
                nav_msgs::msg::Odometry odometry = createOdometryMsg(can_msgs_[can_idx]->header.stamp);
                odometry_pub_->publish(odometry);
                broadcastTf(odometry);
                RCLCPP_INFO_ONCE(this->get_logger(), "Publish Odometry !");
            }
            prev_can_time = current_can_time;
            ++can_idx;
        }
    }
    // Erase used messages.
    imu_msgs_.erase(imu_msgs_.begin(), imu_msgs_.begin() + prev_imu_idx);
    if (can_idx) can_msgs_.erase(can_msgs_.begin(), can_msgs_.begin() + can_idx);
}

void GyroOdometryPublisher::initializeVariablesForLinearInterpolation(
    const size_t& prev_imu_idx, double& prev_imu_time, double& current_imu_time, tf2::Quaternion& prev_orientation,
    tf2::Quaternion& current_orientation, double& yaw_rate_lerp_a, double& yaw_rate_lerp_b) const {
    prev_imu_time = toTimeStampDouble(imu_msgs_[prev_imu_idx]->header.stamp);
    current_imu_time = toTimeStampDouble(imu_msgs_[prev_imu_idx + 1]->header.stamp);
    prev_orientation = tf2::impl::toQuaternion(imu_msgs_[prev_imu_idx]->orientation);
    current_orientation = tf2::impl::toQuaternion(imu_msgs_[prev_imu_idx + 1]->orientation);
    const double prev_yaw_rate = imu_msgs_[prev_imu_idx]->angular_velocity.z;
    const double current_yaw_rate = imu_msgs_[prev_imu_idx + 1]->angular_velocity.z;
    yaw_rate_lerp_a = (current_yaw_rate - prev_yaw_rate) / (current_imu_time - prev_imu_time);
    yaw_rate_lerp_b = prev_yaw_rate - yaw_rate_lerp_a * prev_imu_time;
}

void GyroOdometryPublisher::linearlyInterpolateAngleAndRate(
    const double& current_can_time, const double& prev_imu_time, const double& current_imu_time,
    const tf2::Quaternion& prev_orientation, const tf2::Quaternion& current_orientation, const double& yaw_angle_offset,
    const double& yaw_rate_lerp_a, const double& yaw_rate_lerp_b) {
    const double ratio = (current_can_time - prev_imu_time) / (current_imu_time - prev_imu_time);
    const auto orientation = prev_orientation.slerp(current_orientation, ratio);
    yaw_angle_ = tf2::impl::getYaw(orientation) - yaw_angle_offset;
    yaw_rate_ = yaw_rate_lerp_a * current_can_time + yaw_rate_lerp_b;
}

}  // namespace aiformula
