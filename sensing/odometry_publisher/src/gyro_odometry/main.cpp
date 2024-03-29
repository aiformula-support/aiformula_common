#include "odometry_publisher/gyro_odometry.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<aiformula::GyroOdometryPublisher>());
    rclcpp::shutdown();
    return 0;
}
