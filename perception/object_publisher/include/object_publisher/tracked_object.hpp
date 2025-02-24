#ifndef TRACKED_OBJECT_HPP
#define TRACKED_OBJECT_HPP

// ROS
#include <rclcpp/rclcpp.hpp>

// ROS msg
#include <visualization_msgs/msg/marker.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

// Original
#include "common_cpp/camera.hpp"
#include "common_cpp/get_ros_parameter.hpp"

namespace aiformula {

class TrackedObject {
public:
    explicit TrackedObject(const int& id, const float& initial_x, const float& initial_y, const double& timestamp);
    ~TrackedObject() = default;
    static void initStaticMembers(rclcpp::Node* const node_ptr);
    static void printStaticMembers();
    void generateMarker();
    float computeDistanceSquared(const float& x_in, const float& y_in) const;
    void update(const float& x_in, const float& y_in, const double& timestamp);
    bool isExpired(const double& current_time) const;
    float getX() const { return kf_.statePost.at<float>(0); }
    float getY() const { return kf_.statePost.at<float>(1); }

    visualization_msgs::msg::Marker id_marker;

private:
    static rclcpp::Node* node_ptr_;
    static double process_noise_variance_;
    static double measurement_noise_variance_;
    static double initial_error_covariance_;
    static double expiration_duration_;

    cv::KalmanFilter kf_;
    unsigned int id_;
    double last_seen_time_;
};

}  // namespace aiformula

#endif  // TRACKED_OBJECT_HPP
