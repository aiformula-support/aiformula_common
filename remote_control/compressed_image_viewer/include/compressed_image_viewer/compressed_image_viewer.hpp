#ifndef COMPRESSED_IMAGE_VIEWER_HPP
#define COMPRESSED_IMAGE_VIEWER_HPP

// ROS
#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>

// ROS msg
#include <sensor_msgs/msg/compressed_image.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

namespace aiformula {

class CompressedImageViewer : public rclcpp::Node {
public:
    explicit CompressedImageViewer();
    ~CompressedImageViewer() = default;

private:
    void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) const;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
};

}  // namespace aiformula

#endif  // COMPRESSED_IMAGE_VIEWER_HPP
