#ifndef COMPRESSED_IMAGE_VIEWER_HPP
#define COMPRESSED_IMAGE_VIEWER_HPP

// ROS
#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>

// ROS msg
#include <sensor_msgs/msg/compressed_image.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

// X11
#include <X11/Xlib.h>
#include <X11/extensions/Xinerama.h>

// Original
#include "common_cpp/get_ros_parameter.hpp"

namespace aiformula {

class CompressedImageViewer : public rclcpp::Node {
public:
    explicit CompressedImageViewer();
    ~CompressedImageViewer() = default;

private:
    void getRosParams();
    void initValues();
    void getScreenInfo();
    void printParam() const;
    void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) const;
    void setWindowSizeAndPosition(const cv::Size2i& image_size) const;

    bool display_full_screen_;
    int target_screen_idx_;
    double window_width_ratio_;
    cv::Point2d window_posision_ratio_;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;

    std::string window_name_;
    XineramaScreenInfo* screens_info_;
};

}  // namespace aiformula

#endif  // COMPRESSED_IMAGE_VIEWER_HPP
