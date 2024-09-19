#ifndef COMPRESSED_IMAGE_VIEWER_HPP
#define COMPRESSED_IMAGE_VIEWER_HPP

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>

// ROS msg
#include <sensor_msgs/msg/compressed_image.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

// X11
#include <X11/Xlib.h>
#include <X11/extensions/Xinerama.h>

// Original
#include "common_cpp/get_ros_parameter.hpp"
#include "compressed_image_viewer/exception.hpp"

namespace aiformula {

class CompressedImageViewer : public rclcpp::Node {
public:
    explicit CompressedImageViewer();
    ~CompressedImageViewer() = default;
    void setup();

private:
    void getRosParams();
    void initValues();
    XineramaScreenInfo* getScreenInfo();
    void getScreenInfo(XineramaScreenInfo*& screen_infos) const;
    void setWindowSizeAndPosition(XineramaScreenInfo* const& screens_info);
    void printParam() const;
    void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) const;

    bool display_full_screen_;
    int target_screen_idx_;
    int display_scale_setting_;
    double window_width_ratio_;
    cv::Point2d window_position_ratio_;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;

    std::string window_name_;
};

}  // namespace aiformula

#endif  // COMPRESSED_IMAGE_VIEWER_HPP
