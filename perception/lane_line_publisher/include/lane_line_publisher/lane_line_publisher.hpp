#ifndef LANE_LINE_PUBLISHER_HPP
#define LANE_LINE_PUBLISHER_HPP

#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "camera.hpp"
#include "get_ros_parameter.hpp"
#include "lane_line_publisher/lane_line.hpp"
#include "lane_line_publisher/lane_pixel_finder.hpp"

namespace aiformula {

class LaneLinePublisher : public rclcpp::Node {
public:
    LaneLinePublisher();
    ~LaneLinePublisher() = default;

private:
    void initMembers();
    void initConnections();

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const;
    void findLaneLines(const cv::Mat& mask, const builtin_interfaces::msg::Time& timestamp,
                       LaneLines& lane_lines) const;
    void publishAnnotatedMask(const cv::Mat& mask, const builtin_interfaces::msg::Time& timestamp,
                              const LaneLines& lane_lines) const;
    visualization_msgs::msg::Marker createLaneLineMarker(const LaneLine& lane_line, const int& id,
                                                         const std::array<double, 3>& color) const;
    void publishLaneLines(const LaneLines& lane_lines) const;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_mask_image_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lane_lines_pub_;

    LanePixelFinder::ConstPtr lane_pixel_finder_;
    CubicLineFitter::ConstPtr cubic_line_fitter_;

    bool debug_;
    std::string vehicle_frame_id_;
    double xmin_, xmax_, ymin_, ymax_, spacing_;
    cv::Mat camera_matrix_;
    tf2::Transform vehicle_T_camera_;
};

}  // namespace aiformula

#endif  // LANE_LINE_PUBLISHER_HPP
