#ifndef OBJECT_PUBLISHER_HPP
#define OBJECT_PUBLISHER_HPP

// ROS
#include <rclcpp/rclcpp.hpp>

// ROS msg
#include <aiformula_interfaces/msg/object_info.hpp>
#include <aiformula_interfaces/msg/object_info_multi_array.hpp>
#include <aiformula_interfaces/msg/rect.hpp>
#include <aiformula_interfaces/msg/rect_multi_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

// Original
#include "common_cpp/camera.hpp"
#include "common_cpp/get_ros_parameter.hpp"
#include "common_cpp/tf2_transform.hpp"
#include "common_cpp/to_geometry_msgs.hpp"
#include "common_cpp/util.hpp"
#include "object_publisher/tracked_object.hpp"

namespace aiformula {

class ObjectPublisher : public rclcpp::Node {
public:
    explicit ObjectPublisher();
    ~ObjectPublisher() = default;

private:
    void getRosParams();
    void initValues();
    void printParam() const;
    void bboxCallback(const aiformula_interfaces::msg::RectMultiArray::SharedPtr msg);
    bool toPositionInVehicle(const aiformula_interfaces::msg::Rect& rect, tf2::Vector3& bottom_left_point,
                             tf2::Vector3& bottom_right_point) const;
    void updateOrAddObject(const tf2::Vector3& bottom_left, const tf2::Vector3& bottom_right,
                           const double& current_time);
    TrackedObject* findClosestObject(const double& obj_x, const double& obj_y);
    void deleteExpiredObjects(const double& current_time);
    void publishObjectInfo(const std_msgs::msg::Header& header, const tf2::Transform& vehicle_T_odom);

    std::string camera_frame_id_;
    std::string vehicle_frame_id_;
    std::string odom_frame_id_;
    bool debug_;
    double object_separation_distance_;

    cv::Mat camera_matrix_;  // This is defined for debug log
    cv::Mat invert_camera_matrix_;
    tf2::Transform vehicle_T_camera_;

    rclcpp::Subscription<aiformula_interfaces::msg::RectMultiArray>::SharedPtr bbox_sub_;
    rclcpp::Publisher<aiformula_interfaces::msg::ObjectInfoMultiArray>::SharedPtr object_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr unfilered_object_pub_;

    std::vector<TrackedObject> tracked_objects_;
};

}  // namespace aiformula

#endif  // OBJECT_PUBLISHER_HPP
