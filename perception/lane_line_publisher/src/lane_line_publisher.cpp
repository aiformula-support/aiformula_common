#include "lane_line_publisher/lane_line_publisher.hpp"

namespace aiformula {

LaneLinePublisher::LaneLinePublisher() : Node("lane_line_publisher") {
    initMembers();
    initConnections();
    RCLCPP_INFO(get_logger(), "Launched %s", get_name());
}

void LaneLinePublisher::initMembers() {
    debug_ = getRosParameter<bool>(this, "debug");
    vehicle_frame_id_ = getRosParameter<std::string>(this, "vehicle_frame_id");
    xmin_ = getRosParameter<double>(this, "xmin");
    xmax_ = getRosParameter<double>(this, "xmax");
    ymin_ = getRosParameter<double>(this, "ymin");
    ymax_ = getRosParameter<double>(this, "ymax");
    spacing_ = getRosParameter<double>(this, "spacing");
    const auto camera_frame_id = getRosParameter<std::string>(this, "camera_frame_id");
    const auto min_area = getRosParameter<int>(this, "min_area");
    const auto tolerance = getRosParameter<int>(this, "tolerance");

    getCameraParams(this, "zedX.LEFT_CAM_SVGA", camera_matrix_);
    camera_matrix_ = camera_matrix_.inv();
    vehicle_T_camera_ = getTf2Transform(this, vehicle_frame_id_, camera_frame_id);

    lane_pixel_finder_ = std::make_shared<const LanePixelFinder>(min_area, tolerance);
    cubic_line_fitter_ = std::make_shared<const CubicLineFitter>(xmin_, xmax_, spacing_);
}

void LaneLinePublisher::initConnections() {
    const auto queue_size = getRosParameter<int>(this, "queue_size");
    mask_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "mask_image", queue_size, std::bind(&LaneLinePublisher::imageCallback, this, std::placeholders::_1));
    lane_lines_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("lane_lines", queue_size);

    if (!debug_) return;

    annotated_mask_image_pub_ = create_publisher<sensor_msgs::msg::Image>("annotated_mask_image", queue_size);
}

void LaneLinePublisher::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const {
    RCLCPP_DEBUG(get_logger(), "Recieved image [%ix%i]", msg->width, msg->height);

    auto cv_img = cv_bridge::toCvShare(msg, msg->encoding);
    if (cv_img->image.empty()) {
        RCLCPP_WARN(get_logger(), "Recieved empty image");
        return;
    }

    LaneLines lane_lines;
    findLaneLines(cv_img->image, msg->header.stamp, lane_lines);
    publishLaneLines(lane_lines);
}

void LaneLinePublisher::findLaneLines(const cv::Mat &mask, const builtin_interfaces::msg::Time &timestamp,
                                      LaneLines &lane_lines) const {
    lane_pixel_finder_->findLanePixels(mask, lane_lines);
    if (debug_) publishAnnotatedMask(mask, timestamp, lane_lines);

    for (auto *lane_line : {&lane_lines.left, &lane_lines.right, &lane_lines.center}) {
        lane_line->toVehicleFrame(camera_matrix_, vehicle_T_camera_);
        lane_line->cropToRoi(xmin_, xmax_, ymin_, ymax_);
        lane_line->respacePoints(spacing_);
        lane_line->fitPoints(cubic_line_fitter_);
    }
}

void LaneLinePublisher::publishAnnotatedMask(const cv::Mat &mask, const builtin_interfaces::msg::Time &timestamp,
                                             const LaneLines &lane_lines) const {
    cv_bridge::CvImage cv_img;
    cv_img.header.stamp = timestamp;
    cv_img.encoding = "bgr8";
    cv_img.image = lane_pixel_finder_->visualizeLanePixels(mask, lane_lines);

    sensor_msgs::msg::Image msg;
    cv_img.toImageMsg(msg);
    annotated_mask_image_pub_->publish(std::move(msg));
}

visualization_msgs::msg::Marker LaneLinePublisher::createLaneLineMarker(const LaneLine &lane_line, const int &id,
                                                                        const std::array<double, 3> &color) const {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = vehicle_frame_id_;
    marker.id = id;
    marker.action = visualization_msgs::msg::Marker::MODIFY;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;
    for (const auto &point : lane_line.points) {
        auto &p = marker.points.emplace_back();
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
    }
    return marker;
}

void LaneLinePublisher::publishLaneLines(const LaneLines &lane_lines) const {
    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    auto &marker = marker_array.markers.emplace_back();
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker.id = id++;

    static const std::array<double, 3> GREEN = {0.0, 1.0, 0.5};
    static const std::array<double, 3> BLUE = {0.0, 0.5, 1.0};
    marker_array.markers.emplace_back(createLaneLineMarker(lane_lines.left, id++, GREEN));
    marker_array.markers.emplace_back(createLaneLineMarker(lane_lines.right, id++, GREEN));
    marker_array.markers.emplace_back(createLaneLineMarker(lane_lines.center, id++, BLUE));

    lane_lines_pub_->publish(marker_array);
}

}  // namespace aiformula
