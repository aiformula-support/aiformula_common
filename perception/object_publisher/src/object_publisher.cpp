#include "object_publisher/object_publisher.hpp"

namespace aiformula {

ObjectPublisher::ObjectPublisher()
    : Node("object_publisher"),
      camera_frame_id_(""),
      vehicle_frame_id_(""),
      odom_frame_id_(""),
      debug_(false),
      object_separation_distance_(5.0),
      camera_matrix_(cv::Mat()),
      invert_camera_matrix_(cv::Mat()),
      vehicle_T_camera_(tf2::Transform()),
      tracked_objects_(std::vector<TrackedObject>()) {
    getRosParams();
    initValues();
    printParam();
}

void ObjectPublisher::getRosParams() {
    // From 'launch file'
    camera_frame_id_ = getRosParameter<std::string>(this, "camera_frame_id");
    vehicle_frame_id_ = getRosParameter<std::string>(this, "vehicle_frame_id");
    odom_frame_id_ = getRosParameter<std::string>(this, "odom_frame_id");
    debug_ = getRosParameter<bool>(this, "debug");

    // From 'object_publisher.yaml'
    object_separation_distance_ = getRosParameter<double>(this, "object_separation_distance");

    TrackedObject::initStaticMembers(this);
}

void ObjectPublisher::initValues() {
    // Subscriber & Publisher
    const int buffer_size = 10;
    bbox_sub_ = this->create_subscription<aiformula_interfaces::msg::RectMultiArray>(
        "sub_bbox", buffer_size, std::bind(&ObjectPublisher::bboxCallback, this, std::placeholders::_1));
    object_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("pub_object", buffer_size);
    unfilered_object_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("pub_unfilered_object", buffer_size);
    object_id_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("pub_object_id", buffer_size);

    getCameraParams(this, "zedx", camera_matrix_);
    invert_camera_matrix_ = camera_matrix_.inv();
    vehicle_T_camera_ = getTf2Transform(this, vehicle_frame_id_, camera_frame_id_);
}

void ObjectPublisher::printParam() const {
    std::ostringstream log_stream;
    const auto& formatter = createFormatter(20);
    const auto& trans = vehicle_T_camera_.getOrigin();
    const auto& rot = vehicle_T_camera_.getRotation();
    log_stream << "\n[" << __func__ << "] " << std::string(57, '=') << "\n"
               << "(launch)\n"
               << formatter("camera_frame_id_") << camera_frame_id_ << "\n"
               << formatter("vehicle_frame_id_") << vehicle_frame_id_ << "\n"
               << formatter("odom_frame_id_") << odom_frame_id_ << "\n"
               << formatter("debug_") << (debug_ ? "true" : "false") << "\n"
               << "\n(object_publisher.yaml)\n"
               << std::fixed << std::setprecision(1) << formatter("object_separation_distance_")
               << object_separation_distance_ << " [m]\n"
               << "\n(initValues)\n"
               << formatter("camera_matrix_") << "[" << camera_matrix_.row(0) << "\n"
               << std::string(27, ' ') << camera_matrix_.row(1) << "\n"
               << std::string(27, ' ') << camera_matrix_.row(2) << "]\n"
               << std::setprecision(2) << formatter("trans") << "(" << trans.x() << ", " << trans.y() << ", "
               << trans.z() << ") [m]\n"
               << formatter("rot") << "(" << rot.x() << ", " << rot.y() << ", " << rot.z() << ", " << rot.w() << ")\n"
               << std::string(70, '=') << "\n";
    RCLCPP_INFO(this->get_logger(), "%s", log_stream.str().c_str());
    TrackedObject::printStaticMembers();
}

void ObjectPublisher::bboxCallback(const aiformula_interfaces::msg::RectMultiArray::SharedPtr msg) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Subscribe BBox Rect !");
    std_msgs::msg::Header header = msg->header;
    header.frame_id = vehicle_frame_id_;

    geometry_msgs::msg::PoseArray unfiltered_object_msg;
    if (debug_) unfiltered_object_msg.header = header;

    const auto odom_T_vehicle = getTf2Transform(this, odom_frame_id_, vehicle_frame_id_);
    const double current_time = toTimeStampDouble(msg->header.stamp);
    for (const auto& rect : msg->rects) {
        tf2::Vector3 object_position_in_vehicle;
        if (!toPositionInVehicle(rect, object_position_in_vehicle)) continue;
        const auto object_position_in_odom = odom_T_vehicle * object_position_in_vehicle;
        updateOrAddObject(object_position_in_odom, current_time);
        if (debug_) unfiltered_object_msg.poses.emplace_back(toPoseMsg(object_position_in_vehicle));
    }
    deleteExpiredObjects(current_time);
    publishCorrectedPositions(header, odom_T_vehicle.inverse());
    if (debug_) unfilered_object_pub_->publish(unfiltered_object_msg);
}

bool ObjectPublisher::toPositionInVehicle(const aiformula_interfaces::msg::Rect& rect, tf2::Vector3& position) const {
    const cv::Point2f bottom_midpoint(rect.x + rect.width * 0.5, rect.y + rect.height);
    return pixelToPoint(bottom_midpoint, invert_camera_matrix_, vehicle_T_camera_, position);
}

void ObjectPublisher::updateOrAddObject(const tf2::Vector3& obj_pos, const double& current_time) {
    TrackedObject* closest_object = findClosestObject(obj_pos.x(), obj_pos.y());
    if (closest_object) {
        closest_object->update(obj_pos.x(), obj_pos.y(), current_time);
    } else {
        static unsigned int next_object_id = 0;
        tracked_objects_.emplace_back(next_object_id++, obj_pos.x(), obj_pos.y(), current_time);
    }
}

TrackedObject* ObjectPublisher::findClosestObject(const double& obj_x, const double& obj_y) {
    TrackedObject* closest_object = nullptr;
    double closest_distance_squared = std::numeric_limits<double>::max();
    const double object_separation_distance_squared = object_separation_distance_ * object_separation_distance_;

    for (auto& tracked_object : tracked_objects_) {
        const double distance_squared = tracked_object.computeDistanceSquared(obj_x, obj_y);
        if (object_separation_distance_squared < distance_squared) continue;
        if (distance_squared < closest_distance_squared) {
            closest_distance_squared = distance_squared;
            closest_object = &tracked_object;
        }
    };
    return closest_object;
}

void ObjectPublisher::deleteExpiredObjects(const double& current_time) {
    tracked_objects_.erase(
        std::remove_if(tracked_objects_.begin(), tracked_objects_.end(),
                       [current_time](const TrackedObject& obj) { return obj.isExpired(current_time); }),
        tracked_objects_.end());
}

void ObjectPublisher::publishCorrectedPositions(const std_msgs::msg::Header& header,
                                                const tf2::Transform& vehicle_T_odom) {
    geometry_msgs::msg::PoseArray pub_msg;
    pub_msg.header = header;

    for (auto& tracked_object : tracked_objects_) {
        const auto object_position_in_odom = tf2::Vector3(tracked_object.getX(), tracked_object.getY(), 0.0);
        const auto object_position_in_vehicle = vehicle_T_odom * object_position_in_odom;
        pub_msg.poses.emplace_back(toPoseMsg(object_position_in_vehicle));
    };
    object_pub_->publish(pub_msg);
    RCLCPP_INFO_ONCE(this->get_logger(), "Publish Object Position !");

    if (debug_) publishObjectIDs(pub_msg);
}

void ObjectPublisher::publishObjectIDs(const geometry_msgs::msg::PoseArray& pose_array) {
    visualization_msgs::msg::MarkerArray id_markers;

    visualization_msgs::msg::Marker delete_all_marker;
    delete_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    id_markers.markers.emplace_back(delete_all_marker);

    const double id_marker_offset_x = -0.6;  // [m]
    for (size_t idx = 0; idx < tracked_objects_.size(); ++idx) {
        tracked_objects_[idx].id_marker.header = pose_array.header;
        tracked_objects_[idx].id_marker.pose = pose_array.poses[idx];
        tracked_objects_[idx].id_marker.pose.position.x += id_marker_offset_x;
        id_markers.markers.emplace_back(tracked_objects_[idx].id_marker);
    }
    object_id_pub_->publish(id_markers);
}

}  // namespace aiformula
