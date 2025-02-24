#include "object_publisher/tracked_object.hpp"

namespace aiformula {

rclcpp::Node* TrackedObject::node_ptr_ = nullptr;
double TrackedObject::process_noise_variance_ = 1e-4;
double TrackedObject::measurement_noise_variance_ = 1e-2;
double TrackedObject::initial_error_covariance_ = 1e-0;
double TrackedObject::expiration_duration_ = 3.0;

void TrackedObject::initStaticMembers(rclcpp::Node* const node_ptr) {
    // From 'tracked_object.yaml'
    node_ptr_ = node_ptr;
    process_noise_variance_ = getRosParameter<double>(node_ptr_, "process_noise_variance");
    measurement_noise_variance_ = getRosParameter<double>(node_ptr_, "measurement_noise_variance");
    initial_error_covariance_ = getRosParameter<double>(node_ptr_, "initial_error_covariance");
    expiration_duration_ = getRosParameter<double>(node_ptr_, "expiration_duration");
}

void TrackedObject::printStaticMembers() {
    std::ostringstream log_stream;
    const auto& formatter = createFormatter(30);
    log_stream << "\n[" << __func__ << " (TrackedObject)] " << std::string(33, '=') << "\n"
               << "(tracked_object.yaml)\n"
               << std::fixed << std::setprecision(1) << std::scientific << formatter("process_noise_variance_")
               << process_noise_variance_ << "\n"
               << formatter("measurement_noise_variance_") << measurement_noise_variance_ << "\n"
               << formatter("initial_error_covariance_") << initial_error_covariance_ << "\n"
               << std::fixed << formatter("expiration_duration_") << expiration_duration_ << " [sec]\n"
               << std::string(70, '=') << "\n";
    RCLCPP_INFO(node_ptr_->get_logger(), "%s", log_stream.str().c_str());
}

TrackedObject::TrackedObject(const int& id, const float& initial_x, const float& initial_y, const double& timestamp)
    : kf_(cv::KalmanFilter(2, 2, 0)), id_(id), last_seen_time_(timestamp) {
    cv::setIdentity(kf_.transitionMatrix);
    cv::setIdentity(kf_.measurementMatrix);

    cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(process_noise_variance_));
    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(measurement_noise_variance_));
    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(initial_error_covariance_));

    kf_.statePost.at<float>(0) = initial_x;
    kf_.statePost.at<float>(1) = initial_y;
    generateMarker();
}

void TrackedObject::generateMarker() {
    this->id_marker.ns = "tracked_objects/id";
    this->id_marker.id = id_;
    this->id_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    this->id_marker.action = visualization_msgs::msg::Marker::ADD;
    this->id_marker.text = std::to_string(id_);

    this->id_marker.scale.x = 1.0;
    this->id_marker.scale.y = 1.0;
    this->id_marker.scale.z = 1.0;

    this->id_marker.color.r = 1.0;
    this->id_marker.color.g = 1.0;
    this->id_marker.color.b = 1.0;
    this->id_marker.color.a = 1.0;
}

float TrackedObject::computeDistanceSquared(const float& x_in, const float& y_in) const {
    const int obj_x = getX();
    const int obj_y = getY();
    return (obj_x - x_in) * (obj_x - x_in) + (obj_y - y_in) * (obj_y - y_in);
}

void TrackedObject::update(const float& x_in, const float& y_in, const double& current_time) {
    cv::Mat measurement = (cv::Mat_<float>(2, 1) << x_in, y_in);
    kf_.predict();
    kf_.correct(measurement);
    last_seen_time_ = current_time;
}

bool TrackedObject::isExpired(const double& current_time) const {
    return current_time - last_seen_time_ > expiration_duration_;
}

}  // namespace aiformula
