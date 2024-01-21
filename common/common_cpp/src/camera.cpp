#include "camera.hpp"

namespace aiformula {

/**
 * Get Camera Parameter from rosparam.
 *
 * @param[in] node_ptr node pointer
 * @param[in] camera_matrix Camera Matris
 * @param[in] image_size Image Size
 */
void getCameraParams(rclcpp::Node* node_ptr, cv::Mat& camera_matrix, cv::Size* image_size) {
    camera_matrix = cv::Mat::eye(3, 3, CV_32F);
    camera_matrix.at<float>(0, 0) = getRosParameter<double>(node_ptr, "focal_length.x");
    camera_matrix.at<float>(1, 1) = getRosParameter<double>(node_ptr, "focal_length.y");
    camera_matrix.at<float>(0, 2) = getRosParameter<double>(node_ptr, "center_point.x");
    camera_matrix.at<float>(1, 2) = getRosParameter<double>(node_ptr, "center_point.y");

    if (image_size) {
        image_size->width = getRosParameter<int>(node_ptr, "size.width");
        image_size->height = getRosParameter<int>(node_ptr, "size.height");
    }
}  // getCameraParams

}  // namespace aiformula
