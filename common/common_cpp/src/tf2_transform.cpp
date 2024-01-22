#include "tf2_transform.hpp"

namespace aiformula {

/**
 * Get tf2 transformation from source to destination frame.
 *
 * @param[in] node_ptr node pointer
 * @param[in] dst_frame_id Destination frame ID.
 * @param[in] src_frame_id Source frame ID.
 * @return Tf2 transformation from source to destination frame.
 */
tf2::Transform getTf2Transform(rclcpp::Node* node_ptr, const std::string& dst_frame_id,
                               const std::string& src_frame_id) {
    static tf2_ros::Buffer tf_buffer(node_ptr->get_clock());
    static tf2_ros::TransformListener tf_listener(tf_buffer);

    // wait a little for an instance of tf2_ros::TransformListener
    static bool is_first = true;
    if (is_first && !(is_first = false)) rclcpp::sleep_for(std::chrono::milliseconds(200));

    geometry_msgs::msg::TransformStamped stamped_tf_dst_T_src;
    try {
        stamped_tf_dst_T_src =
            tf_buffer.lookupTransform(dst_frame_id, src_frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0));
    } catch (const tf2::TransformException& e) {
        RCLCPP_ERROR(node_ptr->get_logger(), "[%s] Could not transform %s to %s: %s !", __func__, dst_frame_id.c_str(),
                     src_frame_id.c_str(), e.what());
        rclcpp::shutdown();
        exit(1);
    }
    const geometry_msgs::msg::Quaternion& rotation = stamped_tf_dst_T_src.transform.rotation;
    const geometry_msgs::msg::Vector3& translation = stamped_tf_dst_T_src.transform.translation;
    tf2::Quaternion q(rotation.x, rotation.y, rotation.z, rotation.w);
    tf2::Vector3 t(translation.x, translation.y, translation.z);
    return tf2::Transform(q, t);
}  // getTf2Transform

}  // namespace aiformula
