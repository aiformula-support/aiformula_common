#include "compressed_image_viewer/compressed_image_viewer.hpp"

namespace aiformula {

CompressedImageViewer::CompressedImageViewer() : Node("compressed_image_viewer") {
    // Subscriber
    const int buffer_size = 10;
    compressed_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "sub_compressed_image", buffer_size,
        std::bind(&CompressedImageViewer::compressedImageCallback, this, std::placeholders::_1));
}

void CompressedImageViewer::compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) const {
    RCLCPP_INFO_ONCE(this->get_logger(), "Subscribe Compressed Image !");
    try {
        const auto decoded_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        cv::imshow("Compressed Image", decoded_image);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "[%s] cv_bridge exception: %s", __func__, e.what());
        return;
    }
}

}  // namespace aiformula
