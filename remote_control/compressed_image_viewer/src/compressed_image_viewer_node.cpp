#include "compressed_image_viewer/compressed_image_viewer.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<aiformula::CompressedImageViewer>());
    rclcpp::shutdown();
    return 0;
}
