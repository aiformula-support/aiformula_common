#include "compressed_image_viewer/compressed_image_viewer.hpp"

namespace aiformula {

CompressedImageViewer::CompressedImageViewer()
    : Node("compressed_image_viewer"),
      display_full_screen_(true),
      target_screen_idx_(0),
      display_scale_setting_(100),
      window_width_ratio_(0.5),
      window_position_ratio_(cv::Point2d(0.0, 0.0)),
      window_name_("AI Formula Pilot"),
      screens_info_(NULL) {
    getRosParams();
    initValues();
    printParam();
}

void CompressedImageViewer::getRosParams() {
    // compressed_image_viewer.yaml
    display_full_screen_ = getRosParameter<bool>(this, "display_full_screen");

    if (!display_full_screen_) {
        target_screen_idx_ = getRosParameter<int>(this, "target_screen_idx");
        display_scale_setting_ = getRosParameter<int>(this, "display_scale_setting");
        window_width_ratio_ = getRosParameter<double>(this, "window.width_ratio");
        window_position_ratio_.x = getRosParameter<double>(this, "window.position_ratio.x");
        window_position_ratio_.y = getRosParameter<double>(this, "window.position_ratio.y");

        if (target_screen_idx_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "'target_screen_idx_'(=%d) must be greater than or equal to 0",
                         target_screen_idx_);
            rclcpp::shutdown();
            exit(1);
        }
        if (window_width_ratio_ <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "'window_width_ratio_'(%.2lf) must be greater than 0.0",
                         window_width_ratio_);
            rclcpp::shutdown();
            exit(1);
        }
    }
}

void CompressedImageViewer::initValues() {
    // Subscriber
    const int buffer_size = 10;
    compressed_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "sub_compressed_image", buffer_size,
        std::bind(&CompressedImageViewer::compressedImageCallback, this, std::placeholders::_1));

    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    if (display_full_screen_) {
        cv::setWindowProperty(window_name_, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        /* In OpenCV 4.2.0, 'cv::setWindowProperty()' makes the window full screen,
           but the displayed image is not scaled to full screen.
           By executing 'cv::resizeWindow()', even just as a formality,
           the displayed image is scaled to full screen (the reason is unknown). */
        cv::resizeWindow(window_name_, cv::Size());
    } else {
        getScreenInfo();
    }
}

void CompressedImageViewer::getScreenInfo() {
    Display* display = NULL;
    try {
        display = XOpenDisplay(NULL);
        if (display == NULL) throw std::logic_error("Cannot connect to any displays.");
        // Check if Xinerama is available
        int event_base, error_base;
        if (!XineramaQueryExtension(display, &event_base, &error_base)) {
            throw std::logic_error("Xinerama extension is not available.");
        }
        // Check if Xinerama is active
        if (!XineramaIsActive(display)) {
            throw std::logic_error("Xinerama is not active.");
        }
        // Get information about screens
        int num_screens;
        screens_info_ = XineramaQueryScreens(display, &num_screens);
        if (screens_info_ == NULL) {
            throw std::logic_error("Failed to query screens.");
        }
        if (target_screen_idx_ >= num_screens) {
            std::ostringstream error_ss;
            error_ss << "'target_screen_idx_'(=" << target_screen_idx_ << ") must be an integer between 0 and "
                     << num_screens - 1 << ".";
            throw std::logic_error(error_ss.str());
        }
        XCloseDisplay(display);
    } catch (const std::exception& e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Error: " << e.what());
        if (display) XCloseDisplay(display);
        rclcpp::shutdown();
        exit(1);
    }
}

void CompressedImageViewer::printParam() const {
    RCLCPP_INFO(this->get_logger(), "[%s] ===============", __func__);
    RCLCPP_INFO(this->get_logger(), "(compressed_image_viewer.yaml)");
    RCLCPP_INFO(this->get_logger(), "  display_full_screen_   : %s", display_full_screen_ ? "true" : "else");
    RCLCPP_INFO(this->get_logger(), "  target_screen_idx_     : %d", target_screen_idx_);
    RCLCPP_INFO(this->get_logger(), "  display_scale_setting_ : %d", display_scale_setting_);
    RCLCPP_INFO(this->get_logger(), "  window_width_ratio_    : %.2lf", window_width_ratio_);
    RCLCPP_INFO(this->get_logger(), "  window_position_ratio_ : (%.2lf, %.2lf)", window_position_ratio_.x,
                window_position_ratio_.y);
    RCLCPP_INFO(this->get_logger(), "============================\n");
}

void CompressedImageViewer::compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) const {
    RCLCPP_INFO_ONCE(this->get_logger(), "Subscribe Compressed Image !");
    try {
        const auto decoded_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

        static bool is_first = true;
        if (!display_full_screen_ && is_first && !(is_first = false)) {
            setWindowSizeAndPosition(decoded_image.size());
            XFree(screens_info_);
        }

        cv::imshow(window_name_, decoded_image);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "[%s] cv_bridge exception: %s", __func__, e.what());
        return;
    }
}

void CompressedImageViewer::setWindowSizeAndPosition(const cv::Size2i& image_size) const {
    const auto& screen = screens_info_[target_screen_idx_];
    const double display_scale_factor = 100. / display_scale_setting_;
    const cv::Size2i screen_size(static_cast<int>(screen.width * display_scale_factor),
                                 static_cast<int>(screen.height * display_scale_factor));
    const cv::Point2i screen_org(static_cast<int>(screen.x_org * display_scale_factor),
                                 static_cast<int>(screen.y_org * display_scale_factor));
    const int window_width = static_cast<int>(screen_size.width * window_width_ratio_);
    const cv::Size2i window_size(
        window_width, static_cast<int>(static_cast<double>(window_width) * image_size.height / image_size.width));
    cv::resizeWindow(window_name_, window_size);

    const int window_x = screen_org.x + static_cast<int>((screen_size - window_size).width * window_position_ratio_.x);
    const int window_y = screen_org.y + static_cast<int>((screen_size - window_size).height * window_position_ratio_.y);
    cv::moveWindow(window_name_, window_x, window_y);
}

}  // namespace aiformula
