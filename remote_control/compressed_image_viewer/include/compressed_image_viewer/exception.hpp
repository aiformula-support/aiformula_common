#ifndef COMPRESSED_IMAGE_VIEWER_EXCEPTION_HPP
#define COMPRESSED_IMAGE_VIEWER_EXCEPTION_HPP

// Original
#include "common_cpp/util.hpp"

namespace aiformula {

class ScreenIndexException : public BasicException {
public:
    ScreenIndexException(const int& target_screen_idx)
        : BasicException("'target_screen_idx_'(=" + std::to_string(target_screen_idx) +
                         ") must be greater than or equal to 0.") {}
    ScreenIndexException(const int& target_screen_idx, const int& num_screens)
        : BasicException("'target_screen_idx_'(=" + std::to_string(target_screen_idx) +
                         ") must be an integer between 0 and " + std::to_string(num_screens - 1) + ".") {}
};

class DisplayScaleException : public BasicException {
public:
    DisplayScaleException(const int& display_scale_setting)
        : BasicException("'display_scale_setting_'(=" + std::to_string(display_scale_setting) +
                         ") must be greater than 0.") {}
};

class WindowWidthRatioException : public BasicException {
public:
    WindowWidthRatioException(const double& window_width_ratio)
        : BasicException("'window_width_ratio_'(=" + std::to_string(window_width_ratio) +
                         ") must be greater than 0.0.") {}
};

class X11Exception : public BasicException {
public:
    X11Exception(const std::string& error_message) : BasicException("Cannot connect to any displays.") {}
};

}  // namespace aiformula

#endif  // COMPRESSED_IMAGE_VIEWER_EXCEPTION_HPP
