#ifndef COMPRESSED_IMAGE_VIEWER_EXCEPTION_HPP
#define COMPRESSED_IMAGE_VIEWER_EXCEPTION_HPP

// Original
#include "common_cpp/util.hpp"

namespace aiformula {

class ScreenIndexException : public std::runtime_error {
public:
    ScreenIndexException(const int& target_screen_idx)
        : std::runtime_error("'target_screen_idx_'(=" + std::to_string(target_screen_idx) +
                             ") must be greater than or equal to 0.") {}
    ScreenIndexException(const int& target_screen_idx, const int& num_monitors)
        : std::runtime_error("'target_screen_idx_'(=" + std::to_string(target_screen_idx) +
                             ") must be an integer between 0 and " + std::to_string(num_monitors - 1) + ".") {}
};

class WindowWidthRatioException : public std::runtime_error {
public:
    WindowWidthRatioException(const double& window_width_ratio)
        : std::runtime_error("'window_width_ratio_'(=" + std::to_string(window_width_ratio) +
                             ") must be greater than 0.0.") {}
};

}  // namespace aiformula

#endif  // COMPRESSED_IMAGE_VIEWER_EXCEPTION_HPP
