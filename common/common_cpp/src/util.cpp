#include "util.hpp"

namespace aiformula {

/**
 * If `variable` is `false_value`, output `error_statement` and shutdown ros.
 *
 * @param[in] node_ptr node pointer
 * @param[in] variable Variable to check
 * @param[in] false_value Error value
 * @param[in] error_statement Output statement in case of error
 */
template <typename T>
inline void checkVariable(const rclcpp::Node* node_ptr, const T& variable, const T& false_value,
                          const std::string& error_statement) {
    if (variable == false_value) {
        RCLCPP_ERROR(node_ptr->get_logger(), "[%s] %s", __func__, error_statement.c_str());
        rclcpp::shutdown();
        exit(1);
    }
}  // checkVariable

}  // namespace aiformula
