#ifndef GET_ROS_PARAMETER_HPP
#define GET_ROS_PARAMETER_HPP

// ROS
#include <rclcpp/rclcpp.hpp>

namespace aiformula {

template <typename T>
T getParameterAsType(rclcpp::Node* node_ptr, const std::string& param_name);

template <>
inline bool getParameterAsType<bool>(rclcpp::Node* node_ptr, const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_bool();
}  // getParameterAsType<bool>

template <>
inline int getParameterAsType<int>(rclcpp::Node* node_ptr, const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_int();
}  // getParameterAsType<int>

template <>
inline double getParameterAsType<double>(rclcpp::Node* node_ptr, const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_double();
}  // getParameterAsType<double>

template <>
inline std::string getParameterAsType<std::string>(rclcpp::Node* node_ptr, const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_string();
}  // getParameterAsType<std::string>

template <>
inline std::vector<bool> getParameterAsType<std::vector<bool>>(rclcpp::Node* node_ptr, const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_bool_array();
}  // getParameterAsType<std::vector<bool>>

template <>
inline std::vector<long int> getParameterAsType<std::vector<long int>>(rclcpp::Node* node_ptr,
                                                                       const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_integer_array();
}  // getParameterAsType<std::vector<long int>>

template <>
inline std::vector<double> getParameterAsType<std::vector<double>>(rclcpp::Node* node_ptr,
                                                                   const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_double_array();
}  // getParameterAsType<std::vector<double>>

template <>
inline std::vector<std::string> getParameterAsType<std::vector<std::string>>(rclcpp::Node* node_ptr,
                                                                             const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_string_array();
}  // getParameterAsType<std::vector<std::string>>

/**
 * If `param_name` is not declared, output error statement and shutdown ros.
 * If `param_name` type is different from `T`, output error statement and shutdown ros.
 *
 * @param[in] node_ptr node pointer
 * @param[in] param_name Rosparam Name
 */
template <typename T>
T getRosParameter(rclcpp::Node* node_ptr, const std::string& param_name) {
    try {
        if (!node_ptr->has_parameter(param_name)) {
            node_ptr->declare_parameter(param_name);
        }
        return getParameterAsType<T>(node_ptr, param_name);
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
        RCLCPP_ERROR(node_ptr->get_logger(), "Parameter '%s' is not declared", e.what());
        rclcpp::shutdown();
        exit(1);
    } catch (const rclcpp::ParameterTypeException& e) {
        RCLCPP_ERROR(node_ptr->get_logger(), "Parameter '%s' has an incorrect type: %s", param_name.c_str(), e.what());
        rclcpp::shutdown();
        exit(1);
    }
}  // getRosParameter

}  // namespace aiformula

#endif  // GET_ROS_PARAMETER_HPP
