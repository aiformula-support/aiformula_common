#ifndef GET_ROS_PARAMETER_HPP
#define GET_ROS_PARAMETER_HPP

#include <cxxabi.h>

// ROS
#include <rclcpp/rclcpp.hpp>

namespace aiformula {

class ParameterException : public std::runtime_error {
public:
    ParameterException(const std::string& message) : std::runtime_error(message) {}
};

template <typename T>
std::string getTypeName() {
    return std::string(abi::__cxa_demangle(typeid(T).name(), 0, 0, nullptr));
}

inline std::string getTypeName(const rclcpp::ParameterType& param_type) {
    if (param_type == rclcpp::ParameterType::PARAMETER_BOOL)
        return "bool";
    else if (param_type == rclcpp::ParameterType::PARAMETER_INTEGER)
        return "int";
    else if (param_type == rclcpp::ParameterType::PARAMETER_DOUBLE)
        return "double";
    else if (param_type == rclcpp::ParameterType::PARAMETER_STRING)
        return "std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >";
    else if (param_type == rclcpp::ParameterType::PARAMETER_BYTE_ARRAY)
        return "std::vector<unsigned char, std::allocator<unsigned char> >";
    else if (param_type == rclcpp::ParameterType::PARAMETER_BOOL_ARRAY)
        return "std::vector<bool, std::allocator<bool> >";
    else if (param_type == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        return "std::vector<long, std::allocator<long> >";
    else if (param_type == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
        return "std::vector<double, std::allocator<double> >";
    else if (param_type == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        return "std::vector<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, "
               "std::allocator<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > > >";
    else
        throw ParameterException("Parameter type '" + std::to_string(param_type) + "' is not supported");
}

/*
Since `as_xxx()` changes depending on the type of template arguments,
each implementation must be defined separately.
*/

// Declaration ==========================================================================================
template <typename T>
T getParameterAsType(const rclcpp::Parameter param);

// Implementation =======================================================================================
template <>
inline bool getParameterAsType<bool>(const rclcpp::Parameter param) {
    return param.as_bool();
}

template <>
inline int getParameterAsType<int>(const rclcpp::Parameter param) {
    return param.as_int();
}

template <>
inline double getParameterAsType<double>(const rclcpp::Parameter param) {
    return param.as_double();
}

template <>
inline std::string getParameterAsType<std::string>(const rclcpp::Parameter param) {
    return param.as_string();
}

template <>
inline std::vector<uint8_t> getParameterAsType<std::vector<uint8_t>>(const rclcpp::Parameter param) {
    return param.as_byte_array();
}

template <>
inline std::vector<bool> getParameterAsType<std::vector<bool>>(const rclcpp::Parameter param) {
    return param.as_bool_array();
}

template <>
inline std::vector<long int> getParameterAsType<std::vector<long int>>(const rclcpp::Parameter param) {
    return param.as_integer_array();
}

template <>
inline std::vector<double> getParameterAsType<std::vector<double>>(const rclcpp::Parameter param) {
    return param.as_double_array();
}

template <>
inline std::vector<std::string> getParameterAsType<std::vector<std::string>>(const rclcpp::Parameter param) {
    return param.as_string_array();
}

/**
 * @brief If the parameter named `param_name` has no value set, output error statement and shutdown ros.
 * If `param_name` type does not match `T`, output error statement and shutdown ros.
 *
 * @param[in] node_ptr node pointer
 * @param[in] param_name Rosparam Name
 * @return Value set for the parameter named `param_name`.
 *
 * @note Usage: `double ret = getRosParameter<double>(this, "test.value");`
 */
template <typename T>
T getRosParameter(rclcpp::Node* const node_ptr, const std::string& param_name) {
    try {
        node_ptr->declare_parameter(param_name, rclcpp::ParameterValue());

        const auto param_overrides = node_ptr->get_node_parameters_interface()->get_parameter_overrides();
        if (param_overrides.find(param_name) == param_overrides.end()) {
            throw ParameterException("The value for '" + param_name + "' has not been provided");
        }

        const rclcpp::Parameter param = node_ptr->get_parameter(param_name);
        const std::string param_type = getTypeName(param.get_type());
        const std::string template_type = getTypeName<T>();

        if (param_type == template_type) {
            return getParameterAsType<T>(param);
        } else {
            throw ParameterException("The parameter '" + param_name + "' is of type '" + param_type +
                                     "', but it is being retrieved as type '" + template_type + "'");
        }
    } catch (const ParameterException& e) {
        RCLCPP_ERROR(node_ptr->get_logger(), "%s !", e.what());
        rclcpp::shutdown();
        exit(1);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
        RCLCPP_ERROR(node_ptr->get_logger(), "%s !", e.what());
        rclcpp::shutdown();
        exit(1);
    } catch (const rclcpp::exceptions::InvalidParametersException& e) {
        RCLCPP_ERROR(node_ptr->get_logger(), "%s !");
        rclcpp::shutdown();
        exit(1);
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
        RCLCPP_ERROR(node_ptr->get_logger(), "Parameter '%s' is not declared !", e.what());
        rclcpp::shutdown();
        exit(1);
    }
}

}  // namespace aiformula

#endif  // GET_ROS_PARAMETER_HPP
