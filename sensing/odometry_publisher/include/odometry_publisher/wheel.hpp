#ifndef WHEEL_HPP
#define WHEEL_HPP

namespace aiformula {
namespace odometry_publisher {

template <typename T>
struct WheelRates {
    WheelRates(const T& left, const T& right) : left(left), right(right) {}
    WheelRates<double> operator*(const double& rhs) const { return {left * rhs, right * rhs}; }
    T left, right;
};
const double MINUTE_TO_SECOND = 0.016667;  // = 1/60
const double DEGREE_TO_RADIAN = M_PI / 180.0;
const double RADIAN_TO_DEGREE = 180.0 / M_PI;

const int RPM_ID = 1809;

}  // namespace odometry_publisher
}  // namespace aiformula

#endif  // WHEEL_HPP
