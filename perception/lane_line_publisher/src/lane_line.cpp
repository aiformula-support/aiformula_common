#include "lane_line_publisher/lane_line.hpp"

namespace aiformula {

void LaneLine::toVehicleFrame(const cv::Mat& camera_matrix, const tf2::Transform& vehicle_T_camera) {
    std::vector<cv::Point2f> pixels_f(pixels.begin(), pixels.end());
    const auto tf_points = pixelsToPoints(pixels_f, camera_matrix, vehicle_T_camera);
    for (const auto& tf_point : tf_points) points.emplace_back(tf_point.x(), tf_point.y(), tf_point.z());
}

void LaneLine::cropToRoi(const double& xmin, const double& xmax, const double& ymin, const double& ymax) {
    std::vector<Eigen::Vector3d> cropped;
    const auto num_points = static_cast<int>(points.size());
    for (int i = 0; i < num_points; ++i) {
        const auto& point = points[i];
        if (point.x() >= xmin && point.x() <= xmax && point.y() >= ymin && point.y() <= ymax)
            cropped.emplace_back(point);
    }
    points = cropped;
}

void LaneLine::respacePoints(const double& spacing) {
    if (points.size() < 2) return;

    const ParametrizedPolyline polyline(points);
    std::vector<double> lengths;
    for (double length = 0.0; length < polyline.length(); length += spacing) lengths.emplace_back(length);
    std::vector<Eigen::Vector3d> respaced;
    polyline.pointsAt(lengths, respaced);
    points = respaced;
}

void LaneLine::fitPoints(const CubicLineFitter::ConstPtr& fitter) {
    std::vector<Eigen::Vector3d> extrapolated;
    fitter->fitAndExtrapolate(points, extrapolated);
    points = extrapolated;
}

}  // namespace aiformula
