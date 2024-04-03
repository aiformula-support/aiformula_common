#include "lane_line_publisher/parametrized_polyline.hpp"

namespace aiformula {

ParametrizedPolyline::ParametrizedPolyline(const std::vector<Eigen::Vector3d> &points)
    : points_(points), cumulative_lengths_{0.0} {
    const auto last_point_it = std::prev(points_.end());
    for (auto point_it = points_.begin(); point_it != last_point_it; ++point_it) {
        const auto segment = *std::next(point_it) - *point_it;
        normalized_segments_.emplace_back(segment.normalized());
        cumulative_lengths_.emplace_back(cumulative_lengths_.back() + segment.norm());
    }
}

Eigen::Vector3d ParametrizedPolyline::pointAt(const double &length) const {
    // Handle extrapolation cases first.
    if (length < 0.0)
        return points_.front() + length * normalized_segments_.front();
    else if (length > this->length())
        return points_.back() + (length - this->length()) * normalized_segments_.back();

    // To find the segment on which the interpolated point should lie, locate the last point whose cumulative length
    // is less than `length`.
    const auto rit =
        std::lower_bound(cumulative_lengths_.rbegin(), cumulative_lengths_.rend(), length, std::greater<double>());
    const auto index = std::distance(cumulative_lengths_.begin(), std::next(rit).base());
    return points_[index] + (length - cumulative_lengths_[index]) * normalized_segments_[index];
}

void ParametrizedPolyline::pointsAt(const std::vector<double> &lengths, std::vector<Eigen::Vector3d> &points) const {
    std::transform(lengths.begin(), lengths.end(), std::back_inserter(points),
                   [this](const auto &length) { return this->pointAt(length); });
}

}  // namespace aiformula
