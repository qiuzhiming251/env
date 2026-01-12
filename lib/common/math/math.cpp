#include "math.h"
#include <cstddef>
#include "modules/common/math/line_segment2d.h"

namespace cem {
namespace fusion {

std::tuple<float, float, float> CalculateCurvesOverlapDist(
    const Curve& first_curve, const Curve& second_curve)
{
    float start_overlap_dist = std::numeric_limits<float>::max();
    float middle_overlap_dist = std::numeric_limits<float>::max();
    float end_overlap_dist = std::numeric_limits<float>::max();

    float farther_start =
        std::max(first_curve.lon_dist_start, second_curve.lon_dist_start);
    float closer_end =
        std::min(first_curve.lon_dist_end, second_curve.lon_dist_end);

    if (closer_end >= farther_start)
    {
        float farther_start_first_y =
            CalculateY(first_curve.c0, first_curve.c1, first_curve.c2,
                       first_curve.c3, farther_start);
        float farther_start_second_y =
            CalculateY(second_curve.c0, second_curve.c1, second_curve.c2,
                       second_curve.c3, farther_start);
        start_overlap_dist =
            std::fabs(farther_start_first_y - farther_start_second_y);

        float anchor_point_x = 0.5f * (farther_start + closer_end);
        float anchor_point_first_y =
            CalculateY(first_curve.c0, first_curve.c1, first_curve.c2,
                       first_curve.c3, anchor_point_x);
        float anchor_point_second_y =
            CalculateY(second_curve.c0, second_curve.c1, second_curve.c2,
                       second_curve.c3, anchor_point_x);
        middle_overlap_dist =
            std::fabs(anchor_point_first_y - anchor_point_second_y);

        float closer_end_first_y =
            CalculateY(first_curve.c0, first_curve.c1, first_curve.c2,
                       first_curve.c3, closer_end);
        float closer_end_second_y =
            CalculateY(second_curve.c0, second_curve.c1, second_curve.c2,
                       second_curve.c3, closer_end);
        end_overlap_dist = std::fabs(closer_end_first_y - closer_end_second_y);
    }

    return {start_overlap_dist, middle_overlap_dist, end_overlap_dist};
}

void MultiLineSegment::Init(const std::vector<Vec2d> &points) {
    path_points_.assign(points.begin(), points.end());
    num_points_ = static_cast<int>(path_points_.size());
    // CHECK_GE(num_points_, 2);

    accumulated_s_.clear();
    accumulated_s_.reserve(num_points_);
    segments_.clear();
    segments_.reserve(num_points_ - 1);
    double s = 0.0;
    for (int i = 0; i < num_points_; ++i) {
        accumulated_s_.emplace_back(s);
        Vec2d heading;
        if (i + 1 >= num_points_) {
          heading = path_points_[i] - path_points_[i - 1];
        } else {
          segments_.emplace_back(path_points_[i], path_points_[i + 1]);
          heading = path_points_[i + 1] - path_points_[i];
          // TODO(All): use heading.length when all adjacent lanes are guarantee to
          // be connected.
          s += heading.Length();
        }
    }
    num_segments_ = num_points_ - 1;
    length_       = s;
    // CHECK_EQ(accumulated_s_.size(), static_cast<size_t>(num_points_));
    // CHECK_EQ(segments_.size(), static_cast<size_t>(num_segments_));
}

bool MultiLineSegment::GetProjection(const Vec2d &point,  // NOLINT
                                     double *accumulate_s, double *lateral, double *min_distance, int *index_min, double radius1d,
                                     int index_center, std::string *const str_info) const {
    if (segments_.empty()) {
        return false;
    }
    if (accumulate_s == nullptr || lateral == nullptr || min_distance == nullptr) {
        return false;
    }
    if (str_info != nullptr) {
        *str_info = "";
    }

    // CHECK_GE(num_points_, 2);
    int min_index = 0;
    if (num_points_ == 2) {
        LineSegment2d segment_min_1 = segments_[0];
        *min_distance               = sqrt(segment_min_1.DistanceSquareTo(point));
        if (str_info != nullptr) {
          *str_info += "  min_dis:" + std::to_string(*min_distance);
        }
    } else {
        *min_distance = std::numeric_limits<double>::infinity();

        double distance_temp_min = segments_[0].DistanceSquareTo(point);
        if (str_info != nullptr) {
          *str_info += "  distance_temp_min:" + std::to_string(distance_temp_min);
        }
        // distance_temp_min = std::numeric_limits<double>::infinity();
        LineSegment2d segment_min_1 = segments_[0];
        LineSegment2d segment_min_2 = segments_[1];

        double distance    = 0;
        size_t start_index = 1;
        size_t end_index   = std::numeric_limits<int>::max();
        if (radius1d >= 0 && index_center >= 0 && index_center < accumulated_s_.size()) {
          double center_s = accumulated_s_.at(index_center);
          double start_s  = fmax(center_s - radius1d, accumulated_s_.front());
          double end_s    = fmin(center_s + radius1d, accumulated_s_.back());
          start_index = std::lower_bound(accumulated_s_.begin(), accumulated_s_.begin() + index_center, start_s) - accumulated_s_.begin();
          end_index   = std::upper_bound(accumulated_s_.begin() + index_center, accumulated_s_.end(), end_s) - accumulated_s_.begin();
          start_index     = byd::common::math::Clamp(start_index, static_cast<size_t>(0), segments_.size() - 1);
          end_index       = byd::common::math::Clamp(end_index, static_cast<size_t>(0), segments_.size() - 1);
        }
        // bool flag = false;
        for (size_t i = start_index; i < num_segments_ && i <= end_index; ++i) {
          distance = segments_[i].DistanceSquareTo(point);
          if (distance < distance_temp_min) {
            // flag = true;
            min_index         = static_cast<int>(i);
            distance_temp_min = distance;
          }
        }
        if (str_info != nullptr) {
          *str_info += "  start_index:" + std::to_string(start_index) + "  end_index:" + std::to_string(end_index);
        }

        *min_distance = std::sqrt(distance_temp_min);
    }
    if (index_min != nullptr) {
        *index_min = min_index;
    }
    const auto &nearest_seg = segments_[min_index];
    const auto  prod        = nearest_seg.ProductOntoUnit(point);
    const auto  proj        = nearest_seg.ProjectOntoUnit(point);
    if (str_info != nullptr) {
        *str_info += "  num_segment:" + std::to_string(num_segments_) + "  min_index:" + std::to_string(min_index) +
                     "  prod:" + std::to_string(prod) + "  proj:" + std::to_string(proj);
    }

    if (num_points_ == 2) {
        *accumulate_s = proj;
        *lateral = prod;
        return true;
    }

    if (min_index == 0) {
        *accumulate_s = std::min(proj, nearest_seg.length());
        if (proj < 0) {
          *lateral = prod;
        } else {
          *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
        }
    } else if (min_index == num_segments_ - 1) {
        *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
        if (proj > 0) {
          *lateral = prod;
        } else {
          *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
        }
    } else {
        *accumulate_s = accumulated_s_[min_index] + std::max(0.0, std::min(proj, nearest_seg.length()));
        *lateral      = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
    return true;
}

std::pair<Vec2d, double> MultiLineSegment::GetPointAtDistance(double dis_spec) const {
    std::pair<Vec2d, double> rev;
    if (path_points_.size() <= 1) {
        return rev;
    }
    if (dis_spec <= 0.0) {
        byd::common::math::LineSegment2d seg(path_points_[0], path_points_[1]);
        return std::make_pair(path_points_[0], seg.heading());
    }
    if (dis_spec >= length_) {
        std::size_t                      len = path_points_.size();
        byd::common::math::LineSegment2d seg(path_points_[len - 2], path_points_[len - 1]);
        return std::make_pair(path_points_[len - 1], seg.heading());
    }

    auto it = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), dis_spec);
    if (it == accumulated_s_.begin()) {
        byd::common::math::LineSegment2d seg(path_points_[0], path_points_[1]);
        return std::make_pair(path_points_[0], seg.heading());
    }

    const int seg_index = std::distance(accumulated_s_.begin(), it) - 1;

    const double seg_start_s = accumulated_s_[seg_index];
    const double seg_end_s   = accumulated_s_[seg_index + 1];
    const double seg_length  = seg_end_s - seg_start_s;
    const double ratio       = (dis_spec - seg_start_s) / seg_length;

    const Vec2d &start_point = path_points_[seg_index];
    const Vec2d &end_point   = path_points_[seg_index + 1];
    byd::common::math::LineSegment2d seg(start_point, end_point);

    return std::make_pair(start_point + (end_point - start_point) * ratio, seg.heading());
}

} // namespace fusion
} // namespace cem
