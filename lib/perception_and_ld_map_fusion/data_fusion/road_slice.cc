
#include "lib/perception_and_ld_map_fusion/data_fusion/road_slice.h"

#include <algorithm>
#include <boost/math/distributions/normal.hpp>
#include <cmath>

#include "middleware/middleware_util.h"

namespace cem {
namespace fusion {

constexpr double RoadSlice::SLICE_DEFAULT_LENGTH;
constexpr double RoadSlice::CELL_WIDTH;
constexpr double RoadSlice::SHIFT_LIMIT;

constexpr double RoadSlice::MEAN_NORMAL_DISTRIBUTE;
constexpr double RoadSlice::STDDEV_NORMAL_DISTRIBUTE;
constexpr double RoadSlice::PROBILITY_RESOLUSION;

constexpr double RoadSlice::N_TIMES;
constexpr double RoadSlice::SIGMA_N_TIMES;
constexpr int RoadSlice::CELL_SEARCH_RADIUS;

bool RoadSlice::is_gen_normal_distribute_ = false;
std::vector<double> RoadSlice::normal_distribute_probability_;

int RoadSlice::GetProbilityLookUpTabelIndex(double mean, double value) {
  double offset = -mean + RoadSlice::MEAN_NORMAL_DISTRIBUTE;
  value += offset;
  if (value + RoadSlice::SIGMA_N_TIMES < -1e-6) {
    return 0;  // less -3σ set id of as -∞
  } else if (value - RoadSlice::SIGMA_N_TIMES > 1e-6) {
    return std::floor((2 * RoadSlice::SIGMA_N_TIMES +
                       0.5 * RoadSlice::PROBILITY_RESOLUSION) /
                      RoadSlice::PROBILITY_RESOLUSION) +
           2;  // over 3σ set id of as +∞
  }

  int ret = std::floor((value + RoadSlice::SIGMA_N_TIMES +
                        0.5 * RoadSlice::PROBILITY_RESOLUSION) /
                       RoadSlice::PROBILITY_RESOLUSION) +
            1;
  return ret;
}

void RoadSlice::GenNormalDistribute() {
  static std::once_flag flag;

  std::call_once(flag, []() {
    double mean = RoadSlice::MEAN_NORMAL_DISTRIBUTE;
    double sigma = RoadSlice::STDDEV_NORMAL_DISTRIBUTE;
    double resolution = RoadSlice::PROBILITY_RESOLUSION;
    boost::math::normal_distribution<> dist(mean, sigma);

    auto probability_array_size = RoadSlice::GetProbilityLookUpTabelIndex(
                                      mean, RoadSlice::SIGMA_N_TIMES + sigma) +
                                  1;
    normal_distribute_probability_.assign(probability_array_size, 0);

    for (auto x = -RoadSlice::SIGMA_N_TIMES - resolution;
         x <= RoadSlice::SIGMA_N_TIMES + 1.5 * resolution; x += resolution) {
      auto id = RoadSlice::GetProbilityLookUpTabelIndex(mean, x);
      normal_distribute_probability_.at(id) = boost::math::cdf(dist, x);
    }
    normal_distribute_probability_.front() = 0.0;
    normal_distribute_probability_.back() = 1.0;

    RoadSlice::is_gen_normal_distribute_ = true;
  });
}

RoadSlice::RoadSlice(Eigen::Vector2d& slice_origin_point,
                     Eigen::Vector2d& slice_direction)
    : slice_origin_point_(slice_origin_point),
      slice_direction_(slice_direction) {
  slice_direction_.normalize();
  if (RoadSlice::CELL_WIDTH < 1e-5) {
    AERROR << "Cell width must be bigger than 0";
    throw std::runtime_error("Cell width cannot be 0 or negative");
  }
  if (!RoadSlice::is_gen_normal_distribute_) {
    AERROR << "Normal distribute not initialized.";
    throw std::runtime_error("Normal distribute not initialized.");
  }
  int cell_size = 2 * (static_cast<int>(RoadSlice::SLICE_DEFAULT_LENGTH /
                                        RoadSlice::CELL_WIDTH));
  cells_.resize(cell_size);
  std::fill(cells_.begin(), cells_.end(), nullptr);
  left_first_cell_id_ = static_cast<int>(cell_size / 2);
  left_last_cell_id_ = -1;
  right_last_cell_id_ = cells_.size();
  shift_step_ = 0;
  left_road_edge_feature_involved_ = true;
  right_road_edge_feature_involved_ = true;
}

RoadSlice::~RoadSlice() {}

// void RoadSlice::FilterLanePosition() {
//   // for (auto i = right_last_cell_id_; i <= left_last_cell_id_; i++) {
//
//   // }
// }

bool RoadSlice::LeftShift() {
  if (std::abs(shift_step_ * RoadSlice::CELL_WIDTH) > RoadSlice::SHIFT_LIMIT) {
    return false;
  }
  // if (left_first_cell_id_ + shift_step_ >= left_last_cell_id_) {
  //   return false;
  // }
  shift_step_++;
  return true;
}

bool RoadSlice::RightShift() {
  if (std::abs(shift_step_ * RoadSlice::CELL_WIDTH) > RoadSlice::SHIFT_LIMIT) {
    return false;
  }
  // if (left_first_cell_id_ + shift_step_ <= right_last_cell_id_ + 1) {
  //   return false;
  // }
  shift_step_--;
  return true;
}

void RoadSlice::FillCell(Eigen::Vector2d& point, uint64_t lane_id,
                         cem::message::env_model::LineType left_line_type,
                         cem::message::env_model::LineType right_line_type,
                         LanePosition lane_position) {
  origin_points_[lane_id] = point;
  if (lane_position == LanePosition::kCloseLeftRoadEdge) {
    left_road_edge_feature_involved_ = true;
  }
  if (lane_position == LanePosition::kCloseRightRoadEdge) {
    right_road_edge_feature_involved_ = true;
  }
  auto cell_index_float = (point - slice_origin_point_).dot(slice_direction_) /
                          RoadSlice::CELL_WIDTH;
  int center_cell_relative_index = std::floor(cell_index_float);

  double mean = cell_index_float * RoadSlice::CELL_WIDTH;
  for (int i = -RoadSlice::CELL_SEARCH_RADIUS;
       i < RoadSlice::CELL_SEARCH_RADIUS; i++) {
    auto cell_index = left_first_cell_id_ + center_cell_relative_index + i;
    if (cell_index < 0 || cell_index >= cells_.size()) {
      continue;
    }

    auto smaller_boundary =
        RoadSlice::CELL_WIDTH * (center_cell_relative_index + i);
    auto bigger_boundary =
        RoadSlice::CELL_WIDTH * (center_cell_relative_index + i + 1);
    auto smaller_boundary_cfd_idx =
        RoadSlice::GetProbilityLookUpTabelIndex(mean, smaller_boundary);
    auto bigger_boundary_cfd_idx =
        RoadSlice::GetProbilityLookUpTabelIndex(mean, bigger_boundary);
    //assert(RoadSlice::normal_distribute_probability_.count(
              //  smaller_boundary_cfd_idx) == 1);
    //assert(RoadSlice::normal_distribute_probability_.count(
              //  bigger_boundary_cfd_idx) == 1);
    auto probability =
        RoadSlice::normal_distribute_probability_[bigger_boundary_cfd_idx] -
        RoadSlice::normal_distribute_probability_[smaller_boundary_cfd_idx];
    if (probability < 1e-5) {
      continue;
    }

    auto& cell_ptr = cells_.at(cell_index);
    if (cell_ptr) {
      if (cell_ptr->probability > probability) {
        continue;
      }
    } else {
      cell_ptr = std::make_unique<Cell>();
    }
    cell_ptr->lane_id = lane_id;
    cell_ptr->laneline_types.left_line_type = left_line_type;
    cell_ptr->laneline_types.right_line_type = right_line_type;
    cell_ptr->lane_position = lane_position;
    cell_ptr->probability = probability;
    cell_ptr->origin_point = point;
    if (cell_index > left_last_cell_id_) {
      left_last_cell_id_ = cell_index;
    }
    if (cell_index < right_last_cell_id_) {
      right_last_cell_id_ = cell_index;
    }
  }
}

void RoadSlice::ResetShift() { shift_step_ = 0; }

void RoadSlice::StablizeShift() {
  left_first_cell_id_ += shift_step_;
  shift_step_ = 0;
}

void RoadSlice::SetShiftStep(int step) { shift_step_ = step; }

bool RoadSlice::ClearIsolatedPoint() {
  int id_tmp = -1;
  for (int i = right_last_cell_id_; i <= left_last_cell_id_; ++i) {
    if (cells_.at(i) == nullptr) {
      continue;
    }
    if (id_tmp == -1) {
      id_tmp = cells_.at(i)->lane_id;
      continue;
    }
    if (id_tmp != cells_.at(i)->lane_id) {
      return false;
    }
  }

  std::fill(cells_.begin(), cells_.end(), nullptr);
  left_first_cell_id_ = static_cast<int>(cells_.size() / 2);
  left_last_cell_id_ = -1;
  right_last_cell_id_ = cells_.size();
  shift_step_ = 0;
  return true;
}

double RoadSlice::operator*(const RoadSlice& other) {
  double joint_probability = 0.0;

  auto lambda_calculate_cumulative_probability = [&joint_probability, &other,
                                                  this](int start_idx,
                                                        int search_size,
                                                        int direction) {
    for (int i = start_idx; i <= search_size; i++) {
      // if ((this->left_first_cell_id_ + this->shift_step_ + i * direction) >=
      //     (this->cells_.size())) {
      //   continue;
      // }

      // if ((other.left_first_cell_id_ + other.shift_step_ + i * direction) >=
      //     (other.cells_.size())) {
      //   continue;
      // }

      auto& this_cell = this->cells_.at(this->left_first_cell_id_ +
                                        this->shift_step_ + i * direction);
      auto& other_cell = other.cells_.at(other.left_first_cell_id_ +
                                         other.shift_step_ + i * direction);
      if (this_cell == nullptr || other_cell == nullptr) {
        continue;
      }
      int factor = 1;
      if (this_cell->lane_position == other_cell->lane_position) {
        if (this_cell->lane_position != LanePosition::kUncertain) {
          factor *= 2;
        }
      } else if (((this_cell->lane_position == LanePosition::kUniqueLane) &&
                  (other_cell->lane_position != LanePosition::kUncertain)) ||
                 ((other_cell->lane_position == LanePosition::kUniqueLane) &&
                  (this_cell->lane_position != LanePosition::kUncertain))) {
        factor *= 2;
      }
      int type_matched_factor = 1;
      if (this_cell->laneline_types.left_line_type ==
          other_cell->laneline_types.left_line_type) {
        if (this_cell->laneline_types.left_line_type !=
            cem::message::env_model::LineType::UNKNOWN) {
          type_matched_factor++;
        }
      }
      if (this_cell->laneline_types.right_line_type ==
          other_cell->laneline_types.right_line_type) {
        if (this_cell->laneline_types.right_line_type !=
            cem::message::env_model::LineType::UNKNOWN) {
          type_matched_factor++;
        }
      }
      factor *= type_matched_factor;
      joint_probability +=
          factor * this_cell->probability * other_cell->probability;
      // AINFO << "This cell id:" << this_cell->lane_id;
      // AINFO << "This cell prob:" << this_cell->probability;
      // AINFO << "other cell id:" << other_cell->lane_id;
      // AINFO << "other cell prob:" << other_cell->probability;
    }
  };

  auto this_last_element_idx_relative =
      this->left_last_cell_id_ - this->left_first_cell_id_ - this->shift_step_;
  auto this_first_element_idx_relative =
      this->right_last_cell_id_ - this->left_first_cell_id_ - this->shift_step_;
  auto other_last_element_idx_relative =
      other.left_last_cell_id_ - other.left_first_cell_id_ - other.shift_step_;
  auto other_first_element_idx_relative =
      other.right_last_cell_id_ - other.left_first_cell_id_ - other.shift_step_;

  auto last_element_idx_relative =
      std::min(this_last_element_idx_relative, other_last_element_idx_relative);
  auto first_element_idx_relative = std::max(this_first_element_idx_relative,
                                             other_first_element_idx_relative);
  lambda_calculate_cumulative_probability(first_element_idx_relative,
                                          last_element_idx_relative, 1);
  /*
    auto this_left_elements_size = this->left_last_cell_id_ -
                                   this->left_first_cell_id_ - this->shift_step_
    + 1;

    AINFO << "this left first cell:" << this->shift_step_;
    AINFO << "this_left_size: " << this_left_elements_size;
    AINFO << "lf first: " << (this->left_first_cell_id_ + this->shift_step_)
          << "lf last:" << this->left_last_cell_id_;
    auto other_left_elements_size = other.left_last_cell_id_ -
                                    other.left_first_cell_id_ -
                                    other.shift_step_ + 1;
    AINFO << "other left first cell:" << other.shift_step_;
    AINFO << "other_left_size: " << other_left_elements_size;
    AINFO << "lf first: " << (other.left_first_cell_id_ + other.shift_step_)
          << "lf last:" << other.left_last_cell_id_;
    // assert(this_left_elements_size >= 0);
    // assert(other_left_elements_size >= 0);
    auto left_size = std::min(this_left_elements_size,
    other_left_elements_size);

    auto this_right_elements_size =
        this->left_first_cell_id_ + this->shift_step_ -
    this->right_last_cell_id_; auto other_right_elements_size =
        other.left_first_cell_id_ + other.shift_step_ -
    other.right_last_cell_id_; AINFO << "other_left_size: " <<
    this_right_elements_size; AINFO << "other_right_size: " <<
    other_right_elements_size; AINFO << "ri first-this: "
          << (this->left_first_cell_id_ + this->shift_step_ - 1)
          << "ri last-this:" << this->right_last_cell_id_;
    AINFO << "ri first-other: "
          << (other.left_first_cell_id_ + other.shift_step_ - 1)
          << "ri last-other:" << other.right_last_cell_id_;
    //assert(this_right_elements_size >= 0);
    //assert(other_right_elements_size >= 0);
    auto right_size =
        std::min(this_right_elements_size, other_right_elements_size);

    lambda_calculate_cumulative_probability(0, left_size, 1);
    lambda_calculate_cumulative_probability(1, right_size + 1, -1);
  */

  return joint_probability;
}

std::unordered_map<std::pair<uint64_t, uint64_t>, double, PairHash>
RoadSlice::operator&(const RoadSlice& other) {
  std::unordered_map<std::pair<uint64_t, uint64_t>, double, PairHash> matchers;
  std::unordered_map<uint64_t, std::vector<uint64_t>> this_to_other;
  std::unordered_map<uint64_t, std::vector<uint64_t>> other_to_this;
  auto this_right_elements_size =
      this->left_first_cell_id_ - this->right_last_cell_id_;
  auto other_right_elements_size =
      other.left_first_cell_id_ - other.right_last_cell_id_;
  //assert(this_right_elements_size >= 0);
  //assert(other_right_elements_size >= 0);
  auto right_size =
      std::min(this_right_elements_size, other_right_elements_size);

  auto this_left_elements_size =
      this->left_last_cell_id_ - this->left_first_cell_id_ + 1;
  auto other_left_elements_size =
      other.left_last_cell_id_ - other.left_first_cell_id_ - +1;
  //assert(this_left_elements_size >= 0);
  //assert(other_left_elements_size >= 0);
  auto left_size = std::min(this_left_elements_size, other_left_elements_size);

  auto overlap_size = left_size + right_size;

  auto this_start_id = this->left_first_cell_id_ - right_size;
  auto other_start_id = other.left_first_cell_id_ - right_size;

  for (auto i = 0; i < overlap_size; ++i) {
    auto this_cell_id = this_start_id + i;
    auto other_cell_id = other_start_id + i;
    const auto& this_cell = this->GetCells().at(this_cell_id);
    const auto& other_cell = other.GetCells().at(other_cell_id);
    if (this_cell && other_cell) {
      std::pair<uint64_t, uint64_t> key;
      key.first = this_cell->lane_id;
      key.second = other_cell->lane_id;
      if (key.first == 0 || key.second == 0) {
        AWARN << "Get invalid lane id.";
        continue;
      }
      if (matchers.count(key) == 0) {
        matchers[key] =
            0.5 * (this_cell->probability + other_cell->probability);
        if (this_to_other.count(key.first) == 0) {
          this_to_other[key.first] =
              std::move(std::vector<uint64_t>(1, key.second));
        } else {
          this_to_other[key.first].push_back(key.second);
        }
        if (other_to_this.count(key.second) == 0) {
          other_to_this[key.second] =
              std::move(std::vector<uint64_t>(1, key.first));
        } else {
          other_to_this[key.second].push_back(key.first);
        }
      } else {
        matchers.at(key) +=
            0.5 * (this_cell->probability + other_cell->probability);
      }
    }
  }
  for (auto& ele : this_to_other) {
    if (ele.second.size() > 1) {
      std::sort(
          ele.second.begin(), ele.second.end(),
          [&matchers, &ele](uint64_t a, uint64_t b) {
            return matchers.at(std::pair<uint64_t, uint64_t>(ele.first, a)) >
                   matchers.at(std::pair<uint64_t, uint64_t>(ele.first, b));
          });
      for (auto iter = ele.second.begin() + 1; iter < ele.second.end();
           ++iter) {
        matchers.erase(std::pair<uint64_t, uint64_t>(ele.first, *iter));
        //assert(other_to_this.count(*iter) != 0);
        auto this_cell_tmp_id = ele.first;
        auto iter_tmp = std::find_if(
            other_to_this.at(*iter).begin(), other_to_this.at(*iter).end(),
            [this_cell_tmp_id](uint64_t this_cell_id_iter) {
              return this_cell_tmp_id == this_cell_id_iter;
            });
        if (iter_tmp != other_to_this.at(*iter).end()) {
          other_to_this.at(*iter).erase(iter_tmp);
        }
      }
    }
  }
  for (auto& ele : other_to_this) {
    if (ele.second.size() > 1) {
      std::sort(
          ele.second.begin(), ele.second.end(),
          [&matchers, &ele](uint64_t a, uint64_t b) {
            return matchers.at(std::pair<uint64_t, uint64_t>(a, ele.first)) >
                   matchers.at(std::pair<uint64_t, uint64_t>(b, ele.first));
          });
      for (auto iter = ele.second.begin() + 1; iter < ele.second.end();
           ++iter) {
        matchers.erase(std::pair<uint64_t, uint64_t>(*iter, ele.first));
      }
    }
  }
  return matchers;
}

std::string RoadSlice::GenerateSlicePrintInfo() const {
  constexpr int cell_width = 11;
  std::ostringstream oss;

  AINFO << "first: " << left_first_cell_id_;
  // draw index
  oss << "\n  cell_index ||";
  for (int i = left_last_cell_id_; i >= right_last_cell_id_; --i) {
    auto id_str = std::to_string(i);
    int padding = cell_width - 2 - id_str.length();
    if (padding > 0) {
      int left_padding = padding / 2;
      int right_padding = padding - left_padding;
      id_str = std::string(left_padding, ' ') + id_str;
      id_str = id_str + std::string(right_padding, ' ');
    }
    oss << std::setw(cell_width - 2) << id_str << "|";
  }
  oss << "\n";

  // draw id
  oss << "   lane_id   ||";
  for (int i = left_last_cell_id_; i >= right_last_cell_id_; --i) {
    const auto& cell_ptr = cells_.at(i);
    if (cell_ptr) {
      auto id_str = std::to_string(cell_ptr->lane_id);
      int padding = cell_width - 2 - id_str.length();
      if (padding > 0) {
        int left_padding = padding / 2;
        int right_padding = padding - left_padding;
        id_str = std::string(left_padding, ' ') + id_str;
        id_str = id_str + std::string(right_padding, ' ');
      }
      oss << std::setw(cell_width - 2) << id_str << "|";
    } else {
      oss << std::setw(cell_width - 2) << " " << "|";
    }
  }
  oss << "\n";

  // draw lane position
  oss << "lane_position||";
  for (int i = left_last_cell_id_; i >= right_last_cell_id_; --i) {
    const auto& cell_ptr = cells_.at(i);
    if (cell_ptr) {
      oss << std::setw(cell_width - 6)
          << static_cast<int>(cell_ptr->lane_position) << std::setw(4) << " "
          << "|";
    } else {
      oss << std::setw(cell_width - 2) << " " << "|";
    }
  }
  oss << "\n";

  // draw laneline type
  oss << "  line_type  ||";
  for (int i = left_last_cell_id_; i >= right_last_cell_id_; --i) {
    const auto& cell_ptr = cells_.at(i);
    if (cell_ptr) {
      oss << static_cast<int>(cell_ptr->laneline_types.left_line_type)
          << std::setw(cell_width - 3)
          << static_cast<int>(cell_ptr->laneline_types.right_line_type) << "|";
    } else {
      oss << std::setw(cell_width - 2) << " " << "|";
    }
  }
  oss << "\n";

  // draw probability
  oss << " probability ||";
  for (int i = left_last_cell_id_; i >= right_last_cell_id_; --i) {
    const auto& cell_ptr = cells_.at(i);
    if (cell_ptr) {
      oss << std::setw(cell_width - 4) << std::fixed << std::setprecision(3)
          << cell_ptr->probability << std::setw(2) << " " << "|";
    } else {
      oss << std::setw(cell_width - 2) << " " << "|";
    }
  }
  oss << "\n";

  // draw intsect_pt

  oss << "   point_x   ||";
  for (int i = left_last_cell_id_; i >= right_last_cell_id_; --i) {
    const auto& cell_ptr = cells_.at(i);
    if (cell_ptr) {
      std::ostringstream ostr;
      ostr << std::fixed << std::setprecision(3) << cell_ptr->origin_point.x();
      std::string ptx_str_s = ostr.str();
      int padding = cell_width - 2 - ptx_str_s.length();
      if (padding > 0) {
        int left_padding = padding / 2;
        int right_padding = padding - left_padding;
        ptx_str_s = std::string(left_padding, ' ') + ptx_str_s;
        ptx_str_s = ptx_str_s + std::string(right_padding, ' ');
      }
      oss << std::setw(cell_width - 2) << ptx_str_s << "|";
    } else {
      oss << std::setw(cell_width - 2) << " " << "|";
    }
  }
  oss << "\n";

  oss << "   point_y   ||";
  for (int i = left_last_cell_id_; i >= right_last_cell_id_; --i) {
    const auto& cell_ptr = cells_.at(i);
    if (cell_ptr) {
      oss << std::setw(cell_width - 4) << std::fixed << std::setprecision(3)
          << cell_ptr->origin_point.y() << std::setw(2) << " " << "|";
    } else {
      oss << std::setw(cell_width - 2) << " " << "|";
    }
  }
  oss << "\n";

  return oss.str();
}

bool RoadSlice::IsLaneAatTheLeftofLaneB(uint64_t a, uint64_t b) {
  int a_id = -1;
  int b_id = -1;
  for (int i = right_last_cell_id_; i <= left_last_cell_id_; ++i) {
    if (cells_.at(i) == nullptr) {
      continue;
    }
    if (a_id < 0) {
      if (cells_.at(i)->lane_id == a) {
        a_id = i;
        continue;
      }
    }
    if (b_id < 0) {
      if (cells_.at(i)->lane_id == b) {
        b_id = i;
        continue;
      }
    }
    if ((a_id >= 0) && (b_id >= 0)) {
      break;
    }
  }
  if ((a_id >= 0) && (b_id >= 0) && (a_id > b_id)) {
    return true;
  } else {
    return false;
  }
}

bool RoadSlice::IsLaneAatTheRightofLaneB(uint64_t a, uint64_t b) {
  int a_id = -1;
  int b_id = -1;
  for (int i = right_last_cell_id_; i <= left_last_cell_id_; ++i) {
    if (cells_.at(i) == nullptr) {
      continue;
    }
    if (a_id < 0) {
      if (cells_.at(i)->lane_id == a) {
        a_id = i;
        continue;
      }
    }
    if (b_id < 0) {
      if (cells_.at(i)->lane_id == b) {
        b_id = i;
        continue;
      }
    }
    if ((a_id >= 0) && (b_id >= 0)) {
      break;
    }
  }
  if ((a_id >= 0) && (b_id >= 0) && (a_id < b_id)) {
    return true;
  } else {
    return false;
  }
}

bool RoadSlice::HasRoadedgeFeature() {
  return left_road_edge_feature_involved_ || right_road_edge_feature_involved_;
}

}  // namespace fusion
}  // namespace cem
