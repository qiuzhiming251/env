#if defined __aarch64__
#include <arm_neon.h>
#endif

#include <Eigen/Geometry>

#include "bev_map_pre_processor.h"

#include "lib/common/utility.h"

#include "common/utils/GeoMathUtil.h"

// #define USE_LIDAR_CLUSTER_ROAD_EDGE_ONLY

namespace cem {
namespace fusion {
const size_t   MSG_BUFFER_MAX_SIZE                = 50;
const int      FREESPACE_RESOLUTION               = 1;  // deg
const uint32_t kEgoLaneTrackFrame                 = 5;
const double   kEgoLaneFrontPointMaxX             = 3;
const double   kEgoLaneEndPointMinX               = 20;
const double   BevMapPreProcessor::DIST_THRESHOLD = 0.2 * 0.2;
const double   kLaneRemovalRange                  = 20;
struct AngleRange {
  bool  valid;
  float start_angle;
  float end_angle;
};

static AngleRange CalcClusterAngleRange(const std::map<uint32_t, std::vector<RoadEdgeInfo>> &cluster) {
  AngleRange range;
  range.valid = false;

  std::vector<float> angles;
  for (const auto &[seq, edges] : cluster) {
    for (const auto &edge : edges) {
      for (const auto &pt : edge.edge.line_points) {
        float angle = std::atan2(pt.y, pt.x) * 180.0F / M_PIf32;
        if (angle < 0) {
          angle += 360.0F;
        }
        angle = angle > 359.9999F ? 359.9999 : angle;
        angle = angle < 0.0F ? 0.0F : angle;
        angles.push_back(angle);
      }
    }
  }
  if (!angles.empty()) {
    std::sort(angles.begin(), angles.end());

    float max_gap       = 0.0F;
    int   max_gap_index = -1;

    for (size_t i = 0; i < angles.size(); ++i) {
      float a1 = angles[i];
      float a2 = angles[(i + 1) % angles.size()];  // wrap around

      float gap = (i + 1 == angles.size()) ? (angles[0] + 360.0F - a1) : (a2 - a1);
      if (gap > max_gap) {
        max_gap       = gap;
        max_gap_index = i;
      }
    }

    range.valid       = true;
    range.start_angle = angles[(max_gap_index + 1) % angles.size()];
    range.end_angle   = angles[max_gap_index];
  }
  return range;
}

struct RayLine {
  cem::fusion::Point2DF direction;
};

static const std::unordered_map<uint32_t, RayLine> &GetRayLine() {
  static std::once_flag                        flag;
  static std::unordered_map<uint32_t, RayLine> ray_lines;
  std::call_once(flag, []() {
    for (auto angle = 0; angle < 360; angle = angle + FREESPACE_RESOLUTION) {
      RayLine ray_line;
      float   rad = angle * M_PIf32 / 180.0F;

      Eigen::Vector2f dir(std::cos(rad), std::sin(rad));
      ray_line.direction.x = 200 * dir.x();
      ray_line.direction.y = 200 * dir.y();
      ray_lines.emplace(angle, std::move(ray_line));
    }
  });
  return ray_lines;
};

BevMapPreProcessor::BevMapPreProcessor(std::shared_ptr<LandmarkBuffer> buffer)
    : landmark_buffer_(buffer) {}

BevMapPreProcessor::~BevMapPreProcessor() {}

void BevMapPreProcessor::Process(BevMapInfoPtr bev_map) {
  InitConfigs();
  if (bev_map != nullptr) {
    input_bev_map_ptr_ = bev_map;
  }

  FuseLidarRoadEdgeIntoBevMap();

  ConstructRightTurnTrajectory();

  if (!input_bev_map_ptr_) {
    return;
  }

  T_local_ego_ = FindTransform(input_bev_map_ptr_->header.timestamp);

  MergeSplitInterpolation();

  // Remove lanes that have no topology and are longer than 20 meters.
  bool is_on_highway = false;
  RoutingMapPtr routing_map_ptr{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(routing_map_ptr);
  if (routing_map_ptr) {
    is_on_highway = routing_map_ptr->is_on_highway;
  }
  if (is_on_highway) {
    for (auto lane_iter = input_bev_map_ptr_->lane_infos.rbegin(); lane_iter != input_bev_map_ptr_->lane_infos.rend(); ++lane_iter) {
      if (!lane_iter->previous_lane_ids.empty() || !lane_iter->next_lane_ids.empty()) {
        continue;
      }
      if (!lane_iter->line_points.empty() && lane_iter->line_points.front().x > kLaneRemovalRange) {
        input_bev_map_ptr_->lane_infos.erase((lane_iter + 1).base());
      }
    }
  }

  AddBevToNavigationContainer();

  AssignAttribute2LaneMarkerPts();

  BevTransToWorld(T_local_ego_);
}

void BevMapPreProcessor::Init() {
  lane_builder_params_ = LANE_BUILDER_PARAMS;
}

void BevMapPreProcessor::InitConfigs() {
  InitBevMapSubscriber();
  InitLidarRoadEdgeSubscriber();
}

void BevMapPreProcessor::InitBevMapSubscriber() {
  BevMapInfoPtr bev_map_ptr = nullptr;
  SensorDataManager::Instance()->GetLatestSensorFrame(bev_map_ptr);

  if (bev_map_ptr) {
    input_bev_map_ptr_  = std::make_shared<BevMapInfo>();
    *input_bev_map_ptr_ = *bev_map_ptr;
  } else {
    input_bev_map_ptr_ = nullptr;
  }
}

void BevMapPreProcessor::InitLidarRoadEdgeSubscriber() {
  LidarRoadEdgeInfoPtr lidar_road_edge = nullptr;
  SensorDataManager::Instance()->GetLatestSensorFrame(lidar_road_edge);
  if (lidar_road_edge) {
    lidar_road_edge_ptr_ = std::make_shared<LidarRoadEdgeInfo>(*lidar_road_edge);
  } else {
    lidar_road_edge_ptr_ = nullptr;
  }
}

bool BevMapPreProcessor::GenerateBezierPoints(std::vector<SamplingPoint> &control_points, std::vector<Point2DF> &res_points) {
  if (control_points.size() != 4) {
    return false;
  }
  SamplingPoint p1, p2, p3, p4;
  p1 = control_points[0];
  p2 = control_points[1];
  p3 = control_points[2];
  p4 = control_points[3];

  const float POINT_INTERVAL = 2.0f;
  if (ThirdOrderBezierFitUniform(POINT_INTERVAL, control_points)) {
    Point2DF pts;
    pts.x            = p1.x;
    pts.y            = p1.y;
    pts.mse          = NAN;
    pts.point_source = cem::message::common::PointSource::PS_BEV;
    res_points.emplace_back(pts);

    for (size_t i = 0; i < control_points.size(); i++) {
      if (control_points[i].x <= p1.x || control_points[i].x >= p4.x) {
        continue;
      }
      Point2DF pts;
      pts.x            = control_points[i].x;
      pts.y            = control_points[i].y;
      pts.mse          = NAN;
      pts.point_source = cem::message::common::PointSource::PS_BEV;

      res_points.emplace_back(pts);
    }
    Point2DF pts2;
    pts2.x            = p4.x;
    pts2.y            = p4.y;
    pts2.mse          = NAN;
    pts2.point_source = cem::message::common::PointSource::PS_BEV;
    res_points.emplace_back(pts2);
  } else {
    return false;
  }

  return true;
}
bool BevMapPreProcessor::GetBezierVector(const std::vector<Point2DF> &line_points, const bool &is_front, line_id_vec &liv) {
  if (line_points.size() < 2) {
    return false;
  }
  const static float CONTROL_POINT_LENGTH = 3.0;
  Point2DF           p1(0, 0), p2(0, 0);

  if (true == is_front) {
    p1 = line_points.front();
    p2 = line_points.back();
    for (int i = 1; i < line_points.size(); i++) {
      if (fabs(line_points[i].x - p1.x) > CONTROL_POINT_LENGTH) {
        p2 = line_points[i];
        break;
      } else if (fabs(line_points[i].x - p1.x) > 1e-5) {
        p2 = line_points[i];
      }
    }
    if (fabs(p2.x - line_points[line_points.size() - 1].x) < 1e-5) {
      if (line_points.size() == 2) {
        p2.x = (p1.x + p2.x) * 0.5;
        p2.y = (p1.y + p2.y) * 0.5;
      } else if (line_points.size() > 2) {
        p2.x = line_points[line_points.size() - 2].x;
        p2.y = line_points[line_points.size() - 2].y;
      }
    }
  } else {
    p1 = line_points.front();
    p2 = line_points.back();
    for (int i = (int)(line_points.size() - 2); i >= 0; i--) {
      if (fabs(p2.x - line_points[i].x) > CONTROL_POINT_LENGTH && fabs(p2.x - line_points[i].x) > 1e-5) {
        p1 = line_points[i];
        break;
      } else if (fabs(p2.x - line_points[i].x) > 1e-5) {
        p1 = line_points[i];
      }
    }
    if (fabs(p1.x - line_points[0].x) < 1e-5) {
      if (line_points.size() == 2) {
        p1.x = (p1.x + p2.x) * 0.5;
        p1.y = (p1.y + p2.y) * 0.5;
      } else if (line_points.size() > 2) {
        p1.x = line_points[1].x;
        p1.y = line_points[1].y;
      }
    }
  }
  liv.pts(0)            = p1.x;
  liv.pts(1)            = p1.y;
  liv.pts2(0)           = p2.x;
  liv.pts2(1)           = p2.y;
  Eigen::Vector2f delta = liv.pts2 - liv.pts;
  if (delta.norm() < 0.5f || delta.norm() > 10.0f) {
    return false;
  }

  return true;
}
void BevMapPreProcessor::CompensateLanePoints() {
  if (input_bev_map_ptr_ == nullptr) {
    return;
  }

  for (auto &lane : input_bev_map_ptr_->lane_infos) {
    if (!lane.id || lane.line_points.size() < 2) {
      continue;
    }

    std::sort(lane.line_points.begin(), lane.line_points.end(), [&](Point2DF &pt1, Point2DF &pt2) { return pt1.x < pt2.x; });

    // AINFO << " lane id: " << lane.id;
    for (size_t i = 0; i < lane.line_points.size() - 1; ++i) {
      float x1 = lane.line_points[i].x;
      float y1 = lane.line_points[i].y;
      float x2 = lane.line_points[i + 1].x;
      float y2 = lane.line_points[i + 1].y;
      if (x2 - x1 >= 9) {
        // AINFO << "start: " << lane.line_points[i].x << "," << lane.line_points[i].y;
        // AINFO << "end: " << lane.line_points[i + 1].x << "," << lane.line_points[i + 1].y;
        int numPointsToInsert = static_cast<int>((x2 - x1) / 2) - 1;
        for (int j = 1; j <= numPointsToInsert; ++j) {
          float newX = x1 + j * 2;
          float newY = y1 + (y2 - y1) * (j * 2) / (x2 - x1);
          lane.line_points.insert(lane.line_points.begin() + i + j, Point2DF(newX, newY));
          // AINFO << "add: " << newX << "," << newY;
        }
        i += numPointsToInsert;
        // AINFO << "numPointsToInsert: " << numPointsToInsert;
      }
    }

    std::vector<Eigen::Vector2f> geos;
    for (const auto &point : lane.line_points) {
      geos.emplace_back(point.x, point.y);
    }
    lane.geos = std::make_unique<std::vector<Eigen::Vector2f>>(geos);
  }
}

void BevMapPreProcessor::MergeSplitInterpolation() {
  std::vector<one_interpoltion_group> interpolation_groups;
  uint32_t                            group_id_count = 0;
  std::unordered_set<uint32_t>        next_ids;
  std::unordered_set<uint32_t>        prev_ids;

  CompensateLanePoints();
  for (size_t idx = 0; idx < input_bev_map_ptr_->lane_infos.size(); idx++) {
    auto                  &lane = input_bev_map_ptr_->lane_infos[idx];
    one_interpoltion_group oig;
    oig.group_id        = group_id_count++;
    oig.current_lane_id = lane.id;
    for (auto &id : lane.next_lane_ids) {
      lane_id_flag lif;
      lif.lane_id    = id;
      lif.is_visited = false;
      bool exist     = false;
      for (auto &e : oig.next_lane_ids) {
        if (e.lane_id == id) {
          exist = true;
        }
      }
      if (false == exist) {
        oig.next_lane_ids.emplace_back(lif);
        next_ids.insert(id);
      }
    }
    for (auto &id : lane.previous_lane_ids) {
      lane_id_flag lif;
      lif.lane_id    = id;
      lif.is_visited = false;
      bool exist     = false;
      for (auto &e : oig.prev_lane_ids) {
        if (e.lane_id == id) {
          exist = true;
        }
      }
      if (false == exist) {
        oig.prev_lane_ids.emplace_back(lif);
        prev_ids.insert(id);
      }
    }
    if (0 != lane.next_lane_ids.size() || 0 != lane.previous_lane_ids.size()) {
      interpolation_groups.emplace_back(oig);
    }
  }
  std::vector<line_link_pair_info> valid_link_pairs;
  for (auto &elem : interpolation_groups) {
    for (auto &next : elem.next_lane_ids) {  // next id list
      line_id_vec liv1;
      liv1.line_id    = elem.current_lane_id;
      liv1.is_valid   = false;
      bool liv1_exits = false;
      for (auto &vec : valid_link_pairs) {
        if (vec.first_lane.line_id == liv1.line_id && vec.first_lane.is_valid == true && false == vec.first_lane.is_front) {
          liv1.pts      = vec.first_lane.pts;
          liv1.pts2     = vec.first_lane.pts2;
          liv1.is_valid = true;
          liv1.is_front = false;
          liv1_exits    = true;
        }
      }
      for (auto &lane : input_bev_map_ptr_->lane_infos) {
        if (false == liv1_exits && lane.id == liv1.line_id && lane.line_points.size() >= 2 &&
            true == GetBezierVector(lane.line_points, false, liv1)) {
          liv1.is_valid = true;
          liv1.is_front = false;
          break;
        }
      }

      line_id_vec liv2;
      liv2.line_id    = next.lane_id;
      liv2.is_valid   = false;
      bool liv2_exits = false;
      for (auto &vec : valid_link_pairs) {
        if (vec.second_lane.line_id == liv2.line_id && vec.second_lane.is_valid == true && true == vec.second_lane.is_front) {
          liv2.pts      = vec.second_lane.pts;
          liv2.pts2     = vec.second_lane.pts2;
          liv2.is_valid = true;
          liv2.is_front = true;
          liv2_exits    = true;
        }
      }
      for (auto &lane : input_bev_map_ptr_->lane_infos) {
        if (false == liv2_exits && lane.id == liv2.line_id && lane.line_points.size() >= 2 &&
            true == GetBezierVector(lane.line_points, true, liv2)) {
          liv2.is_valid = true;
          liv2.is_front = true;
          break;
        }
      }

      bool is_duplicate = false;
      for (auto &line_pair : valid_link_pairs) {
        if ((liv1.line_id == line_pair.first_lane.line_id && liv2.line_id == line_pair.second_lane.line_id) ||
            (liv2.line_id == line_pair.first_lane.line_id && liv1.line_id == line_pair.second_lane.line_id)) {
          is_duplicate = true;
        }
      }
      if (false == is_duplicate) {
        line_link_pair_info llpi;
        llpi.first_lane  = liv1;
        llpi.second_lane = liv2;
        valid_link_pairs.emplace_back(llpi);
      }
    }
    for (auto &prev : elem.prev_lane_ids) {  // previous id list
      line_id_vec liv1;
      liv1.line_id    = prev.lane_id;
      liv1.is_valid   = false;
      bool liv1_exits = false;
      for (auto &vec : valid_link_pairs) {
        if (vec.first_lane.line_id == liv1.line_id && vec.first_lane.is_valid == true && false == vec.first_lane.is_front) {
          liv1.pts      = vec.first_lane.pts;
          liv1.pts2     = vec.first_lane.pts2;
          liv1.is_valid = true;
          liv1.is_front = false;
          liv1_exits    = true;
        }
      }
      for (auto &lane : input_bev_map_ptr_->lane_infos) {
        if (false == liv1_exits && lane.id == liv1.line_id && lane.line_points.size() >= 2 &&
            true == GetBezierVector(lane.line_points, false, liv1)) {
          liv1.is_valid = true;
          liv1.is_front = false;
          break;
        }
      }

      line_id_vec liv2;
      liv2.line_id    = elem.current_lane_id;
      liv2.is_valid   = false;
      bool liv2_exits = false;
      for (auto &vec : valid_link_pairs) {
        if (vec.second_lane.line_id == liv2.line_id && vec.second_lane.is_valid == true && true == vec.second_lane.is_front) {
          liv2.pts      = vec.second_lane.pts;
          liv2.pts2     = vec.second_lane.pts2;
          liv2.is_valid = true;
          liv2.is_front = true;
          liv2_exits    = true;
        }
      }
      for (auto &lane : input_bev_map_ptr_->lane_infos) {
        if (false == liv2_exits && lane.id == liv2.line_id && lane.line_points.size() >= 2 &&
            true == GetBezierVector(lane.line_points, true, liv2)) {
          liv2.is_valid = true;
          liv2.is_front = true;
          break;
        }
      }
      bool is_duplicate = false;
      for (auto &line_pair : valid_link_pairs) {
        if ((liv1.line_id == line_pair.first_lane.line_id && liv2.line_id == line_pair.second_lane.line_id) ||
            (liv2.line_id == line_pair.first_lane.line_id && liv1.line_id == line_pair.second_lane.line_id)) {
          is_duplicate = true;
        }
      }
      if (false == is_duplicate) {
        line_link_pair_info llpi;
        llpi.first_lane  = liv1;
        llpi.second_lane = liv2;
        valid_link_pairs.emplace_back(llpi);
      }
    }
  }

  std::unordered_set<uint32_t> multi_topo_line_ids;
  for (auto &id : next_ids) {
    if (prev_ids.count(id)) {  // multi-topo exists in the current lane id.
      int found_lane_index = -1;
      for (size_t i = 0; i < input_bev_map_ptr_->lane_infos.size(); i++) {
        auto lane = input_bev_map_ptr_->lane_infos[i];
        if (found_lane_index < 0 && lane.id == id) {
          found_lane_index = i;
        }
      }
      if (found_lane_index < 0) {
        continue;
      }
      int first_index  = -1;
      int second_index = -1;
      for (size_t idx = 0; idx < valid_link_pairs.size(); idx++) {  // common id connecting previous line.
        auto elem = valid_link_pairs[idx];
        if (id == elem.second_lane.line_id && true == elem.second_lane.is_valid && true == elem.first_lane.is_valid && first_index < 0) {
          auto &first_lane = input_bev_map_ptr_->lane_infos[found_lane_index];
          for (size_t i = 0; i < first_lane.line_points.size(); i++) {
            auto p = first_lane.line_points[i];
            if (p.x > elem.second_lane.pts2(0)) {
              first_index = i;
              break;
            }
          }
        }
      }
      for (size_t idx = 0; idx < valid_link_pairs.size(); idx++) {  // common id connecting next line.
        auto elem = valid_link_pairs[idx];
        if (id == elem.first_lane.line_id && true == elem.first_lane.is_valid && true == elem.second_lane.is_valid && second_index < 0) {
          auto &second_lane = input_bev_map_ptr_->lane_infos[found_lane_index];
          for (int i = static_cast<int>(second_lane.line_points.size() - 1); i >= 0; i--) {
            auto p = second_lane.line_points[i];
            if (p.x < elem.first_lane.pts(0)) {
              second_index = i;
              break;
            }
          }
        }
      }
      if (!((first_index + 2) < second_index)) {
        multi_topo_line_ids.insert(id);
      }
    }
  }
  // TODO: left/right lanemarker id and vector pairs
  for (size_t idx = 0; idx < valid_link_pairs.size(); idx++) {
    auto &elem              = valid_link_pairs[idx];
    int   first_lane_index  = -1;
    int   second_lane_index = -1;
    for (size_t i = 0; i < input_bev_map_ptr_->lane_infos.size(); i++) {
      auto lane = input_bev_map_ptr_->lane_infos[i];
      if (first_lane_index < 0 && lane.id == elem.first_lane.line_id) {
        first_lane_index = i;
      }
      if (second_lane_index < 0 && lane.id == elem.second_lane.line_id) {
        second_lane_index = i;
      }
    }
    if (first_lane_index >= 0 && second_lane_index >= 0) {
      auto first_lane_id             = input_bev_map_ptr_->lane_infos[first_lane_index].id;
      auto first_left_laneMarker_id  = input_bev_map_ptr_->lane_infos[first_lane_index].left_lane_marker_id;
      auto first_right_laneMarker_id = input_bev_map_ptr_->lane_infos[first_lane_index].right_lane_marker_id;

      auto second_lane_id             = input_bev_map_ptr_->lane_infos[second_lane_index].id;
      auto second_left_laneMarker_id  = input_bev_map_ptr_->lane_infos[second_lane_index].left_lane_marker_id;
      auto second_right_laneMarker_id = input_bev_map_ptr_->lane_infos[second_lane_index].right_lane_marker_id;
    }
  }
  // start generating Bezier points
  for (size_t i = 0; i < input_bev_map_ptr_->lane_infos.size(); i++) {
    auto &lane = input_bev_map_ptr_->lane_infos[i];
    std::vector<uint64_t>().swap(lane.next_lane_ids);
    std::vector<uint64_t>().swap(lane.previous_lane_ids);
  }

/*
  std::vector<traverseCrossWalkLane> crosswalk_lane_list;
  lane_topology_processor_.GetTraverseCrossWalkLaneInfo(crosswalk_lane_list);
*/

  for (size_t idx = 0; idx < valid_link_pairs.size(); idx++) {
    auto &elem = valid_link_pairs[idx];
    if (true == elem.first_lane.is_valid && true == elem.second_lane.is_valid) {
      std::vector<SamplingPoint> control_points;
      SamplingPoint              p1, p2, p3, p4;
      p1.x = elem.first_lane.pts(0);
      p1.y = elem.first_lane.pts(1);
      p2.x = elem.first_lane.pts2(0);
      p2.y = elem.first_lane.pts2(1);
      p3.x = elem.second_lane.pts(0);
      p3.y = elem.second_lane.pts(1);
      p4.x = elem.second_lane.pts2(0);
      p4.y = elem.second_lane.pts2(1);
      control_points.emplace_back(p1);
      control_points.emplace_back(p2);
      control_points.emplace_back(p3);
      control_points.emplace_back(p4);

      std::vector<Point2DF> generated_points;
      bool                  ret_val = GenerateBezierPoints(control_points, generated_points);
      if (true == ret_val) {
        int32_t first_lane_index  = -1;
        int32_t second_lane_index = -1;
        for (size_t i = 0; i < input_bev_map_ptr_->lane_infos.size(); i++) {
          auto &lane = input_bev_map_ptr_->lane_infos[i];
          if (first_lane_index < 0 && lane.id == elem.first_lane.line_id) {
            first_lane_index = i;
          }
          if (second_lane_index < 0 && lane.id == elem.second_lane.line_id) {
            second_lane_index = i;
          }
        }
        if (first_lane_index < 0 || second_lane_index < 0) {
          continue;
        }
        uint64_t new_lane_id = 101;
        for (uint64_t k = 1; k < 100UL; ++k) {
          auto tmp = std::find_if(input_bev_map_ptr_->lane_infos.begin(), input_bev_map_ptr_->lane_infos.end(),
                                  [k](const cem::message::sensor::BevLaneInfo &lane) { return lane.id == k; });
          if (tmp == input_bev_map_ptr_->lane_infos.end()) {
            new_lane_id = k;
            break;
          }
        }
        Point2DF first_point, last_point;
        if (generated_points.size() > 0) {
          first_point = generated_points[0];
          last_point  = generated_points[generated_points.size() - 1];
        }
        // first lane
        if (false == elem.first_lane.is_front && true == elem.first_lane.is_valid) {
          if (first_lane_index >= 0) {
            auto &first_lane = input_bev_map_ptr_->lane_infos[first_lane_index];
            if (!multi_topo_line_ids.count(elem.first_lane.line_id)) {
              std::vector<Point2DF> new_line_points;
              for (size_t i = 0; i < first_lane.line_points.size(); i++) {
                auto p = first_lane.line_points[i];
                if (p.x <= elem.first_lane.pts(0)) {
                  p.point_source = cem::message::common::PointSource::PS_BEV;
                  new_line_points.emplace_back(p);
                }
              }
              if (first_lane.line_points.size() == 2) {
                new_line_points.emplace_back(first_point);
              }
              if (new_line_points.size() >= 2) {
                first_lane.line_points.swap(new_line_points);
                first_lane.number_of_points = new_line_points.size();
              }
            }
            first_lane.next_lane_ids.emplace_back(new_lane_id);
            first_lane.is_bev_topo_connected = true;
          }
        }

        // generated lane
        BevLaneInfo one_new_lane;
        one_new_lane.id = new_lane_id;
        if (multi_topo_line_ids.count(elem.first_lane.line_id)) {
          auto &first_lane = input_bev_map_ptr_->lane_infos[first_lane_index];
          if (first_lane.line_points.size() > 0) {
            size_t last_index = first_lane.line_points.size() - 1;
            one_new_lane.line_points.emplace_back(first_lane.line_points[last_index]);
          }
        }
        for (size_t i = 0; i < generated_points.size(); i++) {
          if (multi_topo_line_ids.count(elem.first_lane.line_id)) {
            if (generated_points[i].x < elem.first_lane.pts2(0)) {
              continue;
            }
          }
          if (multi_topo_line_ids.count(elem.second_lane.line_id)) {
            if (generated_points[i].x > elem.second_lane.pts(0)) {
              continue;
            }
          }
          one_new_lane.line_points.emplace_back(generated_points[i]);
        }
        if (multi_topo_line_ids.count(elem.second_lane.line_id)) {
          auto &second_lane = input_bev_map_ptr_->lane_infos[second_lane_index];
          if (second_lane.line_points.size() > 0) {
            one_new_lane.line_points.emplace_back(second_lane.line_points[0]);
          }
        }
        // virtual property in zebra area
        bool is_virtual = false;
        for (auto &p : crosswalk_lane_list_) {
          if ((p.near_lane_id == elem.first_lane.line_id && p.far_lane_id == elem.second_lane.line_id) ||
              (p.far_lane_id == elem.second_lane.line_id && p.near_lane_id == elem.first_lane.line_id)) {
            is_virtual = true;
          }
        }
        one_new_lane.is_virtual       = is_virtual;
        one_new_lane.number_of_points = one_new_lane.line_points.size();
        one_new_lane.position         = static_cast<uint32_t>(BevLanePosition::LANE_LOC_OTHER);
        one_new_lane.previous_lane_ids.emplace_back(elem.first_lane.line_id);
        one_new_lane.next_lane_ids.emplace_back(elem.second_lane.line_id);
        one_new_lane.is_bev_topo_connected = true;
        one_new_lane.is_topological_connection = true;
        if (one_new_lane.line_points.size() >= 2) {
          auto first_lane = std::find_if(input_bev_map_ptr_->lane_infos.begin(), input_bev_map_ptr_->lane_infos.end(),
                              [find_id = elem.first_lane.line_id](BevLaneInfo &lane) { return lane.id == find_id; });
          if (first_lane != input_bev_map_ptr_->lane_infos.end() && first_lane->merge_topo_extend != MergeTopoExtendType::TOPOLOGY_MERGE_NONE) {
            one_new_lane.merge_topo_extend = first_lane->merge_topo_extend;
            first_lane->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
            one_new_lane.merge_info_extend = first_lane->merge_info_extend;
            first_lane->merge_info_extend.merge_valid  = 0;
            first_lane->merge_info_extend.dis_to_merge = 0;
          }

          auto second_lane = std::find_if(input_bev_map_ptr_->lane_infos.begin(), input_bev_map_ptr_->lane_infos.end(),
                              [find_id = elem.second_lane.line_id](BevLaneInfo &lane) { return lane.id == find_id; });
          if (second_lane != input_bev_map_ptr_->lane_infos.end()) {
            if (second_lane->split_topo_extend != SplitTopoExtendType::TOPOLOGY_SPLIT_NONE) {
              one_new_lane.split_topo_extend = second_lane->split_topo_extend;
              second_lane->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
            }
            one_new_lane.lane_type = second_lane->lane_type;
          }

          input_bev_map_ptr_->lane_infos.emplace_back(one_new_lane);
        }

        // second lane
        if (true == elem.second_lane.is_front && true == elem.second_lane.is_valid) {
          if (second_lane_index >= 0) {
            auto &second_lane = input_bev_map_ptr_->lane_infos[second_lane_index];
            if (!multi_topo_line_ids.count(elem.second_lane.line_id)) {
              std::vector<Point2DF> new_line_points;
              if (second_lane.line_points.size() == 2) {
                new_line_points.emplace_back(last_point);
              }
              for (size_t i = 0; i < second_lane.line_points.size(); i++) {
                auto p = second_lane.line_points[i];
                if (p.x >= elem.second_lane.pts2(0)) {
                  p.point_source = cem::message::common::PointSource::PS_BEV;
                  new_line_points.emplace_back(p);
                }
              }
              if (new_line_points.size() >= 2) {
                second_lane.line_points.swap(new_line_points);
                second_lane.number_of_points = new_line_points.size();
              }
            }
            second_lane.previous_lane_ids.emplace_back(new_lane_id);
          }
        }
      }
    }
  }
  return;
}

void BevMapPreProcessor::AddBevToNavigationContainer() {
  auto &raw_bev_data = INTERNAL_PARAMS.raw_bev_data;
  raw_bev_data.ClearContainer();
  raw_bev_data.SetBevMapPtr(input_bev_map_ptr_);

  for (auto &lane_marker_tmp : input_bev_map_ptr_->lanemarkers) {
    lane_marker_tmp.geos             = std::make_shared<std::vector<Eigen::Vector2f>>();
    lane_marker_tmp.geos->clear();
    for (auto &pt : lane_marker_tmp.line_points) {
      lane_marker_tmp.geos->emplace_back(pt.x, pt.y);
    }
    lane_marker_tmp.indexed_geos.BuildIndex(lane_marker_tmp.geos);
    raw_bev_data.AddLaneBoundary(lane_marker_tmp.id, &lane_marker_tmp);
  }
  for (auto &lane_info_tmp : input_bev_map_ptr_->lane_infos) {
    lane_info_tmp.geos             = std::make_shared<std::vector<Eigen::Vector2f>>();
    lane_info_tmp.geos->clear();
    for (auto &pt : lane_info_tmp.line_points) {
      lane_info_tmp.geos->emplace_back(pt.x, pt.y);
    }
    lane_info_tmp.indexed_geos.BuildIndex(lane_info_tmp.geos);
    CalculateLaneLength(lane_info_tmp);
    raw_bev_data.AddLaneInfo(lane_info_tmp.id, &lane_info_tmp);
  }

  for (auto &road_edge_tmp : input_bev_map_ptr_->edges) {
    road_edge_tmp.geos             = std::make_shared<std::vector<Eigen::Vector2f>>();
    road_edge_tmp.geos->clear();
    for (auto &pt : road_edge_tmp.line_points) {
      road_edge_tmp.geos->emplace_back(pt.x, pt.y);
    }
    road_edge_tmp.indexed_geos.BuildIndex(road_edge_tmp.geos);
    raw_bev_data.AddRoadEdge(road_edge_tmp.id, &road_edge_tmp);
  }
}

void BevMapPreProcessor::AssignAttribute2LaneMarkerPts() {
  for (auto &LaneMarker : input_bev_map_ptr_->lanemarkers) {
    int len = LaneMarker.line_points.size();
    for (int ptsIndx = 0; ptsIndx < len; ptsIndx++) {
      BevLaneMarkerType  type  = BevLaneMarkerType::BEV_LMT__UNDECIDED;
      BevLaneMarkerColor color = BevLaneMarkerColor::BEV_LMC__UNKNOWN;
      for (auto &typeSeg : LaneMarker.type_segs) {
        if (typeSeg.start_index <= ptsIndx && ptsIndx <= typeSeg.end_index) {
          type = (typeSeg.type);
          break;
        }
      }
      LaneMarker.line_points[ptsIndx].type = static_cast<cem::message::common::PointType>(type);
      for (auto &colorSeg : LaneMarker.color_segs) {
        if (colorSeg.start_index <= ptsIndx && ptsIndx <= colorSeg.end_index) {
          color = (colorSeg.color);
          break;
        }
      }
      LaneMarker.line_points[ptsIndx].color = static_cast<cem::message::common::PointColor>(color);
    }
  }

  // 修改LaneMarker的颜色
  for (auto &lanemarker : input_bev_map_ptr_->lanemarkers) {
    auto &points = lanemarker.line_points;
    if (points.size() < 2) {
      continue;
    }

    for (size_t pt_idx = 0; pt_idx < points.size() - 1; pt_idx++) {
      if (points[pt_idx + 1].color == PointColor::BEV_LMC__UNKNOWN && points[pt_idx].color != PointColor::BEV_LMC__UNKNOWN) {
        points[pt_idx + 1].color = points[pt_idx].color;
      }
    }

    for (size_t pt_idx = points.size() - 1; pt_idx > 0; pt_idx--) {
      if (points[pt_idx - 1].color == PointColor::BEV_LMC__UNKNOWN && points[pt_idx].color != PointColor::BEV_LMC__UNKNOWN) {
        points[pt_idx - 1].color = points[pt_idx].color;
      }
    }
  }

  // AINFO << "*******PtsColorAfter******"; 
  // AINFO << "sequence number: " << input_bev_map_ptr_->header.cycle_counter;
  // for (auto &lanemarker : input_bev_map_ptr_->lanemarkers){
  //   AINFO << "lanemarker ID: " << lanemarker.id;
  //   std::stringstream pt_color_str;
  //   for (const auto& pt : lanemarker.line_points){
  //     pt_color_str << static_cast<int>(pt.color) << ", ";
  //   }
  //   AINFO << "pt_color_str: " << pt_color_str.str();
  // }
  // AINFO << "*************************";
  return;
}

void BevMapPreProcessor::BevTransToWorld(const std::optional<Eigen::Isometry3d> &T_local_ego) {
  if (T_local_ego == std::nullopt) {
    AERROR << "bev map can not find the odometry";
    detect_bev_map_.bev_map_ptr = nullptr;
    detect_bev_map_.Twb         = Eigen::Isometry3d::Identity();
    return;
  }
  BevMapInfo global_bev_map = *(input_bev_map_ptr_);

  global_bev_map.lanemarkers.clear();
  for (auto lane : input_bev_map_ptr_->lanemarkers) {
    auto &lanePoints = lane.line_points;
    for (auto &point : lanePoints) {
      TransformPoint(&point, T_local_ego.value());
    }
    global_bev_map.lanemarkers.emplace_back(lane);
  }
  global_bev_map.edges.clear();
  for (auto edge : input_bev_map_ptr_->edges) {
    auto &edgePoints = edge.line_points;
    for (auto &point : edgePoints) {
      TransformPoint(&point, T_local_ego.value());
    }
    global_bev_map.edges.emplace_back(edge);
  }
  global_bev_map.lane_infos.clear();
  for (auto laneCenter : input_bev_map_ptr_->lane_infos) {
    auto &laneCenterPoints = laneCenter.line_points;
    for (auto &point : laneCenterPoints) {
      TransformPoint(&point, T_local_ego.value());
    }
    global_bev_map.lane_infos.emplace_back(laneCenter);
  }
  global_bev_map.stop_lines.clear();
  for (auto stopLine : input_bev_map_ptr_->stop_lines) {
    auto &stopLinePoints = stopLine.line_points;
    for (auto &point : stopLinePoints) {
      TransformPoint(&point, T_local_ego.value());
    }
    global_bev_map.stop_lines.emplace_back(stopLine);
  }
  global_bev_map.crosswalks.clear();
  for (auto crossWalk : input_bev_map_ptr_->crosswalks) {
    auto &crossWalkPoints = crossWalk.line_points;
    for (auto &point : crossWalkPoints) {
      TransformPoint(&point, T_local_ego.value());
    }
    global_bev_map.crosswalks.emplace_back(crossWalk);
  }
  global_bev_map.junctions.clear();
  for (auto junction : input_bev_map_ptr_->junctions) {
    auto &points = junction.line_points;
    for (auto &point : points) {
      TransformPoint(&point, T_local_ego.value());
    }
    global_bev_map.junctions.emplace_back(junction);
  }
  global_bev_map.diversion_zone.clear();
  for (auto diversion_tmp : input_bev_map_ptr_->diversion_zone) {
    auto &points = diversion_tmp.line_points;
    for (auto &point : points) {
      TransformPoint(&point, T_local_ego.value());
    }
    global_bev_map.diversion_zone.emplace_back(diversion_tmp);
  }

  for (auto &section : global_bev_map.route.sections) {
    for (auto &point : section.points) {
      TransformPoint(&point, T_local_ego.value());
    }
  }

  detect_bev_map_.bev_map_ptr = std::make_shared<BevMapInfo>(global_bev_map);
  detect_bev_map_.Twb         = T_local_ego.value();
}

void BevMapPreProcessor::CalculateLaneLength(BevLaneInfo &lane) {
  lane.length = 0;
  if (lane.line_points.size() < 2) {
    return;
  }

  for (size_t i = 0; i < lane.line_points.size() - 1; i++) {
    lane.length += std::hypot(lane.line_points[i].x - lane.line_points[i + 1].x, lane.line_points[i].y - lane.line_points[i + 1].y);
  }
}

std::optional<Eigen::Isometry3d> BevMapPreProcessor::FindTransform(const double &timestamp) {
  LocalizationPtr odom_ptr{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(timestamp, 0.05, odom_ptr);

  if (odom_ptr == nullptr) {
    return std::nullopt;
  }

  Eigen::Isometry3d T_local_ego = Eigen::Isometry3d::Identity();
  Eigen::AngleAxisd R_local_ego((odom_ptr->attitude_dr) * M_PI / 180.0, Eigen::Vector3d(0, 0, 1));
  Eigen::Vector3d t(odom_ptr->posne_dr.at(0), odom_ptr->posne_dr.at(1), 0.0);
  T_local_ego.rotate(R_local_ego);
  T_local_ego.pretranslate(t);

  return T_local_ego;
}

Eigen::Isometry3d BevMapPreProcessor::FindRealTransform(const double &timestamp) {
  LocalizationPtr odom_ptr{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(timestamp, 0.05, odom_ptr);

  if (odom_ptr == nullptr) {
    return Eigen::Isometry3d::Identity();
  }

  Eigen::Isometry3d Twb_0 = Eigen::Isometry3d::Identity();
  Eigen::AngleAxisd Rwb_0((odom_ptr->attitude_dr) * M_PI / 180.0, Eigen::Vector3d(0, 0, 1));
  Eigen::Vector3d   trans_0(odom_ptr->posne_dr.at(0), odom_ptr->posne_dr.at(1), 0.0);
  Twb_0.rotate(Rwb_0);
  Twb_0.pretranslate(trans_0);

  return Twb_0;
}

Eigen::Isometry3d BevMapPreProcessor::CalcRotateTranslateMatrix(LocalizationConstPtr last_loc, LocalizationConstPtr cur_loc) {
  Eigen::Isometry3d rotate_translate_matrix = Eigen::Isometry3d::Identity();

  Eigen::AngleAxisd last_global_V(last_loc->attitude_dr * M_PI / 180, Eigen::Vector3d(0, 0, 1));
  Eigen::Vector3d   last_global_T(last_loc->posne_dr.at(0), last_loc->posne_dr.at(1), 0);

  Eigen::AngleAxisd cur_global_V(cur_loc->attitude_dr * M_PI / 180, Eigen::Vector3d(0, 0, 1));
  Eigen::Vector3d   cur_global_T(cur_loc->posne_dr.at(0), cur_loc->posne_dr.at(1), 0);

  Eigen::Matrix3d rotate = Eigen::Matrix3d::Identity();
  rotate                 = cur_global_V.inverse() * last_global_V;

  Eigen::Vector3d translate = cur_global_V.inverse() * (last_global_T - cur_global_T);

  rotate_translate_matrix.rotate(rotate);
  rotate_translate_matrix.pretranslate(translate);

  return rotate_translate_matrix;
}

void BevMapPreProcessor::TransformLidarRoadEdgeToBevTime() {
  if ((lidar_road_edge_ptr_ == nullptr) || (input_bev_map_ptr_ == nullptr)) {
    return;
  }

  double bevMapTime   = input_bev_map_ptr_->header.timestamp;
  double lidarMapTime = lidar_road_edge_ptr_->header.timestamp;
  auto   Tbev_b2w     = FindRealTransform(bevMapTime);
  auto   Tlidar_b2w   = FindRealTransform(lidarMapTime);
  if (Tbev_b2w.isApprox(Eigen::Isometry3d::Identity()) || Tlidar_b2w.isApprox(Eigen::Isometry3d::Identity())) {
    return;
  }
  lidar_road_edge_ptr_->header.timestamp = input_bev_map_ptr_->header.timestamp;
  auto T_lidar2bev                       = Tbev_b2w.inverse() * Tlidar_b2w;
  for (auto &edge : lidar_road_edge_ptr_->edges) {
    for (auto &point : edge.line_points) {
      TransformPoint(&point, T_lidar2bev);
    }
  }
}

void BevMapPreProcessor::FuseLidarRoadEdgeIntoBevMap() {
  lidar_freespace_.clear();
  lidar_cluster_roadedges_.clear();
  static uint32_t sequence_number = 1;
  static double   lidarMapTime    = 0.0;
  if ((lidar_road_edge_ptr_ == nullptr) || (input_bev_map_ptr_ == nullptr)) {
    return;
  }

  // AINFO << "lidar seq: " << lidar_road_edge_ptr_->header.cycle_counter;
  // AINFO << "bev seq: " << input_bev_map_ptr_->header.cycle_counter;
  if (sequence_number >= 50) {
    bool            low_speed = false;
    LocalizationPtr odomPtr{nullptr};
    SensorDataManager::Instance()->GetLatestSensorFrame(lidarMapTime, 0.05, odomPtr);
    if (odomPtr != nullptr) {
      double speed = 0;
      for (auto v : odomPtr->velocity) {
        speed += v * v;
      }
      if (speed < 0.5) {
        low_speed = true;
      }
    }
    for (auto iter_edges_group = hist_lidar_roadedge_group_.begin(); iter_edges_group != hist_lidar_roadedge_group_.end();) {
      if (sequence_number - iter_edges_group->second.begin()->first > 50) {
        if (low_speed) {
          if (iter_edges_group->second.rbegin()->first == sequence_number - 1) {
            iter_edges_group->second.erase(iter_edges_group->second.rbegin()->first);
          }
        } else {
          if (iter_edges_group->second.begin()->first < sequence_number - 50) {
            iter_edges_group->second.erase(iter_edges_group->second.begin()->first);
          }
        }
      }
      if (iter_edges_group->second.empty()) {
        iter_edges_group = hist_lidar_roadedge_group_.erase(iter_edges_group);
      } else {
        ++iter_edges_group;
      }
    }
  }

  double bevMapTime = input_bev_map_ptr_->header.timestamp;
  auto   Tbev_b2w   = FindRealTransform(bevMapTime);
  auto   Tlidar_b2w = FindRealTransform(lidarMapTime);
  if (!Tbev_b2w.isApprox(Eigen::Isometry3d::Identity()) && !Tlidar_b2w.isApprox(Eigen::Isometry3d::Identity())) {
    auto T_lidar2bev = Tbev_b2w.inverse() * Tlidar_b2w;
    for (auto &edges_group : hist_lidar_roadedge_group_) {
      for (auto &seq_edges : edges_group.second) {
        for (auto &edge_info : seq_edges.second) {
          edge_info.segs_bbox.clear();
          edge_info.total_bbox.Reset();
          if (!edge_info.edge.line_points.empty()) {
            TransformPoint(&edge_info.edge.line_points.at(0), T_lidar2bev);
            edge_info.total_bbox.ImportPoint(edge_info.edge.line_points.at(0));
          }
          for (size_t i = 1; i < edge_info.edge.line_points.size(); ++i) {
            auto &pt_0 = edge_info.edge.line_points.at(i - 1);
            auto &pt_1 = edge_info.edge.line_points.at(i);
            TransformPoint(&pt_1, T_lidar2bev);
            edge_info.total_bbox.ImportPoint(pt_1);
            AABB bbox_tmp(pt_0);
            bbox_tmp.ImportPoint(pt_1);
            edge_info.segs_bbox.push_back(std::move(bbox_tmp));
          }
        }
      }
    }
    TransformLidarRoadEdgeToBevTime();
  }

  AddDataIntoLidarRoadEdgeGroup(sequence_number);

#ifdef USE_LIDAR_CLUSTER_ROAD_EDGE_ONLY
  input_bev_map_ptr_->edges.clear();
#endif
  auto &ray_lines = GetRayLine();
  for (auto &edges_group : hist_lidar_roadedge_group_) {
    if (edges_group.second.size() < 10) {
      continue;
    }
    auto angle_range = CalcClusterAngleRange(edges_group.second);
    // AINFO << "group id: " << edges_group.first
    //       << ", seq_size: " << edges_group.second.size() << ", angle_range:
    //       ("
    //       << angle_range.start_angle << ", " << angle_range.end_angle << ")";

    if (!angle_range.valid) {
      continue;
    }

    int start_angle =
        (static_cast<int>(std::ceil(angle_range.start_angle / static_cast<double>(FREESPACE_RESOLUTION))) * FREESPACE_RESOLUTION);

    bool                  not_interpreting = true;
    cem::fusion::Point2DF origin;
    origin.x = 0;
    origin.y = 0;
    for (int ray_line_key = start_angle;; ray_line_key += FREESPACE_RESOLUTION) {
      if (angle_range.start_angle > angle_range.end_angle) {
        if (ray_line_key == 360) {
          ray_line_key     = 0;
          not_interpreting = false;
        }
        if (!not_interpreting) {
          if (ray_line_key > angle_range.end_angle) {
            break;
          }
        }
      } else {
        if (ray_line_key > angle_range.end_angle) {
          break;
        }
      }

      if (ray_lines.count(ray_line_key) == 0) {
        continue;
      }

      cem::fusion::Point2DF intersection_hist;
      intersection_hist.x       = 200;
      intersection_hist.y       = 200;
      bool     has_intersection = false;
      uint32_t from_seq_hist;
      for (auto iter = edges_group.second.rbegin(); iter != edges_group.second.rend(); ++iter) {
        cem::fusion::Point2DF intersection_cur_frame;
        intersection_cur_frame.x        = 200;
        intersection_cur_frame.y        = 200;
        bool has_intersection_cur_frame = false;
        for (auto &edge : iter->second) {
          if (edge.edge.line_points.size() < 2) {
            continue;
          }
          for (size_t i = 1; i < edge.edge.line_points.size(); ++i) {
            cem::fusion::Point2DF tmp;
            if (GetLineIntersection(origin, ray_lines.at(ray_line_key).direction, edge.edge.line_points.at(i - 1),
                                    edge.edge.line_points.at(i), tmp)) {
              if (intersection_cur_frame.x * intersection_cur_frame.x + intersection_cur_frame.y * intersection_cur_frame.y >
                  tmp.x * tmp.x + tmp.y * tmp.y) {
                intersection_cur_frame.x   = tmp.x;
                intersection_cur_frame.y   = tmp.y;
                has_intersection_cur_frame = true;
              }
            }
          }
        }
        if (has_intersection_cur_frame) {
          float normal_cur_frame =
              intersection_cur_frame.x * intersection_cur_frame.x + intersection_cur_frame.y * intersection_cur_frame.y;
          float normal_hist = intersection_hist.x * intersection_hist.x + intersection_hist.y * intersection_hist.y;
          /*if (iter == edges_group.second.rbegin()) {
            intersection_hist.x = intersection_cur_frame.x;
            intersection_hist.y = intersection_cur_frame.y;
            has_intersection = true;
            from_seq_hist = iter->first;
            break;
          } else */
          if (normal_hist - normal_cur_frame > 0.3 * 0.3) {
            intersection_hist.x = intersection_cur_frame.x;
            intersection_hist.y = intersection_cur_frame.y;
            has_intersection    = true;
            from_seq_hist       = iter->first;
          }
        }
      }
      if (has_intersection) {
        if (lidar_freespace_.count(ray_line_key) == 0) {
          FreeSpaceRay ray_line;
          ray_line.angle         = ray_line_key;
          ray_line.end_point.x() = intersection_hist.x;
          ray_line.end_point.y() = intersection_hist.y;
          ray_line.from_seq      = from_seq_hist;
          ray_line.from_group    = edges_group.first;
          lidar_freespace_.emplace(ray_line_key, std::move(ray_line));
        } else {
          float normal_this_group  = intersection_hist.x * intersection_hist.x + intersection_hist.y * intersection_hist.y;
          auto &intersection_exist = lidar_freespace_.at(ray_line_key);
          float normal_other_group = intersection_exist.end_point.x() * intersection_exist.end_point.x() +
                                     intersection_exist.end_point.y() * intersection_exist.end_point.y();
          if (from_seq_hist >= intersection_exist.from_seq) {
            if (normal_this_group < normal_other_group) {
              intersection_exist.end_point.x() = intersection_hist.x;
              intersection_exist.end_point.y() = intersection_hist.y;
              intersection_exist.from_seq      = from_seq_hist;
              intersection_exist.from_group    = edges_group.first;
            }
          } else {
            if (normal_this_group - normal_other_group < -0.3 * 0.3) {
              intersection_exist.end_point.x() = intersection_hist.x;
              intersection_exist.end_point.y() = intersection_hist.y;
              intersection_exist.from_seq      = from_seq_hist;
              intersection_exist.from_group    = edges_group.first;
            }
          }
        }
      }
    }
    // for (auto &seq_edges : edges_group.second) {
    //   for (auto &edge : seq_edges.second) {
    //     edge.edge.id = edges_group.first;
    //     input_bev_map_ptr_->edges.push_back(edge.edge);
    //   }
    // }
  }

  // freespace blind area scan
  std::unordered_map<uint32_t, std::vector<FreeSpaceRay *>> edges;
  for (uint32_t i = 0; i < 360; i += FREESPACE_RESOLUTION) {
    if (lidar_freespace_.count(i) == 0) {
      continue;
    }
    auto &ray = lidar_freespace_.at(i);
    ray.points.push_back(ray.end_point);
    if (edges.count(ray.from_group) == 0) {
      edges[ray.from_group].push_back(&ray);
    } else {
      auto &edge = edges.at(ray.from_group);
      auto  it   = std::lower_bound(edge.begin(), edge.end(), &ray,
                                    [](const FreeSpaceRay *ele, const FreeSpaceRay *ray_in) { return ele->angle < ray_in->angle; });
      edge.insert(it, &ray);
    }
  }
  for (auto &[group, edge] : edges) {
    if (edge.back()->angle - edge.front()->angle > 350) {
      for (int i = 1; i < edge.size(); ++i) {
        if (edge.at(i)->angle - edge.at(i - 1)->angle > 10) {
          std::rotate(edge.begin(), edge.begin() + i, edge.end());
          break;
        }
      }
    }
  }
  Eigen::Rotation2D<double> rotation_anticlockwise(M_PI / 2);
  Eigen::Rotation2D<double> rotation_clockwise(-M_PI / 2);
  for (auto &[group, edge] : edges) {
    if (edge.size() < 2) {
      continue;
    }
    {
      auto                 &front_ray = edge.front();
      cem::fusion::Point2DF end_scan_pt;
      {
        int front_ray_scan_end_angle = front_ray->angle - FREESPACE_RESOLUTION;
        front_ray_scan_end_angle     = front_ray_scan_end_angle < 0 ? front_ray_scan_end_angle + 360 : front_ray_scan_end_angle;
        while (true) {
          // AINFO << __LINE__ << ": " << front_ray_scan_end_angle;
          if (lidar_freespace_.count(front_ray_scan_end_angle) == 0) {
            front_ray_scan_end_angle -= FREESPACE_RESOLUTION;
            front_ray_scan_end_angle = front_ray_scan_end_angle < 0 ? front_ray_scan_end_angle + 360 : front_ray_scan_end_angle;
          } else {
            break;
          }
        }
        auto pt       = lidar_freespace_.at(front_ray_scan_end_angle).end_point;
        end_scan_pt.x = pt.x();
        end_scan_pt.y = pt.y();
      }
      auto front_blind_area_scan_direction = rotation_anticlockwise * (-front_ray->end_point);
      front_blind_area_scan_direction.normalize();
      front_blind_area_scan_direction                   = 8 * front_blind_area_scan_direction;
      auto                  front_blind_area_scan_point = front_blind_area_scan_direction + front_ray->end_point;
      cem::fusion::Point2DF front_scan_start;
      front_scan_start.x = front_blind_area_scan_point.x();
      front_scan_start.y = front_blind_area_scan_point.y();
      cem::fusion::Point2DF front_ray_pt;
      front_ray_pt.x = front_ray->end_point.x();
      front_ray_pt.y = front_ray->end_point.y();

      int scan_flag = CrossProduct(front_scan_start, front_ray_pt, end_scan_pt) > 0 ? 1 : -1;

      // cem::message::sensor::BevLaneMarker tmp;
      // tmp.line_points.push_back(front_scan_start);
      // tmp.line_points.push_back(front_ray_pt);
      // tmp.number_of_points = 2;
      // tmp.id = 100 * group + 1;
      // input_bev_map_ptr_->edges.push_back(std::move(tmp));
      // cem::message::sensor::BevLaneMarker tmp1;
      // tmp1.id = 100 * group + 2;
      // tmp1.line_points.push_back(front_scan_start);
      // tmp1.line_points.push_back(end_scan_pt);
      // tmp1.number_of_points = 2;
      // input_bev_map_ptr_->edges.push_back(std::move(tmp1));
      // AINFO << "scan_flag: " << scan_flag;
      // AINFO << "scan_st: " << front_scan_start.x << ", " <<
      // front_scan_start.y; AINFO << "scan_end1: " << front_ray_pt.x << ", " <<
      // front_ray_pt.y; AINFO << "scan_end2: " << end_scan_pt.x << ", " <<
      // end_scan_pt.y;

      auto &cluster = hist_lidar_roadedge_group_.at(group);
      int   i       = 5;
      while (true) {
        // AINFO << __LINE__;
        Eigen::Rotation2D<double> rotation(-i * M_PI / 180);
        auto                      scan_direction = rotation * (-front_blind_area_scan_direction);
        cem::fusion::Point2DF     front_scan_end;
        front_scan_end.x = 100 * scan_direction.x() + front_scan_start.x;
        front_scan_end.y = 100 * scan_direction.y() + front_scan_start.y;

        // if (i % 10 == 0) {
        //   cem::message::sensor::BevLaneMarker tmp;
        //   tmp.line_points.push_back(front_scan_start);
        //   tmp.line_points.push_back(front_scan_end);
        //   tmp.number_of_points = 2;
        //   tmp.id = 100 * group + i * 100000;
        //   input_bev_map_ptr_->edges.push_back(std::move(tmp));
        // }

        cem::fusion::Point2DF intersection;
        intersection.x        = 200;
        intersection.y        = 200;
        bool has_intersection = false;
        for (auto iter = cluster.rbegin(); iter != cluster.rend(); ++iter) {
          cem::fusion::Point2DF intersection_cur_frame;
          intersection_cur_frame.x        = 200;
          intersection_cur_frame.y        = 200;
          bool has_intersection_cur_frame = false;
          for (auto &edge : iter->second) {
            if (edge.edge.line_points.size() < 2) {
              continue;
            }
            for (size_t i = 1; i < edge.edge.line_points.size(); ++i) {
              cem::fusion::Point2DF tmp;
              if (GetLineIntersection(front_scan_start, front_scan_end, edge.edge.line_points.at(i - 1), edge.edge.line_points.at(i),
                                      tmp)) {
                if (intersection_cur_frame.x * intersection_cur_frame.x + intersection_cur_frame.y * intersection_cur_frame.y >
                    tmp.x * tmp.x + tmp.y * tmp.y) {
                  intersection_cur_frame.x   = tmp.x;
                  intersection_cur_frame.y   = tmp.y;
                  has_intersection_cur_frame = true;
                }
              }
            }
          }
          if (has_intersection_cur_frame) {
            if (intersection.x * intersection.x + intersection.y * intersection.y - intersection_cur_frame.x * intersection_cur_frame.x -
                    intersection_cur_frame.y * intersection_cur_frame.y >
                1) {
              intersection.x   = intersection_cur_frame.x;
              intersection.y   = intersection_cur_frame.y;
              has_intersection = true;
            }
          }
        }
        if (has_intersection) {
          bool valid_intersect = false;
          int  tmp             = CrossProduct(front_scan_start, front_scan_end, end_scan_pt) > 0 ? 1 : -1;
          if (tmp * scan_flag > 0) {
            valid_intersect = true;
          }
          if (valid_intersect) {
            Eigen::Vector2d pt_tmp;
            pt_tmp.x() = intersection.x;
            pt_tmp.y() = intersection.y;
            if (front_ray->points.size() >= 1) {
              // auto &refer_first = (*(edge.begin() + 1))->end_point;
              auto &refer_first  = edge.back()->end_point;
              auto &refer_second = front_ray->end_point;
              // auto &cur_first = front_ray->points.front();
              if ((refer_second - refer_first).norm() > (pt_tmp - refer_first).norm()) {
                break;
              }

              // if ((refer_second - refer_first).dot(pt_tmp - cur_first) < 0) {
              //   break;
              // }
              // double cos_theta =
              //     (refer_second - refer_first).dot(pt_tmp - cur_first) /
              //     ((refer_second - refer_first).norm() *
              //      (pt_tmp - cur_first).norm());
              // double theta = std::acos(cos_theta);
              // if (cos_theta > M_PI / 4) {
              //   break;
              // }
            }
            front_ray->points.insert(front_ray->points.begin(), pt_tmp);
          } else {
            break;
          }
        } else {
          break;
        }
        i += 5;
      }
    }
    {
      auto                 &back_ray = edge.back();
      cem::fusion::Point2DF end_scan_pt;
      {
        int back_ray_scan_end_angle = back_ray->angle + FREESPACE_RESOLUTION;
        back_ray_scan_end_angle %= 360;
        while (true) {
          // AINFO << __LINE__ << ":" << back_ray_scan_end_angle;
          if (lidar_freespace_.count(back_ray_scan_end_angle) == 0) {
            back_ray_scan_end_angle += FREESPACE_RESOLUTION;
            back_ray_scan_end_angle %= 360;
          } else {
            break;
          }
        }
        auto pt       = lidar_freespace_.at(back_ray_scan_end_angle).end_point;
        end_scan_pt.x = pt.x();
        end_scan_pt.y = pt.y();
      }
      auto back_blind_area_scan_direction = rotation_clockwise * (-back_ray->end_point);
      back_blind_area_scan_direction.normalize();
      back_blind_area_scan_direction                   = 8 * back_blind_area_scan_direction;
      auto                  back_blind_area_scan_point = back_blind_area_scan_direction + back_ray->end_point;
      cem::fusion::Point2DF back_scan_start;
      back_scan_start.x = back_blind_area_scan_point.x();
      back_scan_start.y = back_blind_area_scan_point.y();
      cem::fusion::Point2DF back_ray_pt;
      back_ray_pt.x = back_ray->end_point.x();
      back_ray_pt.y = back_ray->end_point.y();

      int scan_flag = CrossProduct(back_scan_start, back_ray_pt, end_scan_pt) > 0 ? 1 : -1;

      // cem::message::sensor::BevLaneMarker tmp;
      // tmp.line_points.push_back(back_scan_start);
      // tmp.line_points.push_back(back_ray_pt);
      // tmp.number_of_points = 2;
      // tmp.id = 100 * group + 1;
      // input_bev_map_ptr_->edges.push_back(std::move(tmp));
      // cem::message::sensor::BevLaneMarker tmp1;
      // tmp1.id = 100 * group + 2;
      // tmp1.line_points.push_back(back_scan_start);
      // tmp1.line_points.push_back(end_scan_pt);
      // tmp1.number_of_points = 2;
      // input_bev_map_ptr_->edges.push_back(std::move(tmp1));
      // AINFO << "scan_flag: " << scan_flag;
      // AINFO << "scan_st: " << back_scan_start.x << ", " << back_scan_start.y;
      // AINFO << "scan_end1: " << back_ray_pt.x << ", " << back_ray_pt.y;
      // AINFO << "scan_end2: " << end_scan_pt.x << ", " << end_scan_pt.y;

      auto &cluster = hist_lidar_roadedge_group_.at(group);
      int   i       = 5;
      while (true) {
        // AINFO << __LINE__;
        Eigen::Rotation2D<double> rotation(i * M_PI / 180);
        auto                      scan_direction = rotation * (-back_blind_area_scan_direction);
        cem::fusion::Point2DF     back_scan_end;
        back_scan_end.x = 100 * scan_direction.x() + back_scan_start.x;
        back_scan_end.y = 100 * scan_direction.y() + back_scan_start.y;

        // if (i % 10 == 0) {
        //   cem::message::sensor::BevLaneMarker tmp;
        //   tmp.line_points.push_back(back_scan_start);
        //   tmp.line_points.push_back(back_scan_end);
        //   tmp.number_of_points = 2;
        //   tmp.id = 100 * group + i * 100000;
        //   input_bev_map_ptr_->edges.push_back(std::move(tmp));
        // }

        cem::fusion::Point2DF intersection;
        intersection.x        = 200;
        intersection.y        = 200;
        bool has_intersection = false;
        for (auto iter = cluster.rbegin(); iter != cluster.rend(); ++iter) {
          cem::fusion::Point2DF intersection_cur_frame;
          intersection_cur_frame.x        = 200;
          intersection_cur_frame.y        = 200;
          bool has_intersection_cur_frame = false;
          for (auto &edge : iter->second) {
            if (edge.edge.line_points.size() < 2) {
              continue;
            }
            for (size_t i = 1; i < edge.edge.line_points.size(); ++i) {
              cem::fusion::Point2DF tmp;
              if (GetLineIntersection(back_scan_start, back_scan_end, edge.edge.line_points.at(i - 1), edge.edge.line_points.at(i), tmp)) {
                if (intersection_cur_frame.x * intersection_cur_frame.x + intersection_cur_frame.y * intersection_cur_frame.y >
                    tmp.x * tmp.x + tmp.y * tmp.y) {
                  intersection_cur_frame.x   = tmp.x;
                  intersection_cur_frame.y   = tmp.y;
                  has_intersection_cur_frame = true;
                }
              }
            }
          }
          if (has_intersection_cur_frame) {
            if (intersection.x * intersection.x + intersection.y * intersection.y - intersection_cur_frame.x * intersection_cur_frame.x -
                    intersection_cur_frame.y * intersection_cur_frame.y >
                1) {
              intersection.x   = intersection_cur_frame.x;
              intersection.y   = intersection_cur_frame.y;
              has_intersection = true;
              if (iter == cluster.rbegin()) {
                break;
              }
            }
          }
        }
        if (has_intersection) {
          bool valid_intersect = false;
          int  tmp             = CrossProduct(back_scan_start, back_scan_end, end_scan_pt) > 0 ? 1 : -1;
          if (tmp * scan_flag > 0) {
            valid_intersect = true;
          }
          if (valid_intersect) {
            Eigen::Vector2d pt_tmp;
            pt_tmp.x() = intersection.x;
            pt_tmp.y() = intersection.y;
            // AINFO << "ray_id: " << back_ray->angle
            //       << "size: " << back_ray->points.size();
            if (back_ray->points.size() >= 1) {
              // auto &refer_first =
              //     (*(edge.begin() + edge.size() - 2))->end_point;
              auto &refer_first  = edge.front()->end_point;
              auto &refer_second = back_ray->end_point;
              // auto &cur_first = back_ray->points.back();

              if ((refer_second - refer_first).norm() > (pt_tmp - refer_first).norm()) {
                break;
              }

              // if ((refer_second - refer_first).dot(pt_tmp - cur_first) < 0) {
              //   break;
              // }

              // double cos_theta =
              //     (refer_second - refer_first).dot(pt_tmp - cur_first) /
              //     ((refer_second - refer_first).norm() *
              //      (pt_tmp - cur_first).norm());
              // double theta = std::acos(cos_theta);
              // if (cos_theta > M_PI / 4) {
              //   break;
              // }
            }
            back_ray->points.insert(back_ray->points.end(), pt_tmp);
          } else {
            break;
          }
        } else {
          break;
        }
        i += 5;
      }
    }
  }

  if (ray_lines.size() < 30) {
    return;
  }

  for (uint32_t i = 0; i < 360; i += FREESPACE_RESOLUTION) {
    double virtual_range = 0.0;
    if (i <= 15 || i >= 345) {
      virtual_range = 60;
    } else if (i >= 165 && i <= 195) {
      virtual_range = 30;
    } else {
      continue;
    }
    if (lidar_freespace_.count(i) == 0) {
      FreeSpaceRay ray;
      ray.end_point.x() = ray_lines.at(i).direction.x * virtual_range / 200;
      ray.end_point.y() = ray_lines.at(i).direction.y * virtual_range / 200;
      ray.points.push_back(ray.end_point);
      ray.from_seq   = 0;
      ray.from_group = 0;
      lidar_freespace_.emplace(i, std::move(ray));
    }
  }

  {
    uint32_t st = 0;
    for (uint32_t i = 0; i < 360; i += FREESPACE_RESOLUTION) {
      if (lidar_freespace_.count(i) == 0) {
        st = i;
        break;
      }
    }
    std::unordered_map<uint64_t, cem::message::sensor::BevLaneMarker> edges_vis;
    for (uint32_t i = st; i < 360 + st; i += FREESPACE_RESOLUTION) {
      auto i_remainder = i % 360;
      if (lidar_freespace_.count(i_remainder) == 0) {
        continue;
      }
      auto &ray        = lidar_freespace_.at(i_remainder);
      auto  from_group = ray.from_group;
      if (from_group == 0) {
        continue;
      }
      cem::message::sensor::BevLaneMarker *marker = nullptr;
      if (edges_vis.count(from_group) > 0) {
        marker = &edges_vis.at(from_group);
      } else {
        marker     = &edges_vis[from_group];
        marker->id = from_group;
      }
      for (auto &pt : ray.points) {
        cem::fusion::Point2DF point;
        point.x = pt.x();
        point.y = pt.y();
        marker->line_points.push_back(std::move(point));
      }
    }

    for (auto &marker : edges_vis) {
      lidar_cluster_roadedges_.push_back(marker.second);
#ifdef USE_LIDAR_CLUSTER_ROAD_EDGE_ONLY
      input_bev_map_ptr_->edges.push_back(std::move(marker.second));
#endif
    }
  }

  // input_bev_map_ptr_->edges.clear();
  freespace_polyline_.line_points.clear();
  freespace_polyline_.id = 13636513565U;
  for (uint32_t i = 0; i < 360; i += FREESPACE_RESOLUTION) {
    if (lidar_freespace_.count(i) == 0) {
      continue;
    }
    auto &ray = lidar_freespace_.at(i);
    for (auto &pt : ray.points) {
      cem::fusion::Point2DF point;
      point.x = pt.x();
      point.y = pt.y();
      freespace_polyline_.line_points.push_back(std::move(point));
    }
  }

  if (!freespace_polyline_.line_points.empty()) {
    freespace_polyline_.line_points.push_back(freespace_polyline_.line_points.front());
    freespace_polyline_.number_of_points = freespace_polyline_.line_points.size();
    FilterBevRoadEdge(freespace_polyline_);
    // freespace_polyline_.geos = nullptr;
    // freespace_polyline_.geos =
    // std::make_shared<std::vector<Eigen::Vector2f>>();
    // input_bev_map_ptr_->edges.push_back(freespace_polyline_);
  }

  // cem::fusion::Point2DF origin;
  // for (uint32_t i = 0; i < 360; i += FREESPACE_RESOLUTION) {
  //   if (lidar_freespace_.count(i) == 0) {
  //     continue;
  //   }
  //   auto &ray = lidar_freespace_.at(i);
  //   cem::fusion::Point2DF point;
  //   point.x = ray.end_point.x();
  //   point.y = ray.end_point.y();

  //   cem::message::sensor::BevLaneMarker tmp;
  //   tmp.id = lidar_freespace_.at(i).from_group;
  //   tmp.line_points.push_back(origin);
  //   tmp.line_points.push_back(point);
  //   tmp.number_of_points = 2;
  //   input_bev_map_ptr_->edges.push_back(std::move(tmp));
  // }
  input_bev_map_ptr_->edge_num = input_bev_map_ptr_->edges.size();
  lidarMapTime                 = bevMapTime;
  sequence_number++;
}

void BevMapPreProcessor::AddDataIntoLidarRoadEdgeGroup(uint32_t sequence_number) {
  static uint32_t group_id = 1;
  for (auto &edge_cur_frame : lidar_road_edge_ptr_->edges) {
    RoadEdgeInfo edge_info_tmp;
    edge_info_tmp.edge = edge_cur_frame;
    if (!edge_info_tmp.edge.line_points.empty()) {
      edge_info_tmp.total_bbox.ImportPoint(edge_info_tmp.edge.line_points.at(0));
    }
    for (size_t i = 1; i < edge_info_tmp.edge.line_points.size(); ++i) {
      auto &pt_0 = edge_info_tmp.edge.line_points.at(i - 1);
      auto &pt_1 = edge_info_tmp.edge.line_points.at(i);
      edge_info_tmp.total_bbox.ImportPoint(pt_1);
      AABB bbox_tmp(pt_0);
      bbox_tmp.ImportPoint(pt_1);
      edge_info_tmp.segs_bbox.push_back(std::move(bbox_tmp));
    }
    std::vector<uint32_t> belongs_groups_id;
    for (auto &edges_group : hist_lidar_roadedge_group_) {
      bool is_belongs_this_group = false;
      for (auto &group_seq_group : edges_group.second) {
        for (auto &edge_hist : group_seq_group.second) {
          if (IsCurveCloseEnough(edge_info_tmp, edge_hist)) {
            is_belongs_this_group = true;
            break;
          }
        }
        if (is_belongs_this_group) {
          belongs_groups_id.push_back(edges_group.first);
          break;
        }
      }
    }
    if (belongs_groups_id.empty()) {
      while (hist_lidar_roadedge_group_.count(group_id) != 0) {
        group_id++;
      }
      std::map<uint32_t, std::vector<RoadEdgeInfo>> edges;
      edges.emplace(sequence_number, std::vector<RoadEdgeInfo>(1, std::move(edge_info_tmp)));
      hist_lidar_roadedge_group_.emplace(group_id, edges);
    } else {
      auto &merged_group = hist_lidar_roadedge_group_.at(belongs_groups_id.front());
      for (size_t i = 1; i < belongs_groups_id.size(); ++i) {
        for (auto &[key, vec2] : hist_lidar_roadedge_group_.at(belongs_groups_id.at(i))) {
          auto it = merged_group.find(key);
          if (it != merged_group.end()) {
            auto &vec1 = it->second;
            vec1.reserve(vec1.size() + vec2.size());
            std::move(vec2.begin(), vec2.end(), std::back_inserter(vec1));
          } else {
            merged_group.emplace(key, std::move(vec2));
          }
        }
        hist_lidar_roadedge_group_.erase(belongs_groups_id.at(i));
      }
      merged_group[sequence_number].push_back(std::move(edge_info_tmp));
    }
  }
}

bool BevMapPreProcessor::IsCurveCloseEnough(const RoadEdgeInfo &edge1_info, const RoadEdgeInfo &edge2_info) {
  auto &edge1 = edge1_info.edge;
  auto &edge2 = edge2_info.edge;
  if (edge1.line_points.empty() || edge2.line_points.empty()) {
    return false;
  }
  if ((edge1.line_points.size() == 1) && (edge2.line_points.size() == 1)) {
    double dist =
        (edge1.line_points.front().x - edge2.line_points.front().x) * (edge1.line_points.front().x - edge2.line_points.front().x) +
        (edge1.line_points.front().y - edge2.line_points.front().y) * (edge1.line_points.front().y - edge2.line_points.front().y);
    return dist < DIST_THRESHOLD;
  } else if ((edge1.line_points.size() == 1) && (edge2.line_points.size() != 1)) {
    for (size_t i = 1; i < edge2.line_points.size(); ++i) {
      if (DistancePointToSegment(edge1.line_points.front(), edge2.line_points.at(i - 1), edge2.line_points.at(i)) < DIST_THRESHOLD) {
        return true;
      }
    }
  } else if ((edge2.line_points.size() == 1) && (edge1.line_points.size() != 1)) {
    for (size_t i = 1; i < edge1.line_points.size(); ++i) {
      if (DistancePointToSegment(edge2.line_points.front(), edge1.line_points.at(i - 1), edge1.line_points.at(i)) < DIST_THRESHOLD) {
        return true;
      }
    }
  } else {
    if (edge1_info.total_bbox.DistanceTo(edge2_info.total_bbox) < DIST_THRESHOLD) {
      for (size_t i = 1; i < edge1.line_points.size(); ++i) {
        auto &bbox_seg1 = edge1_info.segs_bbox.at(i - 1);
        for (size_t j = 1; j < edge2.line_points.size(); ++j) {
          auto &bbox_seg2 = edge2_info.segs_bbox.at(j - 1);
          if (bbox_seg1.DistanceTo(bbox_seg2) < DIST_THRESHOLD) {
            if (LineSegmentsDistance(edge1.line_points.at(i - 1), edge1.line_points.at(i), edge2.line_points.at(j - 1),
                                     edge2.line_points.at(j)) < DIST_THRESHOLD) {
              return true;
            }
          }
        }
      }
    }
  }
  return false;
}

void BevMapPreProcessor::FilterBevRoadEdge(const cem::message::sensor::BevLaneMarker &freespace) {
  if (freespace.line_points.size() < 3) {
    return;
  }
  cem::fusion::Point2DF origin;
  cem::fusion::Point2DF tmp;
  // std::map<size_t, std::map<size_t, bool>> deactive_mask;
  std::map<size_t, std::map<size_t, bool>> active_mask;
  for (size_t i = 1; i < freespace.line_points.size(); ++i) {
    auto &pt0_at_fs = freespace.line_points.at(i - 1);
    auto &pt1_at_fs = freespace.line_points.at(i);
    for (size_t j = 0; j < input_bev_map_ptr_->edges.size(); ++j) {
      auto &edge = input_bev_map_ptr_->edges.at(j);
      if (edge.id > 100) {
        continue;
      }
      if (active_mask.count(j) == 0) {
        active_mask[j];
      }
      for (size_t k = 0; k < edge.line_points.size(); ++k) {
        if (GetLineIntersection(origin, edge.line_points.at(k), pt0_at_fs, pt1_at_fs, tmp)) {
          active_mask[j][k] = true;
          continue;
        } else {
          if (DistancePointToSegment(edge.line_points.at(k), pt0_at_fs, pt1_at_fs) < 0.1 * 0.1) {
            active_mask[j][k] = true;
            continue;
          }
        }
      }
    }
  }
  auto find_continuous_ranges = [](const std::map<size_t, bool> &input) -> std::map<uint32_t, std::pair<size_t, size_t>> {
    std::map<uint32_t, std::pair<size_t, size_t>> result;
    if (input.size() < 2)
      return result;

    size_t   start = input.begin()->first, prev = input.begin()->first;
    uint32_t index = 0;

    for (auto iter = std::next(input.begin()); iter != input.end(); ++iter) {
      if (iter->first == prev + 1) {
        prev = iter->first;
      } else {
        result[index++] = {start, prev};
        start = prev = iter->first;
      }
    }
    result[index] = {start, prev};  // 最后一段

    return result;
  };
  std::vector<size_t> rm_edge_idxs;
  for (auto &[id, mask] : active_mask) {
    // std::string tmp = "";
    // for (auto iter = mask.begin(); iter != mask.end(); ++iter) {
    //   tmp += std::to_string(iter->first);
    //   tmp += ",";
    // }
    // AINFO << "mask:" << tmp;
    // AINFO << "id:" << input_bev_map_ptr_->edges.at(id).id
    //       << ", size: " <<
    //       input_bev_map_ptr_->edges.at(id).line_points.size();
    auto keep_range = find_continuous_ranges(mask);
    if (keep_range.empty()) {
      rm_edge_idxs.push_back(id);
      continue;
    }
    // AINFO << "keep range size: " << keep_range.size();
    bool is_first_one = true;
    for (auto iter = keep_range.rbegin(); iter != keep_range.rend(); ++iter) {
      if (iter->second.second == iter->second.first) {
        continue;
      }
      if (iter->second.second - iter->second.first == input_bev_map_ptr_->edges.at(id).line_points.size() - 1) {
        continue;
      }
      auto tmp = input_bev_map_ptr_->edges.at(id);
      tmp.line_points.erase(tmp.line_points.begin() + iter->second.second + 1, tmp.line_points.end());
      tmp.line_points.erase(tmp.line_points.begin(), tmp.line_points.begin() + iter->second.first);
      if (!is_first_one) {
        tmp.id = 100 * tmp.id + tmp.id;
      } else {
        rm_edge_idxs.push_back(id);
      }
      // AINFO << tmp.line_points.size();
      input_bev_map_ptr_->edges.push_back(std::move(tmp));
      is_first_one = false;
    }
  }
  // AINFO << rm_edge_idxs.size() << ", " << active_mask.size();
  for (auto iter = rm_edge_idxs.rbegin(); iter != rm_edge_idxs.rend(); ++iter) {
    input_bev_map_ptr_->edges.erase(input_bev_map_ptr_->edges.begin() + (*iter));
  }
  // for (auto &edge : input_bev_map_ptr_->edges) {
  //   AINFO << edge.line_points.size();
  // }
  // AINFO << input_bev_map_ptr_->edges.size();
}

DetectBevMap BevMapPreProcessor::GetGlobalBevMapPtr() {
  return detect_bev_map_;
}

void BevMapPreProcessor::SetRightTurnTrajectory(double timestamp, const std::vector<Eigen::Vector2f> &trajectory) {
  sd_right_turn_trajectory_.valid = false;
  sd_right_turn_trajectory_.trajectory.clear();
  if (!trajectory.empty()) {
    sd_right_turn_trajectory_.valid     = true;
    sd_right_turn_trajectory_.timestamp = timestamp;
    sd_right_turn_trajectory_.trajectory.insert(sd_right_turn_trajectory_.trajectory.end(), trajectory.begin(), trajectory.end());
  }
  // AINFO << std::setprecision(16) << timestamp;
  // AINFO << "RIGHT TURN VALID: " << sd_right_turn_trajectory_.valid;
}

void BevMapPreProcessor::ConstructRightTurnTrajectory() {
  if (input_bev_map_ptr_ == nullptr) {
    return;
  }
  if (!sd_right_turn_trajectory_.valid) {
    return;
  }
  if (sd_right_turn_trajectory_.trajectory.empty()) {
    return;
  }
  if (sd_right_turn_trajectory_.trajectory.front().x() > 20) {
    return;
  }
  if (lidar_freespace_.size() < 30) {
    return;
  }
  cem::message::sensor::BevLaneInfo *ego_lane = nullptr;
  for (auto &lane : input_bev_map_ptr_->lane_infos) {
    /*if ((lane.position ==
         static_cast<uint32_t>(BevLanePosition::LANE_LOC_RIGHT_FIRST)) ||
        (lane.position ==
         static_cast<uint32_t>(BevLanePosition::LANE_LOC_RIGHT_SECOND)) ||
        (lane.position ==
         static_cast<uint32_t>(BevLanePosition::LANE_LOC_RIGHT_THIRD)) ||
        (lane.position ==
         static_cast<uint32_t>(BevLanePosition::LANE_LOC_RIGHT_FOURTH)) ||
        (lane.position ==
         static_cast<uint32_t>(BevLanePosition::LANE_LOC_RIGHT_FIFTH))) {
      return;
    } else*/
    if (lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO)) {
      ego_lane = &lane;
    }
  }
  if (ego_lane != nullptr) {
    if (!ego_lane->line_points.empty()) {
      if ((ego_lane->line_points.back().x > 20) && (ego_lane->line_points.front().x < 1)) {
        return;
      }
    }
    if (!ego_lane->next_lane_ids.empty()) {
      return;
    }
  }
  double bev_timestamp = input_bev_map_ptr_->header.timestamp;
  auto   Tbev_b2w      = FindRealTransform(bev_timestamp);
  auto   Tsdmap_b2w    = FindRealTransform(sd_right_turn_trajectory_.timestamp);
  if (!Tbev_b2w.isApprox(Eigen::Isometry3d::Identity()) && !Tsdmap_b2w.isApprox(Eigen::Isometry3d::Identity())) {
    auto T_sdmap2bev = Tbev_b2w.inverse() * Tsdmap_b2w;
    for (auto &point : sd_right_turn_trajectory_.trajectory) {
      cem::fusion::Point2DF tmp;
      tmp.x = point.x();
      tmp.y = point.y();
      TransformPoint(&tmp, T_sdmap2bev);
      point.x() = tmp.x;
      point.y() = tmp.y;
    }
  }

  if (FittingCenterLine()) {
    auto &added_lane = input_bev_map_ptr_->lane_infos.back();
    ego_lane         = nullptr;
    for (auto &lane : input_bev_map_ptr_->lane_infos) {
      if (lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO) && lane.id != input_bev_map_ptr_->lane_infos.back().id) {
        ego_lane = &lane;
        break;
      }
    }
    if (ego_lane != nullptr) {
      for (auto iter = ego_lane->line_points.begin(); iter != ego_lane->line_points.end();) {
        if (iter->x > added_lane.line_points.front().x) {
          iter = ego_lane->line_points.erase(iter);
        } else {
          iter++;
        }
      }
      if (ego_lane->line_points.size() < 2) {
        auto it = std::find_if(input_bev_map_ptr_->lane_infos.begin(), input_bev_map_ptr_->lane_infos.end(),
                               [&ego_lane](const BevLaneInfo tgt) { return tgt.id == ego_lane->id; });
        if (it != input_bev_map_ptr_->lane_infos.end()) {
          input_bev_map_ptr_->lane_infos.erase(it);
        }

        ego_lane = nullptr;
      }
    }
    if (ego_lane != nullptr) {
      ego_lane->next_lane_ids.clear();
      ego_lane->next_lane_ids.push_back(input_bev_map_ptr_->lane_infos.back().id);
      input_bev_map_ptr_->lane_infos.back().previous_lane_ids.clear();
      input_bev_map_ptr_->lane_infos.back().previous_lane_ids.push_back(ego_lane->id);
    }

    if (!input_bev_map_ptr_->lane_infos.back().line_points.empty()) {
      cem::message::sensor::BevLaneInfo *next_lane = nullptr;
      double                             min_dist  = std::numeric_limits<double>::max();
      cem::fusion::Point2DF              orin;
      orin.x = 0;
      orin.y = 0;
      for (auto iter = input_bev_map_ptr_->lane_infos.begin(); iter != input_bev_map_ptr_->lane_infos.end(); ++iter) {
        if (ego_lane != nullptr) {
          if (ego_lane->id == iter->id) {
            continue;
          }
        }
        if (input_bev_map_ptr_->lane_infos.back().id == iter->id) {
          continue;
        }
        if (iter->line_points.empty()) {
          continue;
        }

        bool first_pt_is_outside = false;
        bool last_pt_is_outside  = false;
        for (size_t i = 1; i < freespace_polyline_.line_points.size(); ++i) {
          cem::fusion::Point2DF tmp;
          if (GetLineIntersection(orin, iter->line_points.front(), freespace_polyline_.line_points.at(i - 1),
                                  freespace_polyline_.line_points.at(i), tmp)) {
            first_pt_is_outside = true;
          }
          if (GetLineIntersection(orin, iter->line_points.back(), freespace_polyline_.line_points.at(i - 1),
                                  freespace_polyline_.line_points.at(i), tmp)) {
            last_pt_is_outside = true;
          }
        }
        if ((!first_pt_is_outside) && last_pt_is_outside) {
          auto tmp_dist = std::numeric_limits<double>::max();
          for (auto pt = iter->line_points.begin(); pt != iter->line_points.end(); ++pt) {
            auto delta_x = input_bev_map_ptr_->lane_infos.back().line_points.back().x - pt->x;
            auto delta_y = input_bev_map_ptr_->lane_infos.back().line_points.back().y - pt->y;
            auto tmp     = delta_x * delta_x + delta_y * delta_y;
            if (tmp > tmp_dist) {
              break;
            } else {
              tmp_dist = tmp;
            }
          }
          if (tmp_dist < min_dist) {
            min_dist  = tmp_dist;
            next_lane = &(*iter);
          }
        }
      }
      if (min_dist < 5 * 5) {
        auto &refer_pt = input_bev_map_ptr_->lane_infos.back().line_points.back();
        for (auto iter_pt = next_lane->line_points.begin(); iter_pt != next_lane->line_points.end();) {
          if (iter_pt->x * iter_pt->x + iter_pt->y * iter_pt->y < refer_pt.x * refer_pt.x + refer_pt.y * refer_pt.y + 4 * 4) {
            if (next_lane->line_points.size() > 2) {
              iter_pt = next_lane->line_points.erase(iter_pt);
            } else {
              ++iter_pt;
            }
          } else {
            ++iter_pt;
          }
        }
        if (!next_lane->line_points.empty()) {
          next_lane->previous_lane_ids.clear();
          next_lane->previous_lane_ids.push_back(input_bev_map_ptr_->lane_infos.back().id);
          input_bev_map_ptr_->lane_infos.back().next_lane_ids.clear();
          input_bev_map_ptr_->lane_infos.back().next_lane_ids.push_back(next_lane->id);
        }
      }
    }
  }

  sd_right_turn_trajectory_.valid = false;
}

bool BevMapPreProcessor::FittingCenterLine() {
  std::unordered_map<uint32_t, std::vector<FreeSpaceRay *>> edges;
  std::vector<uint32_t>                                     id_anticlockwise;
  // uint32_t left_edge_id = 0;

  for (int i = -90; i < 90; ++i) {
    uint32_t angle = static_cast<uint32_t>(i < 0 ? i + 360 : i);
    if (lidar_freespace_.count(angle) == 0) {
      continue;
    }
    auto &ray = lidar_freespace_.at(angle);
    // if ((i <-60) && (left_edge_id==0)){
    //   if (ray.end_point.y > -5) {}
    // }
    std::vector<FreeSpaceRay *> *edge = nullptr;
    if (edges.count(ray.from_group) == 0) {
      id_anticlockwise.push_back(ray.from_group);
      std::vector<FreeSpaceRay *> tmp(1, &ray);
      edges.emplace(ray.from_group, std::move(tmp));
    } else {
      edges.at(ray.from_group).push_back(&ray);
    }
  }

  Eigen::Vector2d front_dir;
  front_dir.x() = 1;
  front_dir.y() = 0;
  std::vector<uint32_t> left_edges;
  std::vector<uint32_t> right_edges;
  for (auto id : id_anticlockwise) {
    auto             edge     = edges.at(id);
    Eigen::Vector2d &first_pt = edge.front()->points.front();
    Eigen::Vector2d &last_pt  = edge.back()->points.back();
    if ((last_pt - first_pt).dot(front_dir) > 0) {
      // if (right_edges.size() == 0) {
      left_edges.push_back(id);
      // }
    } else {
      right_edges.push_back(id);
    }
  }

  if (left_edges.empty()) {
    return false;
  }

  // auto& left_edge = edges.at(left_edges.front());
  // if (left_edge.size() < 2) {
  //   return;
  // }
  if ((edges.at(left_edges.front()).front()->points.front().y() > 0) || (edges.at(left_edges.front()).front()->points.front().y() < -5)) {
    return false;
  }

  std::vector<cem::fusion::Point2DF> left_points;
  for (auto &left_id : left_edges) {
    auto &left_edge = edges.at(left_id);
    for (auto i = 0; i < left_edge.size(); ++i) {
      for (auto &pt : left_edge.at(i)->points) {
        cem::fusion::Point2DF tmp;
        tmp.x = pt.x();
        tmp.y = pt.y();
        left_points.push_back(std::move(tmp));
      }
    }
  }

  std::vector<cem::fusion::Point2DF> right_points;
  for (auto &right_id : right_edges) {
    auto &right_edge = edges.at(right_id);
    for (auto i = 0; i < right_edge.size(); ++i) {
      for (auto &pt : right_edge.at(i)->points) {
        cem::fusion::Point2DF tmp;
        tmp.x = pt.x();
        tmp.y = pt.y();
        right_points.push_back(std::move(tmp));
      }
    }
  }

  struct CounterPair {
    cem::fusion::Point2DF left_point;
    cem::fusion::Point2DF right_point;
    uint32_t              left_id;
    uint32_t              right_id;
  };
  std::vector<CounterPair> counter_points;
  size_t                   last_right = 0;
  for (size_t i = 0; i < left_points.size(); ++i) {
    auto                  &pt_l        = left_points.at(i);
    double                 min_dist    = std::numeric_limits<double>::max();
    cem::fusion::Point2DF *right_point = nullptr;
    for (size_t j = 0; j < right_points.size(); ++j) {
      auto &pt_r    = right_points.at(j);
      auto  delta_x = pt_l.x - pt_r.x;
      auto  delta_y = pt_l.y - pt_r.y;
      auto  dist    = delta_x * delta_x + delta_y * delta_y;
      if (min_dist > dist) {
        right_point = &pt_r;
        min_dist    = dist;
        if (i == left_points.size() - 1) {
          last_right = j;
        }
      }
    }
    if (min_dist < 100) {
      CounterPair pair;
      pair.left_point  = pt_l;
      pair.right_point = *right_point;
      counter_points.push_back(std::move(pair));
    }
  }
  if (last_right > 0) {
    auto &pt_l = left_points.back();
    for (auto i = last_right; i != 0; --i) {
      auto &pt_r    = right_points.at(i - 1);
      auto  delta_x = pt_l.x - pt_r.x;
      auto  delta_y = pt_l.y - pt_r.y;
      auto  dist    = delta_x * delta_x + delta_y * delta_y;
      if (dist < 100) {
        CounterPair pair;
        pair.left_point  = pt_l;
        pair.right_point = pt_r;
        counter_points.push_back(std::move(pair));
      }
    }
  }
  cem::message::sensor::BevLaneInfo center_line;
  // cem::message::sensor::BevLaneMarker edge_debug;
  // edge_debug.id = 66;
  for (uint32_t i = 0; i < counter_points.size(); i++) {
    auto                 &counter_pt = counter_points.at(i);
    cem::fusion::Point2DF point;
    point.x = (counter_pt.left_point.x + counter_pt.right_point.x) * 0.5;
    point.y = (counter_pt.left_point.y + counter_pt.right_point.y) * 0.5;

    if (center_line.line_points.empty()) {
      // edge_debug.line_points.push_back(point);
      center_line.line_points.push_back(std::move(point));
    } else {
      if (sqrt(center_line.line_points.back().x * center_line.line_points.back().x +
               center_line.line_points.back().y * center_line.line_points.back().y) +
              0.2 <
          sqrt(point.x * point.x + point.y * point.y)) {
        // edge_debug.line_points.push_back(point);
        center_line.line_points.push_back(std::move(point));
      }
    }
  }
  if (center_line.line_points.size() < 2) {
    return false;
  }
  std::unordered_map<uint64_t, uint32_t> id_map;
  for (auto &lane : input_bev_map_ptr_->lane_infos) {
    id_map.emplace(lane.id, 381558865);
  }
  static uint64_t virtual_id = 66;
  while (id_map.count(virtual_id) > 0) {
    virtual_id++;
    virtual_id %= 100;
  }

  center_line.id               = virtual_id;
  center_line.number_of_points = center_line.line_points.size();
  center_line.lane_type        = BevLaneType::LANE_TYPE_UNKNOWN;
  // if (!left_points.empty()) {
  //   cem::message::sensor::BevLaneMarker line_left;
  //   line_left.id = 121;
  //   line_left.line_points.insert(line_left.line_points.end(),
  //                                left_points.begin(), left_points.end());
  //   line_left.number_of_points = left_points.size();
  //   input_bev_map_ptr_->edges.push_back(std::move(line_left));
  // }
  // if (!right_points.empty()) {
  //   cem::message::sensor::BevLaneMarker line_right;
  //   line_right.id = 121;
  //   line_right.line_points.insert(line_right.line_points.end(),
  //                                 right_points.rbegin(),
  //                                 right_points.rend());
  //   line_right.number_of_points = right_points.size();
  //   input_bev_map_ptr_->edges.push_back(std::move(line_right));
  // }
  // input_bev_map_ptr_->edges.push_back(std::move(edge_debug));
  input_bev_map_ptr_->lane_infos.push_back(std::move(center_line));
  return true;
}







void BevMapPreProcessor::SetTraverseCrossWalkLaneInfo(
    std::vector<traverseCrossWalkLane>& crosswalk_lane_list) {
  crosswalk_lane_list_.clear();
  crosswalk_lane_list_ = crosswalk_lane_list;
}

}  // namespace fusion
}  // namespace cem
