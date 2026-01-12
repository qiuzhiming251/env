#include "lib/localmap_construction/traffic_light_mapping.h"
#include <log_custom.h>
#include <cyber/time/clock.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <string_view>
#include <tuple>
#include <type_traits>

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cyber/common/log.h>
#include <localmap_construction/traffic_light_common.h>
#include <localmap_construction/traffic_light_map_topo.h>
#include <math/math.h>
#include <message/common/geometry.h>
#include <message/internal_message.h>
#include <message/sensor/vision/tsrmobject.h>
#include <perception_and_ld_map_fusion/data_fusion/geometry_match_info.h>
#include <sd_navigation/SDMapElementExtract.h>
#include "fmt/core.h"
#include "fmt/ranges.h"
#include "lib/message/env_model/routing_map/routing_map.h"

#include "Eigen/src/Core/ArrayWrapper.h"
#include "Eigen/src/Core/Matrix.h"
#include "base/sensor_data_manager.h"
#include "fmt/format.h"
#include "lib/common/utility.h"
#include "magic_enum/magic_enum.hpp"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/perception/env/src/lib/base/params_manager/params_manager.h"
#include "modules/perception/env/src/lib/localmap_construction/cloud_config_traffic_light.h"

namespace cem {
namespace fusion {

using byd::common::math::LineSegment2d;
using byd::common::math::Vec2d;

constexpr int max_stay_frames_with_info = 5;
constexpr int max_stay_frames_no_info   = 3;

inline LineSegment2d TranslateSegmentAlongAxisStart(const LineSegment2d &axis, const LineSegment2d &seg1, double distance) {
  const Vec2d translation = axis.unit_direction() * -distance;
  return {seg1.start() + translation, seg1.end() + translation};
}

void TrafficLightMapping::StayPrevTrafficLightSet() {
  auto SaveObj = [&](const TrafficLight &light) {
    if (IsValid(light)) {
      deal_per_traffic_light_objects_.insert({light.traffic_obj_info->id, *light.traffic_obj_info});
    }
  };
  SaveObj(traffic_lights_.left);
  SaveObj(traffic_lights_.u_turn);
  SaveObj(traffic_lights_.straight);
  SaveObj(traffic_lights_.right);
}

void TrafficLightMapping::StayPreviousState(const TrafficLight &previous_light, TrafficLight &traffic_light) {
  if (previous_light.is_valid && !traffic_light.is_valid && previous_light.stay_prev_counter < max_stay_frames_no_info) {
    traffic_light = previous_light;
    traffic_light.stay_prev_counter++;
    traffic_light.traffic_reason = byd::msg::orin::routing_map::LaneInfo::STAY_PREV;
  }
}

void TrafficLightMapping::Process(const RoutingMapPtr &routing_map_input, const RoutingMapPtr &routing_map_output,
                                  const BevMapInfoPtr &bev_map, const std::shared_ptr<navigation::SdNavigationCity> &sd_navigation_city,
                                  BevMapInfoConstPtr &bev_raw, const Eigen::Isometry3d &Tbw) {
  // TRAFFIC_LOG << "----------------------------\n\n";
  TRAFFIC_REC_LOG << "--------------------------------";
  double timestamp     = GetMwTimeNowSec();
  info_traffic_lights_ = "";
  traffic_light_e2e_   = std::make_shared<cem::message::env_model::TrafficLightsE2EInfo>();

  auto Tbw_t = Tbw;

  std::string cloud_debug_info;
  cloud_traffic_light_ = cem::fusion::claud_traffic::GetCloudTrafficLights(bev_map, cloud_debug_info, &Tbw_t);
  CLOUD_TRAFFIC_LOG << "cloud_traffic_light_______________________end______\n\n\n";

  {
    Reset();
    sd_navigation_city_ = sd_navigation_city;
    routing_map_ptr_    = routing_map_input;
    FillPerceptionTrafficLights();
    SetTrafficLightColor();

    SetLightsTurn(straight_shapes_, LightTurn::STRAIGHT);
    SetLightsTurn(left_shapes_, LightTurn::LEFT);
    SetLightsTurn(right_shapes_, LightTurn::RIGHT);
    SetLightsTurn(uturn_shapes_, LightTurn::UTURN);
    /*
    bool valid_transform = TransformPostionOfObjs(timestamp);
    // FilterLightsByPosition();
    if (valid_transform && !perception_traffic_lights_.empty() && perception_traffic_light_info_) {
      SetTrafficLights(LightTurn::STRAIGHT, straight_shapes_, traffic_lights_previous_.straight, traffic_lights_.straight
                       );
      SetTrafficLights(LightTurn::LEFT, left_shapes_, traffic_lights_previous_.left, traffic_lights_.left);
      SetTrafficLights(LightTurn::UTURN, uturn_shapes_, traffic_lights_previous_.u_turn, traffic_lights_.u_turn);
      SetTrafficLights(LightTurn::RIGHT, right_shapes_, traffic_lights_previous_.right, traffic_lights_.right);

      for (const auto &obj : perception_traffic_lights_) {
        deal_per_traffic_light_objects_.insert({obj.id, obj});
        std::vector<std::string> turn_strs;
        for (const auto turn : obj.turn) {
          turn_strs.emplace_back(magic_enum::enum_name(turn));
        }
        TRAFFIC_REC_LOG << fmt::format("final_obj_id:{}  shape:{} color:{} flash:{:d}  turn:{} num:{}", obj.id,
                                       magic_enum::enum_name(obj.attributes.traffic_light_shape),
                                       magic_enum::enum_name(obj.attributes.traffic_light_color), obj.attributes.traffic_light_flashing,
                                       turn_strs, obj.attributes.traffic_light_num);
      }
    } else if (traffic_lights_previous_.straight.is_valid || traffic_lights_previous_.left.is_valid ||
               traffic_lights_previous_.u_turn.is_valid || traffic_lights_previous_.right.is_valid) {
      TRAFFIC_REC_LOG << fmt::format("stay_prev_state.");
      StayPreviousState(traffic_lights_previous_.straight, traffic_lights_.straight);
      StayPreviousState(traffic_lights_previous_.left, traffic_lights_.left);
      StayPreviousState(traffic_lights_previous_.u_turn, traffic_lights_.u_turn);
      StayPreviousState(traffic_lights_previous_.right, traffic_lights_.right);
      StayPrevTrafficLightSet();
    }
    */

    traffic_light_descern_.Proc(routing_map_input, bev_raw, bev_map, Tbw);
    traffic_lights_                 = traffic_light_descern_.GetTrafficLights();
    deal_per_traffic_light_objects_ = traffic_light_descern_.GetDealObjMap();
    IsRightExit();

    if ((traffic_lights_previous_.straight.is_valid && !traffic_lights_.straight.is_valid) ||
        (traffic_lights_previous_.left.is_valid && !traffic_lights_.left.is_valid) ||
        (traffic_lights_previous_.u_turn.is_valid && !traffic_lights_.u_turn.is_valid) ||
        (traffic_lights_previous_.right.is_valid && !traffic_lights_.right.is_valid)) {
      NEW_LOG << fmt::format("stay_prev_state.");
      StayPreviousState(traffic_lights_previous_.straight, traffic_lights_.straight);
      StayPreviousState(traffic_lights_previous_.left, traffic_lights_.left);
      StayPreviousState(traffic_lights_previous_.u_turn, traffic_lights_.u_turn);
      StayPreviousState(traffic_lights_previous_.right, traffic_lights_.right);
      StayPrevTrafficLightSet();
    }

    NEW_LOG << fmt::format("prev_dis_to_junction:{:.2f}", distance_to_junction_prev_);
    IsLightBlockFailed(traffic_lights_previous_.left, traffic_lights_.left);
    IsLightBlockFailed(traffic_lights_previous_.u_turn, traffic_lights_.u_turn);
    IsLightBlockFailed(traffic_lights_previous_.right, traffic_lights_.right);
    IsLightBlockFailed(traffic_lights_previous_.straight, traffic_lights_.straight);
    traffic_lights_previous_ = traffic_lights_;
    NEW_LOG << fmt::format("traffic_straight {}", traffic_lights_.straight);
    NEW_LOG << fmt::format("traffic_left     {}", traffic_lights_.left);
    NEW_LOG << fmt::format("traffic_uturn    {}", traffic_lights_.u_turn);
    NEW_LOG << fmt::format("traffic_right    {}", traffic_lights_.right);
    NEW_LOG << "------------------------------end-----------\n\n";
  }

  if (cloud_traffic_light_) {
    info_traffic_lights_ =
        fmt::format(" cloud_junc_id:{} type:{} ", cloud_traffic_light_->junction_tail_id, cloud_traffic_light_->junction_cloud_type);
    CLOUD_TRAFFIC_LOG << info_traffic_lights_;
    traffic_lights_ = ChooseBestTrafficLights(*cloud_traffic_light_, traffic_lights_);
  }

  SetAllLightStatus();

  SetTrafficLightInfo();

  auto traffic_sd = traffic_lights_;
  SetTrafficLight(routing_map_input, routing_map_output);
  // traffic_lights_ = traffic_sd;

  SetSrTrafficLight(routing_map_output);

  BindingTrafficLightToBev(bev_map);

  CollectAllLightLanes(routing_map_output, bev_map);

  info_traffic_lights_ += " " + cloud_debug_info;

  TRAFFIC_REC_LOG << "\n\n ";

  time_previous_ = timestamp;
}

void TrafficLightMapping::CollectAllLightLanes(const RoutingMapPtr &routing_map_output, const BevMapInfoPtr &bev_map) {
  // skip bev;
  if (!routing_map_output) {
    return;
  }
  std::map<uint64_t, LaneInfo *> routing_lanes;
  for (auto &lane_t : routing_map_output->lanes) {
    routing_lanes.insert({lane_t.id, &lane_t});
    if (lane_t.light_status != LightStatus::NONE_LIGHT) {
      ld_map_lanes_light_.insert(lane_t.id);
    }
  }
  for (auto it = ld_map_lanes_light_.begin(); it != ld_map_lanes_light_.end();) {
    auto it_lane = routing_lanes.find(*it);
    // erase unexist lane
    if (it_lane == routing_lanes.end()) {
      it = ld_map_lanes_light_.erase(it);
      continue;
    }
    if (it_lane->second->light_status == LightStatus::NONE_LIGHT) {
      it_lane->second->light_info.prev_has_light_now_no = true;
    }
    it++;
  }
}

void TrafficLightMapping::MatchLightAndJunction(const RoutingMapPtr &routing_map) {
  if (ldmap_junction_ids_.empty() && maybe_junctions_.empty()) {
    return;
  };

  std::map<uint64_t, std::set<uint64_t>> junction_lights;

  for (const auto &obj : perception_traffic_lights_) {
    double   min_junction_dis = std::numeric_limits<double>::infinity();
    uint64_t min_junction_id  = 0;
    double   min_section_dis  = std::numeric_limits<double>::infinity();
    uint64_t min_section_id   = 0;

    std::map<uint64_t, double> junc_dis;
    for (const auto &jun_polygon : ldmap_junction_ids_) {
      if (jun_polygon.second.points().size() < 3) {
        continue;
      }
      Vec2d  light_pos{obj.position.x, obj.position.y};
      double dis_polygon_traffic_pos = jun_polygon.second.DistanceTo(light_pos);

      if (dis_polygon_traffic_pos < lane_virtual_dis_threshold) {
        junc_dis.insert({jun_polygon.first, dis_polygon_traffic_pos});
        if (min_junction_dis > dis_polygon_traffic_pos) {
          min_junction_dis = dis_polygon_traffic_pos;
          min_junction_id  = jun_polygon.first;
        }
      }
      LD_TRAFFIC_LOG << fmt::format("obj_id:{} jun_id:{}  dis:{:.2f}  min_junction_id:{}  dis:{:.2f}", obj.id, jun_polygon.first,
                                    dis_polygon_traffic_pos, min_junction_id, min_junction_dis);
    }

    const auto &lanes    = routing_map->lanes;
    const auto &sections = routing_map->route.sections;
    for (const auto &sec_polygon : maybe_junctions_) {
      if (sec_polygon.second.points().size() < 3) {
        continue;
      }
      auto it_sec =
          std::find_if(sections.begin(), sections.end(), [&sec_polygon](const SectionInfo &rhs) { return rhs.id == sec_polygon.first; });
      if (it_sec == sections.end()) {
        LD_TRAFFIC_LOG << "no_find_section:" << sec_polygon.first;
        continue;
      }
      bool is_valid = false;
      for (uint64_t lane_id : it_sec->lane_ids) {
        auto it_lane = std::find_if(lanes.begin(), lanes.end(), [&lane_id](const auto &lane_t) { return lane_t.id == lane_id; });
        if (it_lane == lanes.end() || it_lane->type == LaneType::LANE_NON_MOTOR) {
          continue;
        }
        is_valid = LaneStoplineTrafficLight(sec_polygon.first, *it_lane, -50.0, obj);
        if (is_valid) {
          break;
        }
      }

      Vec2d  light_pos{obj.position.x, obj.position.y};
      double dis_polygon_traffic_pos = sec_polygon.second.DistanceTo(light_pos);

      if (dis_polygon_traffic_pos < lane_virtual_dis_threshold && is_valid) {
        junc_dis.insert({sec_polygon.first, dis_polygon_traffic_pos});
        if (min_section_dis > dis_polygon_traffic_pos) {
          min_section_dis = dis_polygon_traffic_pos;
          min_section_id  = sec_polygon.first;
        }
      }

      LD_TRAFFIC_LOG << fmt::format("obj_id:{}  sec_id:{}  dis:{:.2f}  min_section_id:{}  dis:{:.2f} is_valid:{:d}", obj.id,
                                    sec_polygon.first, dis_polygon_traffic_pos, min_section_id, min_section_dis, is_valid);
    }

    if (min_junction_id == 0 && min_section_id == 0) {
      continue;
    }

    if (min_junction_id != 0 && min_section_id == 0) {
      junction_light_ids_[min_junction_id].insert(obj.id);
      continue;
    }
    if (min_junction_id == 0 && min_section_id != 0) {
      section_light_ids_[min_section_id].insert(obj.id);
      continue;
    }
    if (min_junction_id != 0 && min_section_id != 0) {
      if (min_junction_dis < min_section_dis) {
        junction_light_ids_[min_junction_id].insert(obj.id);
        continue;
      }
      section_light_ids_[min_section_id].insert(obj.id);
      continue;
    }
  }
  for (const auto &val : junction_light_ids_) {
    LD_TRAFFIC_LOG << fmt::format("junction_id:{}  light_ids:{}", val.first, val.second);
  }
  for (const auto &val : section_light_ids_) {
    LD_TRAFFIC_LOG << fmt::format("section_id:{}  light_ids:{}", val.first, val.second);
  }
}

bool TrafficLightMapping::IsSameColor(const std::vector<TrfObjectInfo> &qual_obj_temp) {
  if (qual_obj_temp.empty()) {
    return true;
  }
  auto first_color    = qual_obj_temp[0].attributes.traffic_light_color;
  bool all_same_color = std::all_of(qual_obj_temp.begin(), qual_obj_temp.end(), [first_color](const TrfObjectInfo &rhs) {
    if (first_color == message::sensor::TLC_RED || first_color == message::sensor::TLC_RED_FLASHING) {
      return rhs.attributes.traffic_light_color == message::sensor::TLC_RED ||
             rhs.attributes.traffic_light_color == message::sensor::TLC_RED_FLASHING;
    }
    if (first_color == message::sensor::TLC_YELLOW || first_color == message::sensor::TLC_YELLOW_FLASHING) {
      return rhs.attributes.traffic_light_color == message::sensor::TLC_YELLOW ||
             rhs.attributes.traffic_light_color == message::sensor::TLC_YELLOW_FLASHING;
    }
    if (first_color == message::sensor::TLC_GREEN || first_color == message::sensor::TLC_GREEN_FLASHING) {
      return rhs.attributes.traffic_light_color == message::sensor::TLC_GREEN ||
             rhs.attributes.traffic_light_color == message::sensor::TLC_GREEN_FLASHING;
    }
    return false;
  });
  return all_same_color;
};
const auto GetMinLaterval = [](const TransformParams &trans_para, const std::vector<TrfObjectInfo> &quality_objs) {
  std::vector<std::pair<const TrfObjectInfo *, double>> quality_obj_dis;
  for (const auto &obj : quality_objs) {
    Point3DD obj_pos{obj.position.x, obj.position.y, 0.0};
    auto     obj_new_pos = TransformCoordinate(obj_pos, trans_para);
    quality_obj_dis.emplace_back(&obj, std::fabs(obj_new_pos.y + trans_para.width / 2.0));
  }
  std::sort(quality_obj_dis.begin(), quality_obj_dis.end(), [](const auto &lhs, const auto &rhs) { return lhs.second < rhs.second; });
  return quality_obj_dis;
};

void TrafficLightMapping::GetTrafficArrow(const std::vector<TrfObjectInfo>         &traffic_lights_vec,
                                          const std::vector<TrafficLightShapeType> &shapes, const TrafficLight &traffic_light_prev,
                                          TrafficLight &traffic_light_tmp) {
  std::vector<TrfObjectInfo> quality_objs;
  if (traffic_light_tmp.is_valid) {
    return;
  }
  std::vector<uint64_t> obj_ids;
  for (const auto &shape : shapes) {
    for (const auto &obj : traffic_lights_vec) {
      if (obj.attributes.traffic_light_shape == shape) {
        quality_objs.push_back(obj);
        obj_ids.push_back(obj.id);
      }
    }
  }
  std::size_t qual_size = quality_objs.size();
  if (qual_size == 0) {
    return;
  }
  LD_TRAFFIC_LOG << fmt::format("turn_light:{} shape:{} obj_ids:{}", magic_enum::enum_name(traffic_light_tmp.turn_type),
                                magic_enum::enum_name(shapes.front()), obj_ids);
  if (qual_size == 1) {
    LD_TRAFFIC_LOG << "only_one_obj:" << quality_objs[0].id;
    SetLightT(quality_objs.front(), traffic_light_tmp);
    return;
  }
  std::sort(quality_objs.begin(), quality_objs.end(),
            [](const TrfObjectInfo &lhs, const TrfObjectInfo &rhs) { return lhs.direction_conf > rhs.direction_conf; });
  if (quality_objs[0].direction_conf - quality_objs[1].direction_conf > 0.2) {
    SetLightT(quality_objs.front(), traffic_light_tmp);
    return;
  }
  LD_TRAFFIC_LOG << fmt::format("max_direction_id:{}  val:{}", quality_objs[0].id, quality_objs[0].direction_conf);
  if (quality_objs[0].direction_conf > 1 - k_epsilon) {
    quality_objs.erase(std::remove_if(quality_objs.begin(), quality_objs.end(),
                                      [](const TrfObjectInfo &rhs) { return rhs.direction_conf < 1 - k_epsilon; }),
                       quality_objs.end());
  }
  SetLightT(quality_objs.front(), traffic_light_tmp);
  if (quality_objs.size() == 1) {
    LD_TRAFFIC_LOG << "set_default_id:" << quality_objs.front().id << " same_color size:" << quality_objs.size();
    return;
  }
  LD_TRAFFIC_LOG << "set_default_id:" << quality_objs.front().id;

  std::sort(quality_objs.begin(), quality_objs.end(),
            [](const TrfObjectInfo &lhs, const TrfObjectInfo &rhs) { return lhs.track_age > rhs.track_age; });
  auto     qual_obj_tmp_02 = quality_objs;
  uint64_t max_track_age   = quality_objs.front().track_age;
  qual_obj_tmp_02.erase(
      std::remove_if(qual_obj_tmp_02.begin(), qual_obj_tmp_02.end(),
                     [max_track_age](const TrfObjectInfo &rhs) { return max_track_age > rhs.track_age + 60 && rhs.track_age < 40; }),
      qual_obj_tmp_02.end());
  if (qual_obj_tmp_02.size() == 1 || (!qual_obj_tmp_02.empty() && IsSameColor(qual_obj_tmp_02))) {
    for (const auto &obj : qual_obj_tmp_02) {
      LD_TRAFFIC_LOG << "chose_age_id:" << obj.id << " track_age:" << obj.track_age;
    }
    if (junction_orin_vertical_) {
      auto val = GetMinLaterval(*junction_orin_vertical_, qual_obj_tmp_02);
      LD_TRAFFIC_LOG << "choose_obj_based_on_vertical_lateral.";
      SetLightT(*val.front().first, traffic_light_tmp);
      return;
    }
    LD_TRAFFIC_LOG << "choose_obj_based_on_track_age. or same_color.  quality_objs_size:" << quality_objs.size()
                   << "  qual_tmp_size:" << qual_obj_tmp_02.size();
    SetLightT(quality_objs.front(), traffic_light_tmp);
    return;
  }

  std::vector<std::pair<const TrfObjectInfo *, uint64_t>> obj_map_id;
  for (const auto &obj : quality_objs) {
    obj_map_id.emplace_back(&obj, GetHisTimes(traffic_light_tmp.turn_type, obj.id));
  }
  std::sort(obj_map_id.begin(), obj_map_id.end(), [](const auto &lhs, const auto &rhs) { return lhs.second > rhs.second; });
  for (const auto &[obj, times] : obj_map_id) {
    LD_TRAFFIC_LOG << fmt::format("obj_id:{}  time:{}", obj->id, times);
  }
  if (obj_map_id[0].second > 2 && obj_map_id[0].second > obj_map_id[1].second) {
    LD_TRAFFIC_LOG << "choose_obj_base_on_history_times.";
    SetLightT(*obj_map_id[0].first, traffic_light_tmp);
    return;
  }

  auto qual_bak = quality_objs;
  if (traffic_light_prev.traffic_obj_info) {
    qual_bak.erase(std::remove_if(qual_bak.begin(), qual_bak.end(),
                                  [&](const TrfObjectInfo &rhs) {
                                    double dx = traffic_light_prev.traffic_obj_info->position.x - rhs.position.x;
                                    double dy = traffic_light_prev.traffic_obj_info->position.y - rhs.position.y;
                                    return std::sqrt(dx * dx + dy * dy) > 12.0;
                                  }),
                   qual_bak.end());
    if (qual_bak.size() == 1 || (!qual_bak.empty() && IsSameColor(qual_bak))) {
      LD_TRAFFIC_LOG << "choose_obj_base_on_prev_id_pos. or_same_color";
      SetLightT(qual_bak.front(), traffic_light_tmp);
      return;
    }
  }

  std::sort(quality_objs.begin(), quality_objs.end(),
            [](const TrfObjectInfo &lhs, const TrfObjectInfo &rhs) { return lhs.shape_conf > rhs.shape_conf; });
  if (quality_objs[0].shape_conf > quality_objs[1].shape_conf + 0.15) {
    LD_TRAFFIC_LOG << "choose_obj_based_on_shape_conf.";
    SetLightT(quality_objs.front(), traffic_light_tmp);
    return;
  }
  if (junction_orin_vertical_) {
    auto val = GetMinLaterval(*junction_orin_vertical_, quality_objs);
    LD_TRAFFIC_LOG << "choose_obj_based_on_vertical_lateral.";
    SetLightT(*val.front().first, traffic_light_tmp);
    return;
  }
  if (junction_orin_connect_) {
    LD_TRAFFIC_LOG << "choose_obj_based_on_connect_lateral.";
    auto val = GetMinLaterval(*junction_orin_connect_, quality_objs);
    SetLightT(*val.front().first, traffic_light_tmp);
    return;
  }
  if (section_orin_vertical_) {
    LD_TRAFFIC_LOG << "choose_obj_based_on_vertical_lateral.";
    auto val = GetMinLaterval(*section_orin_vertical_, quality_objs);
    SetLightT(*val.front().first, traffic_light_tmp);
    return;
  }
  std::sort(quality_objs.begin(), quality_objs.end(),
            [](const TrfObjectInfo &lhs, const TrfObjectInfo &rhs) { return std::fabs(lhs.position.y) < std::fabs(rhs.position.y); });
  LD_TRAFFIC_LOG << "choose_obj_based_on_lateral.";
  SetLightT(quality_objs.front(), traffic_light_tmp);
  // return;

  // traffic_light_tmp.color          = cem::message::sensor::TrafficLightColorType::TLC_BLURRING_MODE;
  // traffic_light_tmp.traffic_reason = byd::msg::orin::routing_map::LaneInfo::NEW_BLURRING_COLOR;
};

void TrafficLightMapping::GetTrafficLightsLd(const std::optional<byd::common::math::Polygon2d> &junction_polygon,
                                             const std::vector<TrfObjectInfo>                  &traffic_lights_vec) {
  GetTrafficArrow(traffic_lights_vec, uturn_shapes_, traffic_lights_prev_ld_.u_turn, traffic_lights_ld_.u_turn);
  GetTrafficArrow(traffic_lights_vec, left_shapes_, traffic_lights_prev_ld_.left, traffic_lights_ld_.left);
  GetTrafficArrow(traffic_lights_vec, straight_shapes_, traffic_lights_prev_ld_.straight, traffic_lights_ld_.straight);
  GetTrafficArrow(traffic_lights_vec, right_shapes_, traffic_lights_prev_ld_.right, traffic_lights_ld_.right);
  GetTrafficArrow(traffic_lights_vec, {message::sensor::TrafficLightShapeType::TLS_LEFT_ARROW}, traffic_lights_prev_ld_.u_turn,
                  traffic_lights_ld_.u_turn);
  GetTrafficArrow(traffic_lights_vec, {message::sensor::TrafficLightShapeType::TLS_CIRCULAR}, traffic_lights_prev_ld_.u_turn,
                  traffic_lights_ld_.u_turn);
  GetTrafficArrow(traffic_lights_vec, {message::sensor::TrafficLightShapeType::TLS_CIRCULAR}, traffic_lights_prev_ld_.left,
                  traffic_lights_ld_.left);
  GetTrafficArrow(traffic_lights_vec, {message::sensor::TrafficLightShapeType::TLS_CIRCULAR}, traffic_lights_prev_ld_.straight,
                  traffic_lights_ld_.straight);
}

std::optional<LaneInfo> GetEdgeLane(const std::vector<LaneInfo> &lanes, bool is_left, const std::vector<uint64_t> &lane_ids_tmp) {
  if (lane_ids_tmp.empty()) {
    return std::nullopt;
  }
  std::vector<LaneInfo> edges_lanes;
  for (const auto &lane_id : lane_ids_tmp) {
    auto it_lane = std::find_if(lanes.begin(), lanes.end(), [lane_id, is_left](const LaneInfo &rhs) {
      return rhs.id == lane_id && ((is_left && rhs.left_lane_id == 0) || (!is_left && rhs.right_lane_id == 0));
    });
    if (it_lane == lanes.end()) {
      continue;
    }
    edges_lanes.push_back(*it_lane);
  }
  if (edges_lanes.empty()) {
    return std::nullopt;
  }
  if (edges_lanes.size() == 1) {
    return edges_lanes.front();
  }
  auto it_lane = std::find_if(edges_lanes.begin(), edges_lanes.end(), [&lane_ids_tmp, is_left](const LaneInfo &rhs) {
    return (is_left && lane_ids_tmp.front() == rhs.id) || (!is_left && lane_ids_tmp.back() == rhs.id);
  });
  if (it_lane != edges_lanes.end()) {
    return *it_lane;
  }
  return std::nullopt;
};

void TrafficLightMapping::GetPolygonFromJunction(const RoutingMapPtr &routing_map, uint64_t junction_id,
                                                 std::vector<uint64_t> *const prev_lanes_ptr) {
  if (routing_map == nullptr) {
    return;
  }
  int target_junction_idx = -1;

  const auto &lanes          = routing_map->lanes;
  const auto &route_sections = routing_map->route.sections;
  if (route_sections.empty()) {
    return;
  }
  bool junction_no_turn = true;
  for (std::size_t idx = 0; idx < route_sections.size(); idx++) {
    for (const auto lane_id : routing_map->route.sections[idx].lane_ids) {
      auto it_lane = std::find_if(lanes.begin(), lanes.end(),
                                  [&](const LaneInfo &rhs) { return rhs.id == lane_id && rhs.junction_id == junction_id; });
      if (it_lane != lanes.end()) {
        target_junction_idx = static_cast<int>(idx);
        junction_no_turn    = it_lane->turn_type == TurnType::NO_TURN;
      }
    }
    if (target_junction_idx != -1 && !route_sections[target_junction_idx].lane_ids.empty()) {
      break;
    }
  }
  if (target_junction_idx == -1) {
    LD_TRAFFIC_LOG << "target_junction_idx:" << target_junction_idx;
    return;
  }
  LD_TRAFFIC_LOG << fmt::format("junction_idx:{}  is_no_turn:{:d}", target_junction_idx, junction_no_turn);

  int                   prev_junction_idx = -1;
  int                   next_junction_idx = -1;
  std::vector<uint64_t> prev_lanes;
  std::vector<uint64_t> next_lanes;
  std::vector<uint64_t> junction_lanes;

  if (target_junction_idx > 0) {
    prev_junction_idx = target_junction_idx - 1;
  }
  if (junction_no_turn) {
    for (int idx_t = target_junction_idx + 1; idx_t < static_cast<int>(route_sections.size()); idx_t++) {
      for (uint64_t id_t : route_sections[idx_t].lane_ids) {
        auto it_lane =
            std::find_if(lanes.begin(), lanes.end(), [id_t](const LaneInfo &rhs) { return id_t == rhs.id && rhs.junction_id == 0; });
        if (it_lane != lanes.begin()) {
          next_junction_idx = idx_t;
          break;
        }
      }
      if (next_junction_idx != -1) {
        break;
      }
    }
  } else if (prev_junction_idx != -1) {
    for (uint64_t succ_section_id : route_sections[prev_junction_idx].successor_section_id_list) {
      std::for_each(lanes.begin(), lanes.end(), [&](const LaneInfo &rhs) {
        if (rhs.section_id == succ_section_id && rhs.junction_id == junction_id && rhs.turn_type == TurnType::NO_TURN) {
          junction_lanes.emplace_back(rhs.id);
        }
      });
    }
  }

  std::vector<Vec2d> points;

  if (prev_junction_idx != -1) {
    prev_lanes = route_sections[prev_junction_idx].lane_ids;
  }
  if (next_junction_idx != -1) {
    next_lanes = route_sections[next_junction_idx].lane_ids;
  }
  if (junction_no_turn) {
    junction_lanes = route_sections[target_junction_idx].lane_ids;
  }
  if (prev_lanes_ptr != nullptr) {
    *prev_lanes_ptr = prev_lanes;
  }

  auto prev_left_lane      = GetEdgeLane(lanes, true, prev_lanes);
  auto prev_right_lane     = GetEdgeLane(lanes, false, prev_lanes);
  auto junction_left_lane  = GetEdgeLane(lanes, true, junction_lanes);
  auto junction_right_lane = GetEdgeLane(lanes, false, junction_lanes);
  auto next_left_lane      = GetEdgeLane(lanes, true, next_lanes);
  auto next_right_lane     = GetEdgeLane(lanes, false, next_lanes);
  LD_TRAFFIC_LOG << fmt::format("prev_lane:{}  junction_lane:{} next_lane:{}", prev_lanes.size(), junction_lanes.size(), next_lanes.size());
  if (!junction_left_lane || !junction_right_lane || junction_left_lane->points.empty() || junction_right_lane->points.empty()) {
    if (prev_left_lane && prev_right_lane && !prev_left_lane->points.empty() && !prev_right_lane->points.empty()) {
      Vec2d start_left_point{prev_left_lane->points.front().x, prev_left_lane->points.front().y};
      Vec2d start_right_point{prev_right_lane->points.front().x, prev_right_lane->points.front().y};
      Vec2d back_left_point{prev_left_lane->points.back().x, prev_left_lane->points.back().y};
      Vec2d back_right_point{prev_right_lane->points.back().x, prev_right_lane->points.back().y};

      LineSegment2d left_seg{start_left_point, back_left_point};
      LineSegment2d right_seg{start_right_point, back_right_point};
      // LD_TRAFFIC_LOG << fmt::format("left_lane:{} right_lane:{} back_point{:.2f},{:.2f};  {:.2f},{:.2f}", prev_left_lane->id,
      //                               prev_right_lane->id, back_left_point.x(), back_left_point.y(), back_right_point.x(),
      //                               back_right_point.y());

      junction_orin_vertical_         = TransformParams{};
      junction_orin_vertical_->tx     = back_left_point.x();
      junction_orin_vertical_->ty     = back_left_point.y();
      junction_orin_vertical_->width  = back_left_point.DistanceTo(back_right_point);
      junction_orin_vertical_->length = 50.0;
      junction_orin_vertical_->theta  = (left_seg.heading() + right_seg.heading()) / 2.0;
      LD_TRAFFIC_LOG << fmt::format("junction_vertical tx:{:.2f}  ty:{:.2f}  heading:{:.2f}  width:{:.2f}  length:{:.2f}",
                                    junction_orin_vertical_->tx, junction_orin_vertical_->ty, junction_orin_vertical_->theta,
                                    junction_orin_vertical_->width, junction_orin_vertical_->length);
    } else {
      LD_TRAFFIC_LOG << fmt::format("prev_left:{:d}  prev_right:{:d}", prev_left_lane.has_value(), prev_right_lane.has_value());
    }
    return;
  }

  Vec2d junction_start_left{junction_left_lane->points.front().x, junction_left_lane->points.front().y};
  Vec2d junction_start_right{junction_right_lane->points.front().x, junction_right_lane->points.front().y};

  std::optional<double> prev_heading{std::nullopt};
  if (prev_left_lane && prev_right_lane && !prev_left_lane->points.empty() && !prev_right_lane->points.empty()) {
    Vec2d start_left_point{prev_left_lane->points.front().x, prev_left_lane->points.front().y};
    Vec2d start_right_point{prev_right_lane->points.front().x, prev_right_lane->points.front().y};
    Vec2d back_left_point{prev_left_lane->points.back().x, prev_left_lane->points.back().y};
    Vec2d back_right_point{prev_right_lane->points.back().x, prev_right_lane->points.back().y};

    LineSegment2d left_seg{start_left_point, back_left_point};
    LineSegment2d right_seg{start_right_point, back_right_point};
    prev_heading = (left_seg.heading() + right_seg.heading()) / 2.0;

    if (back_left_point.DistanceTo(back_right_point) > junction_start_left.DistanceTo(junction_start_right)) {
      points.emplace_back(back_left_point);
      points.emplace_back(back_right_point);
    } else {
      points.emplace_back(junction_start_left);
      points.emplace_back(junction_start_right);
    }
  } else {
    points.emplace_back(junction_start_left);
    points.emplace_back(junction_start_right);
  }

  std::vector<LaneInfo> left_next_lanes;
  std::vector<LaneInfo> right_next_lanes;
  for (uint64_t id_t : junction_left_lane->next_lane_ids) {
    auto it_lane = std::find_if(lanes.begin(), lanes.end(),
                                [id_t](const LaneInfo &rhs) { return rhs.junction_id != 0 && rhs.id == id_t && !rhs.points.empty(); });
    if (it_lane != lanes.end()) {
      left_next_lanes.push_back(*it_lane);
    }
  }
  if (left_next_lanes.size() == 1) {
    junction_left_lane = left_next_lanes.front();
  } else if (left_next_lanes.size() > 1) {
    TransformParams tra_base;
    tra_base.tx = junction_left_lane->points.back().x;
    tra_base.ty = junction_left_lane->points.back().y;
    LineSegment2d seg{Vec2d(junction_left_lane->points.front().x, junction_left_lane->points.front().y),
                      Vec2d(junction_left_lane->points.back().x, junction_left_lane->points.back().y)};
    tra_base.theta = seg.heading();
    double max_y   = std::numeric_limits<double>::lowest();
    int    max_idx = -1;
    for (std::size_t idx_t = 0; idx_t < left_next_lanes.size(); idx_t++) {
      const auto &lane = left_next_lanes[idx_t];
      Point3DD    end_point{lane.points.back().x, lane.points.back().y, 0.0};
      auto        new_point = TransformCoordinate(end_point, tra_base);
      if (new_point.y > max_y) {
        max_y   = new_point.y;
        max_idx = static_cast<int>(idx_t);
      }
    }
    if (max_idx != -1) {
      junction_left_lane = left_next_lanes[max_idx];
    }
  }
  for (uint64_t id_t : junction_right_lane->next_lane_ids) {
    auto it_lane = std::find_if(lanes.begin(), lanes.end(),
                                [id_t](const LaneInfo &rhs) { return rhs.junction_id != 0 && rhs.id == id_t && !rhs.points.empty(); });
    if (it_lane != lanes.end()) {
      right_next_lanes.push_back(*it_lane);
    }
  }
  if (right_next_lanes.size() == 1) {
    junction_right_lane = right_next_lanes.front();
  } else if (right_next_lanes.size() > 1) {
    TransformParams tra_base;
    tra_base.tx = junction_right_lane->points.back().x;
    tra_base.ty = junction_right_lane->points.back().y;
    LineSegment2d seg{Vec2d(junction_right_lane->points.front().x, junction_right_lane->points.front().y),
                      Vec2d(junction_right_lane->points.back().x, junction_right_lane->points.back().y)};
    tra_base.theta = seg.heading();
    double min_y   = std::numeric_limits<double>::infinity();
    int    min_idx = -1;
    for (std::size_t idx_t = 0; idx_t < right_next_lanes.size(); idx_t++) {
      const auto &lane = right_next_lanes[idx_t];
      Point3DD    end_point{lane.points.back().x, lane.points.back().y, 0.0};
      auto        new_point = TransformCoordinate(end_point, tra_base);
      if (new_point.y < min_y) {
        min_y   = new_point.y;
        min_idx = static_cast<int>(idx_t);
      }
    }
    if (min_idx != -1) {
      junction_right_lane = right_next_lanes[min_idx];
    }
  }
  LD_TRAFFIC_LOG << fmt::format("junction_left_lane:{} junction_right_lane:{}", junction_left_lane->id, junction_right_lane->id);
  Vec2d junction_end_left{junction_left_lane->points.back().x, junction_left_lane->points.back().y};
  Vec2d junction_end_right{junction_right_lane->points.back().x, junction_right_lane->points.back().y};

  if (junction_no_turn || next_lanes.empty()) {
    points.emplace_back(junction_end_right);
    points.emplace_back(junction_end_left);
  } else if (next_left_lane && next_right_lane && !next_left_lane->points.empty() && !next_right_lane->points.empty()) {
    Vec2d left_point{next_left_lane->points.front().x, next_left_lane->points.front().y};
    Vec2d right_point{next_right_lane->points.front().x, next_right_lane->points.front().y};
    if (left_point.DistanceTo(right_point) > junction_start_left.DistanceTo(junction_start_right)) {
      points.emplace_back(right_point);
      points.emplace_back(left_point);
    } else {
      points.emplace_back(junction_end_right);
      points.emplace_back(junction_end_left);
    }
  } else {
    points.emplace_back(junction_end_right);
    points.emplace_back(junction_end_left);
  }
  junction_orin_connect_         = TransformParams{};
  junction_orin_connect_->tx     = points.front().x();
  junction_orin_connect_->ty     = points.front().y();
  junction_orin_connect_->width  = points[0].DistanceTo(points[1]);
  junction_orin_connect_->length = points[0].DistanceTo(points[3]);
  LineSegment2d line_seg{points[0], points[3]};
  junction_orin_connect_->theta = line_seg.heading();

  junction_orin_vertical_         = TransformParams{};
  junction_orin_vertical_->tx     = points.front().x();
  junction_orin_vertical_->ty     = points.front().y();
  junction_orin_vertical_->width  = points[0].DistanceTo(points[1]);
  junction_orin_vertical_->length = points[0].DistanceTo(points[3]);
  if (prev_heading) {
    junction_orin_vertical_->theta = *prev_heading;
  } else {
    junction_orin_vertical_->theta = junction_orin_connect_->theta;
  }
  LD_TRAFFIC_LOG << fmt::format("junction_vertical tx:{:.2f}  ty:{:.2f}  heading:{:.2f}  width:{:.2f}  length:{:.2f}",
                                junction_orin_vertical_->tx, junction_orin_vertical_->ty, junction_orin_vertical_->theta,
                                junction_orin_vertical_->width, junction_orin_vertical_->length);
  LD_TRAFFIC_LOG << fmt::format("junction_connect tx:{:.2f}  ty:{:.2f}  heading:{:.2f}  width:{:.2f}  length:{:.2f}",
                                junction_orin_connect_->tx, junction_orin_connect_->ty, junction_orin_connect_->theta,
                                junction_orin_connect_->width, junction_orin_connect_->length);
  for (const auto &point : points) {
    LD_TRAFFIC_LOG << fmt::format("point_x:{:.2f} point_y:{:.2f}", point.x(), point.y());
  }
}

void TrafficLightMapping::GetPolygonFromSection(const RoutingMapPtr &routing_map, uint64_t section_id,
                                                std::vector<uint64_t> *const prev_lanes_ptr) {
  std::optional<byd::common::math::Polygon2d> section_polygon{std::nullopt};
  if (routing_map == nullptr) {
    return;
  }
  const auto &lanes              = routing_map->lanes;
  const auto &route_sections     = routing_map->route.sections;
  int         target_section_idx = -1;
  for (std::size_t idx = 0; idx < routing_map->route.sections.size(); idx++) {
    if (routing_map->route.sections[idx].id == section_id) {
      target_section_idx = static_cast<int>(idx);
      break;
    }
  }

  if (target_section_idx == -1) {
    LD_TRAFFIC_LOG << "target_section_idx:" << target_section_idx;
    return;
  }
  if (target_section_idx > 0 && (prev_lanes_ptr != nullptr)) {
    *prev_lanes_ptr = routing_map->route.sections[target_section_idx - 1].lane_ids;
  }

  std::vector<uint64_t> junction_lanes = route_sections[target_section_idx].lane_ids;

  auto section_left_lane  = GetEdgeLane(lanes, true, junction_lanes);
  auto section_right_lane = GetEdgeLane(lanes, false, junction_lanes);
  if (!section_left_lane || !section_right_lane) {
    return;
  }
  Vec2d section_start_left{section_left_lane->points.front().x, section_left_lane->points.front().y};
  Vec2d section_start_right{section_right_lane->points.front().x, section_right_lane->points.front().y};
  Vec2d section_end_left{section_left_lane->points.back().x, section_left_lane->points.back().y};
  Vec2d section_end_right{section_right_lane->points.back().x, section_right_lane->points.back().y};
  section_orin_vertical_     = TransformParams{};
  section_orin_vertical_->tx = section_start_left.x();
  section_orin_vertical_->ty = section_start_left.y();
  LineSegment2d seg_2d{section_end_left, section_end_left};
  section_orin_vertical_->theta  = seg_2d.heading();
  section_orin_vertical_->width  = section_start_left.DistanceTo(section_start_right);
  section_orin_vertical_->length = section_start_left.DistanceTo(section_end_left);
  LD_TRAFFIC_LOG << fmt::format("section_vertical tx:{:.2f}  ty:{:.2f}  heading:{:.2f}  width:{:.2f}  length:{:.2f}",
                                section_orin_vertical_->tx, section_orin_vertical_->ty, section_orin_vertical_->theta,
                                section_orin_vertical_->width, section_orin_vertical_->length);
}

bool TrafficLightMapping::IsUturnSection(const RoutingMapPtr &routing_map, const std::vector<uint64_t> &section_lanes_id,
                                         const std::set<uint64_t> &light_ids) {
  const auto &lanes         = routing_map->lanes;
  bool        is_only_uturn = true;
  for (const auto id : light_ids) {
    auto it_light = std::find_if(perception_traffic_lights_.begin(), perception_traffic_lights_.end(),
                                 [id](const TrfObjectInfo &rhs) { return id == rhs.id; });
    if (it_light != perception_traffic_lights_.end() &&
        it_light->attributes.traffic_light_shape != TrafficLightShapeType::TLS_TURN_ARROUND_ARROW) {
      is_only_uturn = false;
      break;
    }
  }
  if (!is_only_uturn) {
    return true;
  }
  for (uint64_t lane_id : section_lanes_id) {
    auto it_lane = std::find_if(lanes.begin(), lanes.end(), [lane_id](const LaneInfo &rhs) { return lane_id == rhs.id; });
    if (it_lane != lanes.end() && it_lane->turn_type == TurnType::U_TURN) {
      return true;
    }
  }
  return false;
}

void TrafficLightMapping::FindBestLight(const RoutingMapPtr &routing_map) {
  traffic_lights_ld_ = TrafficLights{};
  if (junction_light_ids_.empty() && section_light_ids_.empty()) {
    LD_TRAFFIC_LOG << "junction_light_ids_ and section_light_ids_ is empty.";
    return;
  }
  auto it_ego_section =
      std::find_if(routing_map->route.sections.begin(), routing_map->route.sections.end(),
                   [&routing_map](const auto &section) { return routing_map->route.navi_start.section_id == section.id; });
  auto &lanes = routing_map->lanes;
  if (it_ego_section == routing_map->route.sections.end()) {
    LD_TRAFFIC_LOG << "no_find_ego_section.";
    return;
  }

  uint64_t target_junction_id = 0;
  uint64_t target_section_id  = 0;

  std::set<uint64_t> light_ids;
  for (; it_ego_section != routing_map->route.sections.end(); it_ego_section++) {
    for (auto lane_id : it_ego_section->lane_ids) {
      auto it_lane = std::find_if(lanes.begin(), lanes.end(), [&lane_id](const auto &lane_t) { return lane_t.id == lane_id; });
      if (it_lane == lanes.end() || it_lane->type == LaneType::LANE_NON_MOTOR) {
        continue;
      }
      if (it_lane->junction_id != 0 && junction_light_ids_.count(it_lane->junction_id) > 0) {
        target_junction_id = it_lane->junction_id;
        light_ids          = junction_light_ids_[it_lane->junction_id];
        break;
      }
      if (it_lane->section_id != 0 && section_light_ids_.count(it_lane->section_id) > 0 &&
          IsUturnSection(routing_map, it_ego_section->lane_ids, section_light_ids_[it_lane->section_id])) {
        target_section_id = it_lane->section_id;
        light_ids         = section_light_ids_[target_section_id];
        break;
      }
      if (target_section_id != 0 || target_junction_id != 0) {
        break;
      }
    }
    if (target_section_id != 0 || target_junction_id != 0) {
      break;
    }
  }
  LD_TRAFFIC_LOG << fmt::format("target_junction_id:{}  target_section_id:{}  light_ids:{}", target_junction_id, target_section_id,
                                light_ids);
  if ((target_section_id == 0 && target_junction_id == 0) || light_ids.empty()) {
    return;
  }
  std::vector<TrfObjectInfo> traffic_lights_vec;
  for (uint64_t obj_id : light_ids) {
    for (const auto &obj : perception_traffic_lights_) {
      if (obj.id == obj_id) {
        traffic_lights_vec.push_back(obj);
        break;
      }
    }
  }
  std::optional<byd::common::math::Polygon2d> junction_polygon{std::nullopt};

  std::vector<uint64_t> prev_lane_ids;
  constexpr double      width_offset_val = 5.0;

  if (target_junction_id != 0) {
    GetPolygonFromJunction(routing_map, target_junction_id, &prev_lane_ids);
    if (junction_orin_vertical_ && junction_orin_connect_) {
      double width_offset = 10.0;
      LD_TRAFFIC_LOG << "filter_objs_by_vertical_&&_connnect.";
      auto res_vertical = FilterTrfObjByPolygon(junction_orin_vertical_.value(), traffic_lights_vec, width_offset, width_offset, 80, -30);
      auto res_connect  = FilterTrfObjByPolygon(junction_orin_connect_.value(), traffic_lights_vec, width_offset, width_offset, 80, -30);

      std::vector<uint64_t>      obj_ids_filtered;
      std::vector<uint64_t>      obj_ids_all;
      std::vector<uint64_t>      obj_ids_invalid;
      std::vector<TrfObjectInfo> filtered_objs;

      for (const auto &obj_t : traffic_lights_vec) {
        obj_ids_all.emplace_back(obj_t.id);
        auto it_ver = std::find_if(res_vertical.begin(), res_vertical.end(), [&obj_t](const auto &rhs) { return rhs.id == obj_t.id; });
        auto it_con = std::find_if(res_connect.begin(), res_connect.end(), [&obj_t](const auto &rhs) { return rhs.id == obj_t.id; });
        if (it_ver != res_vertical.end() || it_con != res_connect.end()) {
          filtered_objs.emplace_back(obj_t);
          obj_ids_filtered.emplace_back(obj_t.id);
        } else {
          obj_ids_invalid.emplace_back(obj_t.id);
        }
      }
      LD_TRAFFIC_LOG << fmt::format("vertical_connect_obj_all:{}  filtered:{}  invalid:{}", obj_ids_all, obj_ids_filtered, obj_ids_invalid);
      traffic_lights_vec = filtered_objs;
    }
    if (junction_orin_vertical_) {
      // filter traffic_light_vec.
      traffic_lights_vec = FilterTrafObjsUturnPoly(routing_map, *junction_orin_vertical_, traffic_lights_vec, prev_lane_ids);

      LD_TRAFFIC_LOG << "junction_vertical.";
      auto res = FilterTrfObjByPolygon(*junction_orin_vertical_, traffic_lights_vec, width_offset_val, width_offset_val, 30);
      GetTrafficArrow(res, uturn_shapes_, traffic_lights_prev_ld_.u_turn, traffic_lights_ld_.u_turn);
      GetTrafficArrow(res, left_shapes_, traffic_lights_prev_ld_.left, traffic_lights_ld_.left);
      GetTrafficArrow(res, straight_shapes_, traffic_lights_prev_ld_.straight, traffic_lights_ld_.straight);
      GetTrafficArrow(res, right_shapes_, traffic_lights_prev_ld_.right, traffic_lights_ld_.right);
      GetTrafficArrow(traffic_lights_vec, {message::sensor::TrafficLightShapeType::TLS_LEFT_ARROW}, traffic_lights_prev_ld_.u_turn,
                      traffic_lights_ld_.u_turn);
      GetTrafficArrow(res, {message::sensor::TrafficLightShapeType::TLS_CIRCULAR}, traffic_lights_prev_ld_.u_turn,
                      traffic_lights_ld_.u_turn);
      GetTrafficArrow(res, {message::sensor::TrafficLightShapeType::TLS_CIRCULAR}, traffic_lights_prev_ld_.left, traffic_lights_ld_.left);
      GetTrafficArrow(res, {message::sensor::TrafficLightShapeType::TLS_CIRCULAR}, traffic_lights_prev_ld_.straight,
                      traffic_lights_ld_.straight);
    }
    if (junction_orin_connect_) {
      LD_TRAFFIC_LOG << "junction_connect.";
      auto res = FilterTrfObjByPolygon(*junction_orin_connect_, traffic_lights_vec, width_offset_val, width_offset_val, 30);
      GetTrafficArrow(res, uturn_shapes_, traffic_lights_prev_ld_.u_turn, traffic_lights_ld_.u_turn);
      GetTrafficArrow(res, left_shapes_, traffic_lights_prev_ld_.left, traffic_lights_ld_.left);
      GetTrafficArrow(res, straight_shapes_, traffic_lights_prev_ld_.straight, traffic_lights_ld_.straight);
      GetTrafficArrow(res, right_shapes_, traffic_lights_prev_ld_.right, traffic_lights_ld_.right);
      GetTrafficArrow(traffic_lights_vec, {message::sensor::TrafficLightShapeType::TLS_LEFT_ARROW}, traffic_lights_prev_ld_.u_turn,
                      traffic_lights_ld_.u_turn);
      GetTrafficArrow(res, {message::sensor::TrafficLightShapeType::TLS_CIRCULAR}, traffic_lights_prev_ld_.u_turn,
                      traffic_lights_ld_.u_turn);
      GetTrafficArrow(res, {message::sensor::TrafficLightShapeType::TLS_CIRCULAR}, traffic_lights_prev_ld_.left, traffic_lights_ld_.left);
      GetTrafficArrow(res, {message::sensor::TrafficLightShapeType::TLS_CIRCULAR}, traffic_lights_prev_ld_.straight,
                      traffic_lights_ld_.straight);
    }
  } else if (target_section_id != 0) {
    GetPolygonFromSection(routing_map, target_section_id, &prev_lane_ids);
    if (section_orin_vertical_) {
      // filter traffic_light_vec.
      // traffic_lights_vec = FilterTrafObjsUturnPoly(routing_map, *junction_orin_vertical_, traffic_lights_vec, prev_lane_ids);

      LD_TRAFFIC_LOG << "section_vertical.";
      auto res = FilterTrfObjByPolygon(*section_orin_vertical_, traffic_lights_vec, width_offset_val, width_offset_val,
                                       60 - section_orin_vertical_->length);
      GetTrafficArrow(res, uturn_shapes_, traffic_lights_prev_ld_.u_turn, traffic_lights_ld_.u_turn);
      GetTrafficArrow(res, left_shapes_, traffic_lights_prev_ld_.left, traffic_lights_ld_.left);
      GetTrafficArrow(res, straight_shapes_, traffic_lights_prev_ld_.straight, traffic_lights_ld_.straight);
      GetTrafficArrow(res, right_shapes_, traffic_lights_prev_ld_.right, traffic_lights_ld_.right);
      GetTrafficArrow(traffic_lights_vec, {message::sensor::TrafficLightShapeType::TLS_LEFT_ARROW}, traffic_lights_prev_ld_.u_turn,
                      traffic_lights_ld_.u_turn);
      GetTrafficArrow(res, {message::sensor::TrafficLightShapeType::TLS_CIRCULAR}, traffic_lights_prev_ld_.u_turn,
                      traffic_lights_ld_.u_turn);
      GetTrafficArrow(res, {message::sensor::TrafficLightShapeType::TLS_CIRCULAR}, traffic_lights_prev_ld_.left, traffic_lights_ld_.left);
      GetTrafficArrow(res, {message::sensor::TrafficLightShapeType::TLS_CIRCULAR}, traffic_lights_prev_ld_.straight,
                      traffic_lights_ld_.straight);
    }
  }

  GetTrafficLightsLd(junction_polygon, traffic_lights_vec);

  ProcessArrowCircleRightLight();

  uint32_t cyber_counter = perception_traffic_light_info_ ? perception_traffic_light_info_->header.cycle_counter : 0;
  LD_TRAFFIC_LOG << fmt::format("vis_counter:{} ld_traffic_straight {}", cyber_counter, traffic_lights_ld_.straight);
  LD_TRAFFIC_LOG << fmt::format("vis_counter:{} ld_traffic_left     {}", cyber_counter, traffic_lights_ld_.left);
  LD_TRAFFIC_LOG << fmt::format("vis_counter:{} ld_traffic_uturn    {}", cyber_counter, traffic_lights_ld_.u_turn);
  LD_TRAFFIC_LOG << fmt::format("vis_counter:{} ld_traffic_right    {}", cyber_counter, traffic_lights_ld_.right);
}

void TrafficLightMapping::ProcessArrowCircleRightLight() {
  if (traffic_lights_ld_.right.is_valid || traffic_lights_ld_.right.traffic_obj_info) {
    return;
  }
  if (!traffic_lights_prev_ld_.right.is_valid || !traffic_lights_prev_ld_.right.traffic_obj_info) {
    return;
  }
  const auto &prev_obj = traffic_lights_prev_ld_.right.traffic_obj_info;
  LD_TRAFFIC_LOG << fmt::format("prev_right_obj_id:{} box_id:{}", prev_obj->id, prev_obj->traffic_light_box_id);
  for (const auto &obj : perception_traffic_lights_) {
    if (obj.traffic_light_box_id == prev_obj->traffic_light_box_id &&
        (obj.attributes.traffic_light_shape == TrafficLightShapeType::TLS_RIGHT_ARROW ||
         obj.attributes.traffic_light_shape == TrafficLightShapeType::TLS_UP_RIGHT_ARROW ||
         obj.attributes.traffic_light_shape == TrafficLightShapeType::TLS_CIRCULAR)) {
      return SetLightT(obj, traffic_lights_ld_.right);
    }
  }
}

std::vector<TrfObjectInfo> TrafficLightMapping::FilterTrafObjsUturnPoly(const RoutingMapPtr              &routing_map,
                                                                        const TransformParams            &transform_para,
                                                                        const std::vector<TrfObjectInfo> &traffic_lights_vec,
                                                                        const std::vector<uint64_t>      &prev_lane_ids) {

  if (routing_map == nullptr) {
    return traffic_lights_vec;
  }
  const auto &lanes = routing_map->lanes;

  double uturn_width = std::numeric_limits<double>::lowest();
  for (uint64_t lane_id : prev_lane_ids) {
    auto it_lane = std::find_if(lanes.begin(), lanes.end(), [lane_id](const LaneInfo &rhs) { return lane_id == rhs.id; });
    if (it_lane == lanes.end() || it_lane->next_lane_ids.empty()) {
      continue;
    }
    for (uint64_t next_id : it_lane->next_lane_ids) {
      auto it_next_lane = std::find_if(lanes.begin(), lanes.end(),
                                       [next_id](const LaneInfo &rhs) { return next_id == rhs.id && rhs.turn_type == TurnType::U_TURN; });
      if (it_next_lane == lanes.end() || it_next_lane->points.empty()) {
        continue;
      }
      Point3DD back_point{it_next_lane->points.back().x, it_next_lane->points.back().y, 0.0};
      Point3DD new_pos = TransformCoordinate(back_point, transform_para);
      LD_TRAFFIC_LOG << fmt::format("uturn_lane_id:{}  pos_old:{:.2f},{:.2f}  pos_new:{:.2f},{:.2f}", it_next_lane->id, back_point.x,
                                    back_point.y, new_pos.x, new_pos.y);
      if (new_pos.y > 3.0 && uturn_width < new_pos.y) {
        uturn_width = new_pos.y;
      }
    }
  }
  double width_max = 12.0;
  if (uturn_width > width_max) {
    width_max = uturn_width;
  } else {
    width_max = std::max(width_max, uturn_width + 3.5);
  }
  width_max = std::min(width_max, 25.0);
  auto res  = FilterTrfObjByPolygon(transform_para, traffic_lights_vec, width_max, 35, 80, -30);

  std::vector<uint64_t> filtered_obj;
  for (const auto &obj_raw : traffic_lights_vec) {
    auto it_obj = std::find_if(res.begin(), res.end(), [&obj_raw](const TrfObjectInfo &rhs) { return obj_raw.id == rhs.id; });
    if (it_obj == res.end()) {
      filtered_obj.emplace_back(obj_raw.id);
    }
  }

  LD_TRAFFIC_LOG << fmt::format("fitlered_obj_id:{} width_offset:{:.2f}", filtered_obj, uturn_width < -100 ? -100 : uturn_width);

  return res;
}

std::vector<TrfObjectInfo> TrafficLightMapping::FilterTrfObjByPolygon(const TransformParams            &transform_para,
                                                                      const std::vector<TrfObjectInfo> &traffic_lights_vec,
                                                                      double width_offset_left, double width_offset_right,
                                                                      double upper_offset, double lower_offset) {
  std::vector<TrfObjectInfo> res;
  for (const auto &obj : traffic_lights_vec) {
    Point3DD obj_pos{obj.position.x, obj.position.y, 0.0};
    auto     obj_new_pos = TransformCoordinate(obj_pos, transform_para);
    bool     is_fullfill{false};
    if (obj_new_pos.y < width_offset_left && obj_new_pos.y > -transform_para.width - width_offset_right && obj_new_pos.x > lower_offset &&
        obj_new_pos.x < transform_para.length + upper_offset) {
      res.emplace_back(obj);
      is_fullfill = true;
    }
    LD_TRAFFIC_LOG << fmt::format(
        "obj_id:{}  raw_pos x:{:.2f} y:{:.2f}  new_pos x:{:.2f} y:{:.2f} fullfill:{:d}  para:len:{:.2f} width:{:.2f}  offset_left:{:.2f}  "
        "right:{:.2f} low:{:.2f} upper:{:.2f}",
        obj.id, obj_pos.x, obj_pos.y, obj_new_pos.x, obj_new_pos.y, is_fullfill, transform_para.length, transform_para.width,
        width_offset_left, width_offset_right, lower_offset, upper_offset);
  }
  std::vector<TrfObjectInfo> bak;
  for (const auto &obj_0 : res) {
    Vec2d pos_0{obj_0.position.x, obj_0.position.y};
    for (const auto &obj_1 : traffic_lights_vec) {
      auto it_res = std::find_if(res.begin(), res.end(), [&obj_1](const TrfObjectInfo &rhs) { return rhs.id == obj_1.id; });
      if (it_res != res.end()) {
        continue;
      }
      auto it_bak = std::find_if(bak.begin(), bak.end(), [&obj_1](const TrfObjectInfo &rhs) { return rhs.id == obj_1.id; });
      if (it_bak != bak.end()) {
        continue;
      }
      Vec2d  pos_1{obj_1.position.x, obj_1.position.y};
      double dis = pos_0.DistanceTo(pos_1);
      if (dis < 8) {
        LD_TRAFFIC_LOG << fmt::format("obj_0:{} obj_1:{} dis:{:.2f}", obj_0.id, obj_1.id, dis);
        bak.push_back(obj_1);
      }
    }
  }
  res.insert(res.end(), bak.begin(), bak.end());
  return res;
}

std::optional<std::tuple<uint64_t, double, bool>> TrafficLightMapping::MatchTrafficLightAndJunctionStopline(
    const RoutingMapPtr &routing_map) {
  if (ldmap_junction_ids_.empty() && maybe_junctions_.empty()) {
    return std::nullopt;
  }
  std::set<uint64_t> obj_ids_set;

  std::vector<std::shared_ptr<cem::message::sensor::TrfObjectInfo>> light_objs;

  auto CollectLights = [&](const TrafficLight &rhs) {
    if (!rhs.traffic_obj_info) {
      return;
    }
    if (obj_ids_set.count(rhs.traffic_obj_info->id) > 0) {
      return;
    }
    TRAFFIC_LOG << "traffic_light_pool_id:" << rhs.traffic_obj_info->id;
    light_objs.push_back(rhs.traffic_obj_info);
  };
  CollectLights(traffic_lights_.u_turn);
  CollectLights(traffic_lights_.left);
  CollectLights(traffic_lights_.straight);
  CollectLights(traffic_lights_.right);
  if (light_objs.empty()) {
    return std::nullopt;
  }
  double dis_polygon_traffic_pos = 0.0;

  double   min_junction_dis = std::numeric_limits<double>::infinity();
  uint64_t min_junction_id  = 0;

  std::map<uint64_t, double> junc_dis;
  for (const auto &jun_polygon : ldmap_junction_ids_) {
    if (jun_polygon.second.points().size() < 3) {
      continue;
    }
    dis_polygon_traffic_pos = std::numeric_limits<double>::infinity();
    for (const auto &obj : light_objs) {
      Vec2d  light_pos{obj->position.x, obj->position.y};
      double dis_tmp          = jun_polygon.second.DistanceTo(light_pos);
      dis_polygon_traffic_pos = (dis_polygon_traffic_pos > dis_tmp) ? dis_tmp : dis_polygon_traffic_pos;
    }

    if (dis_polygon_traffic_pos < lane_virtual_dis_threshold) {
      junc_dis.insert({jun_polygon.first, dis_polygon_traffic_pos});
      if (min_junction_dis > dis_polygon_traffic_pos) {
        min_junction_dis = dis_polygon_traffic_pos;
        min_junction_id  = jun_polygon.first;
      }
    }

    TRAFFIC_LOG << fmt::format("jun_id:{}  dis:{:.2f}  min_junction_id:{}  dis:{:.2f}", jun_polygon.first, dis_polygon_traffic_pos,
                               min_junction_id, min_junction_dis);
  }

  double   min_section_dis = std::numeric_limits<double>::infinity();
  uint64_t min_section_id  = 0;

  const auto &lanes    = routing_map->lanes;
  const auto &sections = routing_map->route.sections;

  for (const auto &sec_polygon : maybe_junctions_) {
    if (sec_polygon.second.points().size() < 3) {
      continue;
    }
    auto it_sec =
        std::find_if(sections.begin(), sections.end(), [&sec_polygon](const SectionInfo &rhs) { return rhs.id == sec_polygon.first; });
    if (it_sec == sections.end()) {
      TRAFFIC_LOG << "no_find_section:" << sec_polygon.first;
      continue;
    }
    bool is_valid = false;
    for (uint64_t lane_id : it_sec->lane_ids) {
      auto it_lane = std::find_if(lanes.begin(), lanes.end(), [&lane_id](const auto &lane_t) { return lane_t.id == lane_id; });
      if (it_lane == lanes.end() || it_lane->type == LaneType::LANE_NON_MOTOR) {
        continue;
      }
      is_valid = LaneStoplineTrafficLight(sec_polygon.first, *it_lane, -50.0);
      if (is_valid) {
        break;
      }
    }

    dis_polygon_traffic_pos = std::numeric_limits<double>::infinity();
    for (const auto &obj : light_objs) {
      Vec2d  light_pos{obj->position.x, obj->position.y};
      double dis_tmp          = sec_polygon.second.DistanceTo(light_pos);
      dis_polygon_traffic_pos = (dis_polygon_traffic_pos > dis_tmp) ? dis_tmp : dis_polygon_traffic_pos;
    }

    if (dis_polygon_traffic_pos < lane_virtual_dis_threshold && is_valid) {
      junc_dis.insert({sec_polygon.first, dis_polygon_traffic_pos});
      if (min_section_dis > dis_polygon_traffic_pos) {
        min_section_dis = dis_polygon_traffic_pos;
        min_section_id  = sec_polygon.first;
      }
    }

    TRAFFIC_LOG << fmt::format("sec_id:{}  dis:{:.2f}  min_section_id:{}  dis:{:.2f} is_valid:{:d}", sec_polygon.first,
                               dis_polygon_traffic_pos, min_junction_id, min_section_dis, is_valid);
  }
  if (min_junction_id != 0 && min_section_id == 0) {
    return std::make_tuple(min_junction_id, min_junction_dis, true);
  }
  if (min_junction_id == 0 && min_section_id != 0) {
    return std::make_tuple(min_section_id, min_section_dis, false);
  }
  if (min_junction_id != 0 && min_section_id != 0) {
    if (min_junction_dis < min_section_dis) {
      return std::make_tuple(min_junction_id, min_junction_dis, true);
    }
    return std::make_tuple(min_section_id, min_section_dis, false);
  }
  return std::nullopt;
}

TrafficLights TrafficLightMapping::ChooseBestTrafficLights(const TrafficLights &cloud_lights, const TrafficLights &default_lights) {
  TrafficLights traffic_lights{default_lights};
  switch (cloud_lights.junction_cloud_type) {
    case 1:
      return cloud_lights;
    case 2:
      traffic_lights.distance_to_stopline = cloud_lights.distance_to_stopline;
      return traffic_lights;
    case 3:
      return traffic_lights;
    default:
      return cloud_lights;
  }
  return traffic_lights;
}

bool TrafficLightMapping::IsLightBlockFailed(const TrafficLight &light_prev, TrafficLight &light) const {
  light.invalid_counter = light_prev.invalid_counter;
  bool is_prev_valid    = light_prev.color == message::sensor::TLC_RED || light_prev.color == message::sensor::TLC_GREEN ||
                       light_prev.color == message::sensor::TLC_YELLOW || light_prev.color == message::sensor::TLC_YELLOW_FLASHING ||
                       light_prev.color == message::sensor::TLC_GREEN_FLASHING || light_prev.color == message::sensor::TLC_RED_FLASHING ||
                       light_prev.color == message::sensor::TLC_BLURRING_MODE;
  if (light.turn_type == TurnType::RIGHT_TURN && light_prev.color == message::sensor::TLC_GREEN &&
      light_prev.traffic_reason == byd::msg::orin::routing_map::LaneInfo::SET_DEFAULT_OBJ) {
    is_prev_valid = false;
  }
  bool is_current_invalid =
      light.stay_prev_counter > 0 ||
      (light.color != message::sensor::TLC_RED && light.color != message::sensor::TLC_GREEN && light.color != message::sensor::TLC_YELLOW &&
       light.color != message::sensor::TLC_YELLOW_FLASHING && light.color != message::sensor::TLC_GREEN_FLASHING &&
       light.color != message::sensor::TLC_RED_FLASHING && light.color != message::sensor::TLC_BLURRING_MODE);
  if (light.turn_type == TurnType::RIGHT_TURN && light.color == message::sensor::TLC_GREEN &&
      light.traffic_reason == byd::msg::orin::routing_map::LaneInfo::SET_DEFAULT_OBJ) {
    is_current_invalid = true;
  }
  bool is_light_dis_pan = false;
  if (is_current_invalid && is_prev_valid && light_prev.traffic_obj_info) {
    auto it_f = std::find_if(perception_traffic_lights_.begin(), perception_traffic_lights_.end(),
                             [&light_prev](const TrfObjectInfo &rhs) { return rhs.id == light_prev.traffic_obj_info->id; });
    if (it_f != perception_traffic_lights_.end()) {
      is_light_dis_pan = true;
      TRAFFIC_REC_LOG << fmt::format("turn_type:{}  is_light_dis_pan.", magic_enum::enum_name(light.turn_type));
    }
  }
  if (is_current_invalid && !is_light_dis_pan && distance_to_ego_light_junction_prev_ > 0 && distance_to_ego_light_junction_prev_ < 50.0) {
    if (is_prev_valid || light.invalid_counter > 0) {
      light.invalid_counter++;
    }
    if (light.invalid_counter >= 3) {
      light.color             = message::sensor::TLC_BLOCK_FAILED;
      light.is_valid          = true;
      light.stay_prev_counter = 0;
      light.traffic_light_num = 1000;
      light.traffic_obj_info.reset();
      TRAFFIC_REC_LOG << fmt::format("BLOCK_FAILED_check sucessful turn:{}  oringin_counter:{}  invalid_counter:{}  dis:{:.2f}",
                                     magic_enum::enum_name(light.turn_type), light_prev.invalid_counter, light.invalid_counter,
                                     distance_to_ego_light_junction_prev_);
      return true;
    }
    TRAFFIC_REC_LOG << fmt::format("BLOCK_FAILED_check failed counter_less_3. turn:{} oringin_counter:{}   invalid_counter:{}  dis:{:.2f}",
                                   magic_enum::enum_name(light.turn_type), light_prev.invalid_counter, light.invalid_counter,
                                   distance_to_ego_light_junction_prev_);
    return false;
  }
  TRAFFIC_REC_LOG << fmt::format("BLOCK_FAILED_check failed not in 50m->ego_junction turn:{}    dis:{:.2f}  current_invalid:{:d}",
                                 magic_enum::enum_name(light.turn_type), distance_to_ego_light_junction_prev_, !is_current_invalid);
  light.invalid_counter = 0;
  return false;
}

byd::msg::orin::routing_map::LightStatus GetColorStatus(cem::message::sensor::TrafficLightColorType color) {
  switch (color) {
    case cem::message::sensor::TrafficLightColorType::TLC_GREEN: {
      return byd::msg::orin::routing_map::LightStatus::GREEN_LIGHT;
    }
    case cem::message::sensor::TrafficLightColorType::TLC_GREEN_FLASHING: {
      return byd::msg::orin::routing_map::LightStatus::GREEN_BLINKING;
    }
    case cem::message::sensor::TrafficLightColorType::TLC_RED:
    case cem::message::sensor::TrafficLightColorType::TLC_RED_FLASHING: {
      return byd::msg::orin::routing_map::LightStatus::RED_LIGHT;
    }
    case cem::message::sensor::TrafficLightColorType::TLC_YELLOW: {
      return byd::msg::orin::routing_map::LightStatus::YELLOW_LIGHT;
    }
    case cem::message::sensor::TrafficLightColorType::TLC_YELLOW_FLASHING: {
      return byd::msg::orin::routing_map::LightStatus::YELLOW_BLINKING;
    }
    case cem::message::sensor::TrafficLightColorType::TLC_BLURRING_MODE: {
      return byd::msg::orin::routing_map::LightStatus::BLURRING_MODE;
    }
    case cem::message::sensor::TrafficLightColorType::TLC_BLOCK_FAILED: {
      return byd::msg::orin::routing_map::LightStatus::BLOCK_FAIL;
    }
    case cem::message::sensor::TrafficLightColorType::TLC_NONE_LIGHT: {
      return byd::msg::orin::routing_map::LightStatus::NONE_LIGHT;
    }
    case cem::message::sensor::TrafficLightColorType::TLC_NOT_MATCH:
    case cem::message::sensor::TrafficLightColorType::TLC_CLOUD_NOMATCH: {
      return byd::msg::orin::routing_map::LightStatus::CLOUD_LIGHT_NOMATCH;
    }
    default: {
      return byd::msg::orin::routing_map::LightStatus::UNKNOWN_LIGHT;
    }
  }
};

void TrafficLightMapping::LightStatusConverter(const TrafficLight &light, byd::msg::orin::routing_map::TrafficLightStatus &rhs) {
  if (!light.is_valid) {
    return;
  }
  rhs.set_light_status(GetColorStatus(light.color));
  rhs.set_traffic_light_num(light.traffic_light_num);
  rhs.set_perception_sequence(light.perception_seq_num);
  if (light.traffic_obj_info) {
    rhs.set_traffic_obj_id(light.traffic_obj_info->id);
  }
  rhs.set_stay_prev_counter(light.stay_prev_counter);
  rhs.set_invalid_counter(light.invalid_counter);
  rhs.set_traffic_reason(light.traffic_reason);
}

void TrafficLightMapping::SetAllLightStatus() {
  LightStatusConverter(traffic_lights_.left, light_left_);
  LightStatusConverter(traffic_lights_.right, light_right_);
  LightStatusConverter(traffic_lights_.u_turn, light_uturn_);
  LightStatusConverter(traffic_lights_.straight, light_straight_);
  light_left_.set_turn_type(::byd::msg::orin::routing_map::LaneInfo_TurnType::LaneInfo_TurnType_LEFT_TURN);
  light_right_.set_turn_type(::byd::msg::orin::routing_map::LaneInfo_TurnType::LaneInfo_TurnType_RIGHT_TURN);
  light_uturn_.set_turn_type(::byd::msg::orin::routing_map::LaneInfo_TurnType::LaneInfo_TurnType_U_TURN);
  light_straight_.set_turn_type(::byd::msg::orin::routing_map::LaneInfo_TurnType::LaneInfo_TurnType_NO_TURN);
}

void TrafficLightMapping::IsRightExit() {
  if (distance_to_junction_prev_ < k_epsilon) {
    is_right_exist = false;
  } else if (traffic_lights_.right.is_valid && traffic_lights_.right.traffic_obj_info) {
    is_right_exist = true;
  }
  if (!is_right_exist) {
    if (!traffic_lights_.right.is_valid) {
      traffic_lights_.right.is_valid       = true;
      traffic_lights_.right.color          = message::sensor::TLC_GREEN;
      traffic_lights_.right.traffic_reason = byd::msg::orin::routing_map::LaneInfo::SET_DEFAULT_OBJ;
      NEW_LOG << "";
    }
  } else if (!traffic_lights_.right.is_valid || (traffic_lights_.right.color == message::sensor::TLC_UNKNOWN)) {
    traffic_lights_.right.is_valid = false;
    traffic_lights_.right.color    = message::sensor::TLC_UNKNOWN;
  }
  if (traffic_lights_.right.is_valid && traffic_lights_.right.color == message::sensor::TLC_GREEN &&
      traffic_lights_.right.traffic_reason == byd::msg::orin::routing_map::LaneInfo::SET_DEFAULT_OBJ) {
    traffic_lights_.right.is_valid       = false;
    traffic_lights_.right.color          = message::sensor::TLC_NONE_LIGHT;
    traffic_lights_.right.traffic_reason = byd::msg::orin::routing_map::LaneInfo::UNKNOWN_STATE;
  }
  NEW_LOG << fmt::format("is_right_exist:{:d}", is_right_exist);
}

void TrafficLightMapping::SetTrafficLightInfo() {
  auto AppendLightInfo = [&](const TrafficLight &light, const char *prefix) {
    if (light.is_valid) {
      info_traffic_lights_ += fmt::format(" {}:{}-{}-{};    ", prefix, light.traffic_obj_info ? light.traffic_obj_info->id : 0, light.color,
                                          light.traffic_reason);
    }
  };
  info_traffic_lights_ += " TRF";

  AppendLightInfo(traffic_lights_.u_turn, "U");
  AppendLightInfo(traffic_lights_.left, "L");
  AppendLightInfo(traffic_lights_.right, "R");
  AppendLightInfo(traffic_lights_.straight, "S");
}

void TrafficLightMapping::IsEgoDedicatedRight() {
  if (sd_navigation_city_ && sd_navigation_city_->IsEgoInDedicatedRightLane()) {
    TRAFFIC_REC_LOG << fmt::format("ego_in_right_dedicated raw_traffic_right    {}", traffic_lights_.right);
    traffic_lights_.right.color                  = message::sensor::TLC_GREEN;
    traffic_lights_.right.traffic_light_flashing = false;
    traffic_lights_.right.is_valid               = true;
  }
}

void TrafficLightMapping::BindingTrafficLightToBev(const BevMapInfoPtr &bev_map) {
  if (bev_map == nullptr || (sd_navigation_city_ && sd_navigation_city_->IsEgoInDedicatedRightLane())) {
    return;
  }
  auto ConvertLightState = [](cem::message::sensor::TrafficLightColorType color_type, bool is_flash) -> BevTrafficLightState {
    switch (color_type) {
      case cem::message::sensor::TrafficLightColorType::TLC_GREEN: {
        return is_flash ? BevTrafficLightState::TL_COLOR_GREEN : BevTrafficLightState::TL_COLOR_GREEN_FLASH;
      }
      case cem::message::sensor::TrafficLightColorType::TLC_RED: {
        // return is_flash ? BevTrafficLightState::TL_COLOR_RED : BevTrafficLightState::TL_COLOR_RED_FLASH;
        return BevTrafficLightState::TL_COLOR_RED;
      }
      case cem::message::sensor::TrafficLightColorType::TLC_YELLOW: {
        return is_flash ? BevTrafficLightState::TL_COLOR_YELLOW_FLASH : BevTrafficLightState::TL_COLOR_YELLOW;
      }
      case cem::message::sensor::TrafficLightColorType::TLC_GREEN_FLASHING: {
        return BevTrafficLightState::TL_COLOR_GREEN_FLASH;
      }
      case cem::message::sensor::TrafficLightColorType::TLC_YELLOW_FLASHING: {
        return BevTrafficLightState::TL_COLOR_YELLOW_FLASH;
      }
      case cem::message::sensor::TrafficLightColorType::TLC_BLURRING_MODE: {
        return BevTrafficLightState::TL_COLOR_BLURRING_MODE;
      }
      case cem::message::sensor::TrafficLightColorType::TLC_BLOCK_FAILED: {
        return BevTrafficLightState::TL_COLOR_BLOCK_FAILED;
      }
      default: {
        return BevTrafficLightState::TL_COLOR_UNKNOWN;
      }
    }
    return BevTrafficLightState::TL_COLOR_UNKNOWN;
  };

  std::map<uint64_t, byd::common::math::Polygon2d>     junction_id_polygon;
  std::map<uint64_t, byd::common::math::LineSegment2d> stopline_id_line2seg;
  std::map<uint64_t, byd::common::math::Polygon2d>     crosswalk_id_polygon;

  auto ProcessPolygon = [](const auto &items, auto &container, size_t min_points) {
    std::vector<Vec2d>           points;
    byd::common::math::Polygon2d polygon;
    for (const auto &item : items) {
      points.clear();
      points.reserve(item.line_points_ego.size());
      for (const auto &poi : item.line_points_ego) {
        points.emplace_back(poi.x, poi.y);
      }
      if (points.size() >= min_points) {
        polygon.Init(points);
        container.emplace(item.id, polygon);
      }
    }
  };

  auto ProcessStopline = [](const auto &items, auto &container) {
    std::vector<Vec2d> points;
    for (const auto &item : items) {
      points.clear();
      points.reserve(item.line_points_ego.size());
      for (const auto &poi : item.line_points_ego) {
        points.emplace_back(poi.x, poi.y);
      }
      if (points.size() >= 2) {
        container.emplace(item.id, LineSegment2d{points[0], points[1]});
      }
    }
  };

  ProcessPolygon(bev_map->junctions, junction_id_polygon, 3);
  ProcessStopline(bev_map->stop_lines, stopline_id_line2seg);
  ProcessPolygon(bev_map->crosswalks, crosswalk_id_polygon, 3);
  constexpr double traffic_distance = 50.0;
  TRAFFIC_LOG << fmt::format("bev_map_counter:{}", bev_map->header.cycle_counter);

  auto SetLightLanes = [&](std::vector<BevLaneInfo> &lanes_info) {
    for (auto &bev_lane : lanes_info) {
      if (!bev_lane.is_virtual || bev_lane.navi_action == BevAction::UNKNOWN) {
        continue;
      }

      // 打印推荐的虚拟车道线和其对应的action信息
      // AINFO << "bev_lane id: " << bev_lane.id;
      // AINFO << "action bind to bev_lane: " << static_cast<int>(bev_lane.navi_action);

      TrafficLight                                     light_t{};
      byd::msg::orin::routing_map::TrafficLightStatus *light_status{nullptr};
      switch (bev_lane.navi_action) {
        case BevAction::LEFT_TURN: {
          light_t      = traffic_lights_.left;
          light_status = &light_left_;
          break;
        }
        case BevAction::RIGHT_TURN: {
          light_t      = traffic_lights_.right;
          light_status = &light_right_;
          break;
        }
        case BevAction::STRAIGHT: {
          light_t      = traffic_lights_.straight;
          light_status = &light_straight_;
          break;
        }
        case BevAction::U_TURN: {
          light_t      = traffic_lights_.u_turn;
          light_status = &light_uturn_;
          break;
        }
        default:
          break;
      }

      bool  find_pos = false;
      Vec2d traffic_light_pos;
      TRAFFIC_LOG << fmt::format("bev_lane_id:{}", bev_lane.id);

      if (light_t.traffic_obj_info) {
        traffic_light_pos.set_x(light_t.traffic_obj_info->position.x);
        traffic_light_pos.set_y(light_t.traffic_obj_info->position.y);
        find_pos = true;
      } else {
        if (!deal_per_traffic_light_objects_.empty()) {
          const auto &obj = *deal_per_traffic_light_objects_.begin();
          traffic_light_pos.set_x(obj.second.position.x);
          traffic_light_pos.set_y(obj.second.position.y);
          find_pos = true;
        }
      }
      bool find_trf = false;
      if (find_pos) {
        auto it_jun = junction_id_polygon.find(bev_lane.junction_id);
        if (it_jun != junction_id_polygon.end()) {
          double dis_jun = it_jun->second.DistanceTo(traffic_light_pos);
          TRAFFIC_LOG << fmt::format("bev_find_junction_id:{}  dis:{:.2f}", it_jun->first, dis_jun);
          if (dis_jun < traffic_distance) {
            find_trf = true;
          }
        }
        for (uint64_t &stopline_id : bev_lane.stopline_ids) {
          auto it_stop = stopline_id_line2seg.find(stopline_id);
          if (!find_trf && it_stop != stopline_id_line2seg.end()) {
            double dis_stop = it_stop->second.DistanceTo(traffic_light_pos);
            TRAFFIC_LOG << fmt::format("bev_find_stopline_id:{}  dis_stop:{:.2f}", it_stop->first, dis_stop);
            if (dis_stop < traffic_distance) {
              find_trf = true;
              break;
            }
          }
        }
        for (uint64_t &traffic_crosswalk : bev_lane.traffic_crosswalks) {
          auto it_cross = crosswalk_id_polygon.find(traffic_crosswalk);
          if (!find_trf && it_cross != crosswalk_id_polygon.end()) {
            double dis_cross = it_cross->second.DistanceTo(traffic_light_pos);
            TRAFFIC_LOG << fmt::format("bev_find_crosswalk_id:{} dis_cross:{:.2f}", it_cross->first, dis_cross);
            if (dis_cross < traffic_distance) {
              find_trf = true;
              break;
            }
          }
        }
      }
      TRAFFIC_LOG << fmt::format("bev_traffic_t:{} find_pos:{:d}  find_trf:{:d}",
                                 light_t.traffic_obj_info ? light_t.traffic_obj_info->id : 0, find_pos, find_trf);

      if (light_t.is_valid && (find_trf || light_t.color == message::sensor::TLC_BLOCK_FAILED)) {
        if (light_status != nullptr) {
          light_status->add_lane_ids(bev_lane.id);
        }
        bev_lane.trafficlight_state         = ConvertLightState(light_t.color, light_t.traffic_light_flashing);
        bev_lane.traffic_light_num          = light_t.traffic_light_num;
        bev_lane.traffic_light_seq_num      = light_t.perception_seq_num;
        bev_lane.stay_prev_counter          = light_t.stay_prev_counter;
        bev_lane.traffic_set_reason         = light_t.traffic_reason;
        bev_lane.yellow_flashing_start_time = light_t.yellow_flashing_start_time;
        if (traffic_lights_.distance_to_stopline > k_epsilon && traffic_lights_.distance_to_stopline < 10) {
          bev_lane.stopline_angle_flag = 2;
        }
        if (light_t.traffic_obj_info) {
          bev_lane.traffic_light_obj_id = light_t.traffic_obj_info->id;
        }
      }
    }
  };
  SetLightLanes(bev_map->lane_infos);
  for (auto &sec : bev_map->route.sections) {
    SetLightLanes(sec.lane_infos);
  }
}

double TrafficLightMapping::LaneNearDisToJunction(const LaneInfo &ld_lane) {
  if (ld_lane.points.size() < 2 || deal_per_traffic_light_objects_.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const auto &traffic_light = [&]() -> const auto & {
    const auto &light_t = traffic_lights_.GetLightState(ld_lane.turn_type);
    return light_t.traffic_obj_info ? *light_t.traffic_obj_info : deal_per_traffic_light_objects_.begin()->second;
  }
  ();
  Eigen::Vector2d traffic_pos{traffic_light.position.x, traffic_light.position.y};
  Eigen::Vector2d point_start{ld_lane.points.front().x, ld_lane.points.front().y};
  Eigen::Vector2d point_end{ld_lane.points.back().x, ld_lane.points.back().y};
  double          dis_start = (traffic_pos - point_start).norm();
  double          dis_end   = (traffic_pos - point_end).norm();
  return std::min(dis_start, dis_end);
}

bool HeadValidCheck(const cem::message::common::Header &lhs, const cem::message::common::Header &rhs) {
  bool                     is_traffic_light_valid = true;
  static constexpr double  time_invalid_max       = 1;
  static constexpr uint8_t counter_invalid_max    = 10;
  if ((lhs.timestamp - rhs.timestamp) >= time_invalid_max) {
    TRAFFIC_LOG << fmt::format(
        "time exceed the threshold. current_time:{:.2f}  "
        "traffic_light_time:{:.2f}",
        lhs.timestamp, rhs.timestamp);
    is_traffic_light_valid = false;
  }
  // counter check
  if ((lhs.cycle_counter - rhs.cycle_counter) >= counter_invalid_max) {
    TRAFFIC_LOG << fmt::format(
        "counter exceed the threshold. current_counter:{}  "
        "traffic_light_counter:{}",
        lhs.cycle_counter, rhs.cycle_counter);
    is_traffic_light_valid = false;
  }
  return is_traffic_light_valid;
}

bool TrafficLightMapping::IsPrevLaneHasStopLine(const RoutingMapPtr &routing_map_output, TrafficLightStatus &traffic_light_status) {
  const auto &lanes = routing_map_output->lanes;
  for (const auto &lane_id : traffic_light_status.lane_ids) {
    const auto &it_lane = FindTheElement(lanes, lane_id);
    if (!it_lane.has_value()) {
      continue;
    }
    const auto &lane = **it_lane;
    for (const auto &prev_lane_id : lane.previous_lane_ids) {
      const auto &prev_lane = FindTheElement(lanes, prev_lane_id);
      if (!prev_lane.has_value() || (**prev_lane).traffic_stop_lines.empty()) {
        continue;
      }
      for (const auto &prev_stop_id : (**prev_lane).traffic_stop_lines) {
        const auto &stop_line = FindTheElement(routing_map_output->stop_lines, prev_stop_id);
        if (stop_line.has_value() && !(**stop_line).points.empty()) {
          traffic_light_status.stop_line_pts = (**stop_line).points;
          TRAFFIC_LOG << fmt::format("fint stop line:{1} of the previous lane:{0}", (*prev_lane)->id, traffic_light_status.stop_line_pts);
          return true;
        }
      }
    }
  }
  return false;
}

void TrafficLightMapping::GenerateStopLinePoints(const RoutingMapPtr &routing_map_output) {
  std::vector<TurnType> turn_all{TurnType::NO_TURN, TurnType::LEFT_TURN, TurnType::RIGHT_TURN, TurnType::U_TURN};
  for (auto &traffic_light_status : traffic_light_e2e_->traffic_status) {
    TRAFFIC_LOG << fmt::format("{}", traffic_light_status);
    traffic_light_status.is_navi_light = true;
    turn_all.erase(std::remove_if(turn_all.begin(), turn_all.end(),
                                  [&traffic_light_status](const TurnType &turn) { return traffic_light_status.turn_type == turn; }),
                   turn_all.end());
    if (traffic_light_status.lane_ids.empty()) {
      return;
    }
    if (IsPrevLaneHasStopLine(routing_map_output, traffic_light_status)) {
      continue;
    }
    const auto &boundaries = routing_map_output->lane_boundaries;
    const auto &lanes      = routing_map_output->lanes;

    std::optional<LineSegment2d> segment_t;
    double                       left_dis{std::numeric_limits<double>::infinity()};
    double                       right_dis{std::numeric_limits<double>::lowest()};
    Vec2d                        left_point;
    Vec2d                        right_point;

    for (const auto &lane_id : traffic_light_status.lane_ids) {
      const auto &it_lane = FindTheElement(lanes, lane_id);
      if (!it_lane.has_value() || (**it_lane).points.empty()) {
        continue;
      }
      const auto &lane = **it_lane;

      fmt::memory_buffer point_buf;
      fmt::format_to(point_buf, "lane_id:{}  lane_point:{},{}", lane.id, lane.points.front(), lane.points.back());
      if (!segment_t.has_value()) {
        segment_t = LineSegment2d({Vec2d(lane.points.front().x, lane.points.front().y), Vec2d(lane.points.back().x, lane.points.back().y)});
      }
      auto it_left_bound = FindTheElement(boundaries, lane.left_lane_boundary_ids.front());
      if (it_left_bound.has_value() && (*it_left_bound)->points.size() > 2) {
        fmt::format_to(point_buf, "  id:{} left_bound_point:{}", lane.left_lane_boundary_ids.front(),
                       it_left_bound.value()->points.front());
        Vec2d left_p_t = Vec2d((*it_left_bound)->points.front().x, (*it_left_bound)->points.front().y);
        if (auto dis = segment_t->ProductOntoUnit(left_p_t); dis < left_dis) {
          left_dis   = dis;
          left_point = left_p_t;
        }
      }
      auto it_right_bound = FindTheElement(boundaries, lane.right_lane_boundary_ids.front());
      if (it_right_bound.has_value() && (*it_right_bound)->points.size() > 2) {
        fmt::format_to(point_buf, "  id:{} right_bound_point:{}", lane.right_lane_boundary_ids.front(),
                       it_right_bound.value()->points.front());
        Vec2d right_p_t = Vec2d((*it_right_bound)->points.front().x, (*it_right_bound)->points.front().y);
        if (auto dis = segment_t->ProductOntoUnit(right_p_t); dis > right_dis) {
          right_dis   = dis;
          right_point = right_p_t;
        }
      }
      // TRAFFIC_LOG << std::string_view(point_buf.data(), point_buf.size());
    }
    if (segment_t->length() > 1.0 && left_point.DistanceTo(right_point) > 1.0) {
      static constexpr double stop_line_width = 0.2;
      static constexpr double extend_len      = 6.0;
      LineSegment2d           seg1{left_point, right_point};
      left_point -= seg1.unit_direction() * extend_len;
      right_point += seg1.unit_direction() * extend_len;
      LineSegment2d seg2     = TranslateSegmentAlongAxisStart(*segment_t, {left_point, right_point}, stop_line_width);
      auto         &line_pts = traffic_light_status.stop_line_pts;
      line_pts.clear();
      line_pts.emplace_back<Point2D>({left_point.x(), left_point.y()});
      line_pts.emplace_back<Point2D>({right_point.x(), right_point.y()});
      line_pts.emplace_back<Point2D>({seg2.end().x(), seg2.end().y()});
      line_pts.emplace_back<Point2D>({seg2.start().x(), seg2.start().y()});
      TRAFFIC_LOG << fmt::format("stop_line_points based on bounds:{}", line_pts);
    }
  }
  std::vector<Point2D> points;
  if (!traffic_light_e2e_->traffic_status.empty()) {
    points = traffic_light_e2e_->traffic_status.front().stop_line_pts;
  }
  for (TurnType turn_type : turn_all) {
    auto       &traffic_status       = traffic_light_e2e_->traffic_status.emplace_back();
    const auto &light_status         = traffic_lights_.GetLightState(turn_type);
    traffic_status.turn_type         = turn_type;
    traffic_status.stop_line_pts     = points;
    traffic_status.is_navi_light     = false;
    traffic_status.light_status      = LightColorConvert(light_status.color, light_status.traffic_light_flashing);
    traffic_status.traffic_light_num = light_status.traffic_light_num;
  }
  TRAFFIC_LOG << fmt::format("traffic_e2e_info:{}", traffic_light_e2e_->traffic_status);
}

void TrafficLightMapping::SetE2EInfo(const RoutingMapPtr &routing_map_output, std::vector<LaneInfo> &lanes_traffic_all) {
  if (lanes_traffic_all.empty()) {
    TRAFFIC_LOG << "lanes_traffic is empty.";
    return;
  }
  std::unordered_set<uint64_t> present_ids;
  for (const auto &lane : lanes_traffic_all) {
    present_ids.insert(lane.id);
  }
  TRAFFIC_LOG << fmt::format("raw_lanes. size:{}", lanes_traffic_all.size());
  // for (const LaneInfo &lane : lanes_traffic_all) {
  // TRAFFIC_LOG << fmt::format(
  //     "lane_id:{} prev_lanes:{} left_lanes:{}  right_lanes:{} turn:{} "
  //     "light_status:{}",
  //     lane.id, lane.previous_lane_ids, lane.left_lane_id, lane.right_lane_id, lane.turn_type, lane.light_status);
  // }

  // remove the lanes in lanes_traffic_all whose prev-lane has been in lanes_traffic_all.
  // lanes_traffic_all.erase(
  //     std::remove_if(lanes_traffic_all.begin(), lanes_traffic_all.end(),
  //                    [&](const LaneInfo& lane_t) {
  //                      return std::any_of(lane_t.previous_lane_ids.begin(),
  //                                         lane_t.previous_lane_ids.end(),
  //                                         [&](uint64_t prev_id) {
  //                                           return present_ids.count(prev_id) >
  //                                                  0;
  //                                         });
  //                    }),
  //     lanes_traffic_all.end());
  std::sort(lanes_traffic_all.begin(), lanes_traffic_all.end(), [](const LaneInfo &lhs, const LaneInfo &rhs) {
    if (lhs.turn_type != rhs.turn_type) {
      return lhs.turn_type < rhs.turn_type;
    }
    return lhs.light_status < rhs.light_status;
  });
  TRAFFIC_LOG << "filtered_lanes. size:" << lanes_traffic_all.size();
  if (lanes_traffic_all.empty()) {
    return;
  }
  // for (const auto &lane : lanes_traffic_all) {
  // TRAFFIC_LOG << fmt::format("lane_id:{} turn:{} light_status:{}", lane.id, magic_enum::enum_name(lane.turn_type),
  //  magic_enum::enum_name(lane.light_status));
  // }
  TrafficLightStatus traffic_light_status_t;
  size_t             idx                   = 0;
  traffic_light_status_t.light_status      = lanes_traffic_all[idx].light_status;
  traffic_light_status_t.turn_type         = lanes_traffic_all[idx].turn_type;
  traffic_light_status_t.traffic_light_num = lanes_traffic_all[idx].light_countdown;
  traffic_light_status_t.lane_ids.emplace_back(lanes_traffic_all[idx].id);
  traffic_light_e2e_->traffic_status.push_back(traffic_light_status_t);
  for (idx++; idx < lanes_traffic_all.size(); idx++) {
    if (lanes_traffic_all[idx].light_status == traffic_light_status_t.light_status &&
        lanes_traffic_all[idx].turn_type == traffic_light_status_t.turn_type) {
      traffic_light_e2e_->traffic_status.back().lane_ids.emplace_back(lanes_traffic_all[idx].id);
      continue;
    }
    traffic_light_status_t.light_status      = lanes_traffic_all[idx].light_status;
    traffic_light_status_t.turn_type         = lanes_traffic_all[idx].turn_type;
    traffic_light_status_t.traffic_light_num = lanes_traffic_all[idx].light_countdown;
    traffic_light_status_t.lane_ids.emplace_back(lanes_traffic_all[idx].id);
    traffic_light_e2e_->traffic_status.push_back(traffic_light_status_t);
  }
  GenerateStopLinePoints(routing_map_output);
}

void TrafficLightMapping::ProcessNotRoutingLanes(const RoutingMapPtr &routing_map_output, std::vector<LaneInfo> &traffic_light_lanes) {
  if (traffic_light_lanes.empty()) {
    return;
  }
  auto                        &lanes      = routing_map_output->lanes;
  const auto                  &stop_lines = routing_map_output->stop_lines;
  std::unordered_set<uint64_t> junction_id_set;
  for (const auto &lane_info : traffic_light_lanes) {
    if (lane_info.junction_id != 0) {
      junction_id_set.insert(lane_info.junction_id);
    }
  }
  const auto              &routing_sections       = routing_map_output->route.sections;
  uint64_t                 ego_section_id         = routing_map_output->route.navi_start.section_id;
  static constexpr int64_t max_extend_section_num = 3;
  auto                     ego_section_it         = FindTheElement(routing_sections, ego_section_id);
  if (!ego_section_it.has_value()) {
    return;
  }
  int64_t                      farthest_section_idx = 0;
  std::unordered_set<uint64_t> lanes_light_set;
  for (const auto &lane_info_t : traffic_light_lanes) {
    lanes_light_set.insert(lane_info_t.id);
    auto section_idx = FindTheElement(routing_sections, lane_info_t.section_id);
    if (section_idx.has_value()) {
      if (auto dis = std::distance(routing_sections.begin(), *section_idx); dis > farthest_section_idx) {
        farthest_section_idx = dis;
      }
    }
  }
  int64_t farthest_succeed = farthest_section_idx - std::distance(routing_sections.begin(), *ego_section_it) + max_extend_section_num;
  static constexpr int64_t max_recursive_depth = 40;
  TRAFFIC_LOG << "farthest_succeed:" << farthest_succeed;

  if (farthest_succeed <= 0) {
    return;
  }

  std::function<void(const LaneInfo &, int)> SetNextLaneTrafficInfo;
  SetNextLaneTrafficInfo = [&](const LaneInfo &lane, int recursive_num) -> void {
    if (lanes_light_set.count(lane.id) == 0 &&
        ((lane.junction_id != 0 && junction_id_set.count(lane.junction_id) > 0) || lane.turn_type == TurnType::LEFT_TURN ||
         lane.turn_type == TurnType::RIGHT_TURN ||
         (lane.turn_type == TurnType::U_TURN && (lane.junction_id != 0U && junction_id_set.count(lane.junction_id) > 0)))) {
      // traffic_light_lanes.emplace_back(lane);
      TRAFFIC_LOG << fmt::format(
          "SetNextLaneTrafficInfo lane_id:{}  section_id:{}  junction_id:{}  "
          "turn_type:{} recursive_num:{}",
          lane.id, lane.section_id, lane.junction_id, lane.turn_type, recursive_num);
      SetLaneTrafficInfo(lane, lanes, stop_lines);
    }
    if (recursive_num >= farthest_succeed || recursive_num > max_recursive_depth) {
      return;
    }
    if (lane.next_lane_ids.empty()) {
      return;
    }
    for (auto lane_id : lane.next_lane_ids) {
      auto lane_it = FindTheElement(lanes, lane_id);
      if (!lane_it.has_value()) {
        continue;
      }
      SetNextLaneTrafficInfo(**lane_it, recursive_num + 1);
    }
  };

  for (const auto &lane_id : (*ego_section_it)->lane_ids) {
    auto lane = FindTheElement(lanes, lane_id);
    if (!lane.has_value()) {
      continue;
    }
    SetNextLaneTrafficInfo(**lane, 0);
  }
}

std::tuple<uint64_t, bool, bool> TrafficLightMapping::LaneStopLineInfo(const std::vector<LaneInfo>     &lanes_output,
                                                                       const std::vector<StopLineInfo> &stop_lines, uint64_t id_temp) {
  auto it_lane_temp =
      std::find_if(lanes_output.begin(), lanes_output.end(), [&id_temp](const auto &lane_t) { return id_temp == lane_t.id; });
  if (it_lane_temp == lanes_output.end()) {
    return std::make_tuple(0, false, false);
  }
  bool has_stop_line = !it_lane_temp->traffic_stop_lines.empty() && it_lane_temp->traffic_stop_lines.front() != 0;
  if (!has_stop_line) {
    return std::make_tuple(0, false, false);
  }
  bool stop_line_front{false};
  bool stop_line_back{false};
  if (it_lane_temp->points.size() >= 2) {
    if (auto stop_line = FindTheElement(stop_lines, it_lane_temp->traffic_stop_lines.front());
        stop_line.has_value() && !stop_line.value()->points.empty()) {
      Vec2d point_start(it_lane_temp->points.back().x, it_lane_temp->points.back().y);
      Vec2d point_end(it_lane_temp->points.front().x, it_lane_temp->points.front().y);
      Vec2d stop_point(stop_line.value()->points.front().x, stop_line.value()->points.front().y);
      if (point_start.DistanceTo(stop_point) > point_end.DistanceTo(stop_point)) {
        stop_line_front = true;
      } else {
        stop_line_back = true;
      }
    }
  }
  return std::make_tuple(it_lane_temp->traffic_stop_lines.front(), stop_line_front, stop_line_back);
};

std::optional<LaneInfo> TrafficLightMapping::SetLaneTrafficInfo(const LaneInfo &lane_traffic, std::vector<LaneInfo> &lanes_output,
                                                                const std::vector<StopLineInfo> &stop_lines) {
  cem::message::env_model::TurnType turn_temp{lane_traffic.turn_type};
  if (lane_traffic.type == cem::message::env_model::LaneType::LANE_LEFT_WAIT) {
    if (traffic_lights_.left.is_valid &&
        (traffic_lights_.left.color == message::sensor::TLC_GREEN || traffic_lights_.left.color == message::sensor::TLC_GREEN_FLASHING)) {
      turn_temp = TurnType::LEFT_TURN;
    } else {
      turn_temp = TurnType::NO_TURN;
    }
  }
  if (lane_traffic.turn_type == TurnType::RIGHT_TURN && IsSectionEnteringRoundabout(lane_traffic.section_id)) {
    TRAFFIC_LOG << fmt::format("lane_id:{} section_id:{} is_entering_roundabout.", lane_traffic.id, lane_traffic.section_id);
    turn_temp = TurnType::NO_TURN;
  }
  const auto &light_status = traffic_lights_.GetLightState(turn_temp);
  if (!light_status.is_valid) {
    TRAFFIC_LOG << "  lane_id:" << lane_traffic.id << "  turn_type:" << static_cast<int>(lane_traffic.turn_type);
    return nullopt;
  }
  auto it_lane_output =
      std::find_if(lanes_output.begin(), lanes_output.end(), [&lane_traffic](const auto &lane_t) { return lane_traffic.id == lane_t.id; });
  if (it_lane_output == lanes_output.end()) {
    TRAFFIC_LOG << fmt::format("not find the lane:{}", lane_traffic.id);
    return nullopt;
  }
  switch (turn_temp) {
    case TurnType::LEFT_TURN:
      light_left_.add_lane_ids(it_lane_output->id);
      break;
    case TurnType::RIGHT_TURN:
      light_right_.add_lane_ids(it_lane_output->id);
      break;
    case TurnType::U_TURN:
      light_uturn_.add_lane_ids(it_lane_output->id);
      break;
    default:
      light_straight_.add_lane_ids(it_lane_output->id);
      break;
  }
  it_lane_output->light_status          = LightColorConvert(light_status.color, light_status.traffic_light_flashing);
  it_lane_output->light_countdown       = light_status.traffic_light_num;
  it_lane_output->traffic_light_obj_id  = light_status.traffic_obj_info ? light_status.traffic_obj_info->id : 0;
  it_lane_output->traffic_light_seq_num = light_status.perception_seq_num;
  it_lane_output->traffic_set_reason    = light_status.traffic_reason;
  it_lane_output->stay_prev_counter     = light_status.stay_prev_counter;

  it_lane_output->cloud_junction_type      = cloud_traffic_light_ ? cloud_traffic_light_->junction_cloud_type : 0;
  it_lane_output->traffic_match_cloud_file = cloud_traffic_light_.has_value();

  it_lane_output->yellow_flashing_start_time = light_status.yellow_flashing_start_time;

  bool is_special_lane = it_lane_output->id == 70106365 || it_lane_output->id == 70108379;
  bool is_special_road = traffic_lights_.distance_to_stopline > k_epsilon && traffic_lights_.distance_to_stopline < 10;
  if (is_special_lane || is_special_road) {
    it_lane_output->stopline_angle_flag = 2;
  }

  if (it_lane_output->junction_id == 0) {
    bool need_set_junction_id = true;
    if (!it_lane_output->cross_walks.empty() && it_lane_output->cross_walks.front() != 0) {
      for (uint64_t id_prev : it_lane_output->previous_lane_ids) {
        auto lane_prev_t = FindTheElement(lanes_output, id_prev);
        if (lane_prev_t.has_value() && !lane_prev_t.value()->cross_walks.empty() && lane_prev_t->base()->cross_walks.front() != 0) {
          need_set_junction_id = false;
        }
      }
      if (need_set_junction_id) {
        TRAFFIC_LOG << fmt::format("lane_id:{} has_cross id_be_set 1", it_lane_output->id);
        it_lane_output->junction_id = 1;
      }
    }

    auto [stop_line_id, stop_line_front, stop_line_back] = LaneStopLineInfo(lanes_output, stop_lines, it_lane_output->id);
    if (need_set_junction_id && it_lane_output->junction_id == 0 && stop_line_id != 0 && stop_line_front) {
      TRAFFIC_LOG << fmt::format("lane_id:{} has_stop_front id_be_set 1", it_lane_output->id);
      it_lane_output->junction_id = 1;
    }

    if (it_lane_output->turn_type == message::env_model::TurnType::U_TURN) {
      TRAFFIC_LOG << fmt::format("lane_id:{} is_uturn id_be_set 1", it_lane_output->id);
      it_lane_output->junction_id = 1;
    }

    if (need_set_junction_id && it_lane_output->junction_id == 0) {
      for (uint64_t id_prev : it_lane_output->previous_lane_ids) {
        auto [stop_line_id, stop_line_front, stop_line_back] = LaneStopLineInfo(lanes_output, stop_lines, id_prev);
        if (stop_line_id != 0 && stop_line_back) {
          TRAFFIC_LOG << fmt::format("prev_lane_id:{} has_stop_back id_be_set 1", it_lane_output->id);
          it_lane_output->junction_id = 1;
          break;
        }
      }
    }

    if (it_lane_output->junction_id == 1) {
      it_lane_output->type       = message::env_model::LaneType::LANE_VIRTUAL_JUNCTION;
      it_lane_output->is_virtual = true;
    }
  }
  TRAFFIC_LOG << routing_map_ptr_->header.cycle_counter << " lane_id:" << it_lane_output->id
              << "  junction_id:" << it_lane_output->junction_id << "  section_id:" << it_lane_output->section_id
              << "  is_left_wait:" << static_cast<bool>(lane_traffic.type == cem::message::env_model::LaneType::LANE_LEFT_WAIT)
              << "  light_status:" << magic_enum::enum_name(it_lane_output->light_status) << "  num:" << it_lane_output->light_countdown;
  info_traffic_lights_ += fmt::format("{},{},{:d};", it_lane_output->id, it_lane_output->light_status,
                                      light_status.traffic_light_num > 200 ? 200 : light_status.traffic_light_num);
  return *it_lane_output;
}

void TrafficLightMapping::SetJunctionPoints(const RoutingMapPtr &routing_map, const std::vector<LaneInfo> &lanes_traffic_all) {
  std::vector<Point2D> junction_points;
  std::vector<Point2D> cross_points;
  for (const auto &lane : lanes_traffic_all) {
    if (lane.junction_id <= 1) {
      continue;
    }
    auto it_junction = FindTheElement(routing_map->junctions, lane.junction_id);
    if (it_junction) {
      junction_points = (**it_junction).points;
      if (junction_points.size() >= 4) {
        TRAFFIC_LOG << fmt::format("current_junction_points junction_id:{}  point:{:.2f},{:.2f}", lane.junction_id,
                                   junction_points.front().x, junction_points.front().y);
        break;
      }
    }
    for (auto id_cross : lane.cross_walks) {
      auto it_cross = FindTheElement(routing_map->cross_walks, id_cross);
      if (it_cross) {
        cross_points = (**it_cross).points;
        if (cross_points.size() >= 4) {
          TRAFFIC_LOG << fmt::format("current_junction_points cross_id:{}  point:{:.2f},{:.2f}", id_cross, cross_points.front().x,
                                     cross_points.front().y);
          break;
        }
      }
    }
    if (cross_points.size() >= 4) {
      break;
    }
  }
  if (!junction_points.empty()) {
    previous_junction_points_ = junction_points;
  } else if (!cross_points.empty()) {
    previous_junction_points_ = cross_points;
  } else {
    previous_junction_points_.clear();
  }
}

double TrafficLightMapping::GetPreviousJunctionEgoPassed() {
  constexpr double kInvalidDistance = std::numeric_limits<double>::infinity();
  if (!routing_map_ptr_ || junction_ids_vec_.empty()) {
    return kInvalidDistance;
  }
  const auto &sections  = routing_map_ptr_->route.sections;
  const auto &route_ego = routing_map_ptr_->route.navi_start;
  TRAFFIC_LOG << fmt::format("ego_section_id:{}  s_offset:{:.2f}", route_ego.section_id, route_ego.s_offset);
  auto it_section =
      std::find_if(sections.rbegin(), sections.rend(), [&](const auto &section) { return route_ego.section_id == section.id; });
  if (it_section == sections.rend()) {
    return kInvalidDistance;
  }
  auto it_val = std::find(junction_ids_vec_.rbegin(), junction_ids_vec_.rend(), it_section->id);
  if (it_val != junction_ids_vec_.rend()) {
    info_traffic_lights_ += fmt::format("  ego_pass_junction:{}  dis:{:.2f}  ", *it_val, route_ego.s_offset);
    return route_ego.s_offset;
  }
  double dis = route_ego.s_offset;
  for (++it_section; it_section != sections.rend(); ++it_section) {
    dis += it_section->length;
    if (it_val = std::find(junction_ids_vec_.rbegin(), junction_ids_vec_.rend(), it_section->id); it_val != junction_ids_vec_.rend()) {
      info_traffic_lights_ += fmt::format("  ego_pass_junction:{}  dis:{:.2f}  ", *it_val, dis);
      return dis;
    }
  }
  return kInvalidDistance;
}

void TrafficLightMapping::GetJunctionIdsPool(const RoutingMapPtr &routing_map) {
  const auto &sections           = routing_map->route.sections;
  const auto &lanes              = routing_map->lanes;
  auto        GetJunctionPolygon = [&](uint64_t junction_id) {
    auto it_junction = FindTheElement(routing_map->junctions, junction_id);
    if (it_junction && it_junction.value()->points.size() >= 4) {
      std::vector<Vec2d> vec_points;
      for (const auto &p : it_junction.value()->points) {
        vec_points.emplace_back(p.x, p.y);
      }
      return byd::common::math::Polygon2d{vec_points};
    }
    return byd::common::math::Polygon2d{};
  };
  for (size_t idx = 0; idx < sections.size(); idx++) {
    const auto &section_current = sections.at(idx);
    for (uint64_t lane_id : section_current.lane_ids) {
      auto it_lane = std::find_if(lanes.begin(), lanes.end(), [&lane_id](const auto &lane_t) { return lane_t.id == lane_id; });
      if (it_lane == lanes.end()) {
        continue;
      }
      if (IsLinkType(section_current.link_type, byd::msg::orin::routing_map::SectionInfo::LT_TOLLBOOTH)) {
        LD_TRAFFIC_LOG << fmt::format("skip lane_id:{}  section_id:{}  it is a tolltooth.", it_lane->id, it_lane->section_id);
        continue;
      }
      if (it_lane->type == message::env_model::LaneType::LANE_VIRTUAL_JUNCTION && it_lane->junction_id != 0) {
        TRAFFIC_LOG << fmt::format("find_ld_junction:{}  virtual_lane:{}", it_lane->junction_id, lane_id);
        ldmap_junction_ids_.insert({it_lane->junction_id, GetJunctionPolygon(it_lane->junction_id)});
        break;
      }
      if (idx > 0) {
        bool find_virtual_junction{false};
        for (uint64_t lane_temp : sections.at(idx - 1).lane_ids) {
          auto [stop_line_id, _, stopline_in_back] = LaneStopLineInfo(lanes, routing_map->stop_lines, lane_temp);

          auto it_lane_temp = FindTheElement(lanes, lane_temp);
          if (it_lane_temp == lanes.end()) {
            continue;
          }
          if (stopline_in_back) {
            TRAFFIC_LOG << fmt::format("find_stopline_junction:{}  prev_stop_line:{}  junction_id:{} current_section_id:{}",
                                       it_lane->junction_id, lane_temp, it_lane->junction_id, section_current.id);
            if (it_lane->junction_id != 0 && it_lane_temp.value()->junction_id == 0) {
              ldmap_junction_ids_.insert({it_lane->junction_id, GetJunctionPolygon(it_lane->junction_id)});
            } else {
              std::set<uint64_t> stopline_set;
              for (uint64_t lane_temp_02 : sections.at(idx - 1).lane_ids) {
                auto it_lane_temp_02 = FindTheElement(lanes, lane_temp_02);
                if (it_lane_temp_02 == lanes.end()) {
                  continue;
                }
                for (uint64_t stopline_id_t : it_lane_temp_02.value()->traffic_stop_lines) {
                  stopline_set.insert(stopline_id_t);
                }
              }
              for (uint64_t stopline_id_t : stopline_set) {
                auto stop_po = GetTheElementOfMap(routing_map->stop_lines, stopline_id_t, dis_stopline_pan);
                maybe_junctions_.insert({section_current.id, stop_po});
                TRAFFIC_LOG << fmt::format("section_id:{} stopline_id_t:{} lane_id:{} point:{}", section_current.id, stopline_id_t,
                                           it_lane->id, stop_po.points().size());
              }
            }
            find_virtual_junction = true;
            break;
          }
        }
        if (find_virtual_junction) {
          break;
        }
      } else {
        auto [stop_line_id, stopline_in_front, _] = LaneStopLineInfo(lanes, routing_map->stop_lines, lane_id);
        if (stopline_in_front) {
          TRAFFIC_LOG << fmt::format("find_ld_junction:{}  current_stop_line:{}  junction_id:{} current_section_id:{}",
                                     it_lane->junction_id, lane_id, it_lane->junction_id, section_current.id);
          if (it_lane->junction_id != 0) {
            ldmap_junction_ids_.insert({it_lane->junction_id, GetJunctionPolygon(it_lane->junction_id)});
          } else {
            maybe_junctions_.insert({section_current.id, GetTheElementOfMap(routing_map->stop_lines, stop_line_id, dis_stopline_pan)});
          }
          break;
        }
      }
    }
  }
}

double TrafficLightMapping::DistanceToJunction(const byd::common::math::Polygon2d &polygon, const TrafficLight &light) {
  double dis = std::numeric_limits<double>::infinity();
  if (!light.is_valid || !light.traffic_obj_info) {
    return dis;
  }
  if (polygon.points().empty()) {
    return dis;
  }
  Vec2d light_pos{light.traffic_obj_info->position.x, light.traffic_obj_info->position.y};
  return polygon.DistanceTo(light_pos);
}

bool TrafficLightMapping::LaneCrossHasTrafficLight(const LaneInfo &lane_rhs) {
  if (!routing_map_ptr_ || lane_rhs.points.size() < 2) {
    return false;
  }

  bool lane_has_crosswald = !lane_rhs.cross_walks.empty() && lane_rhs.cross_walks.front() != 0;
  if (!lane_has_crosswald) {
    return false;
  }
  bool is_light_block_failed = traffic_lights_.GetLightState(lane_rhs.turn_type).color == message::sensor::TLC_BLOCK_FAILED;

  auto lanes_is_in_junction = [&](const std::vector<uint64_t> &lane_ids) {
    return std::any_of(lane_ids.begin(), lane_ids.end(), [&](uint64_t lane_id_t) {
      auto it_prev_lane_t = FindTheElement(routing_map_ptr_->lanes, lane_id_t);
      if (it_prev_lane_t.has_value()) {
        return (*it_prev_lane_t)->type == message::env_model::LaneType::LANE_VIRTUAL_JUNCTION ||
               (*it_prev_lane_t)->type == message::env_model::LaneType::LANE_VIRTUAL_COMMON || (*it_prev_lane_t)->junction_id != 0;
      }
      return false;
    });
  };
  bool   prev_lane_is_in_junction = lanes_is_in_junction(lane_rhs.previous_lane_ids);
  bool   succ_lane_is_in_junction = lanes_is_in_junction(lane_rhs.next_lane_ids);
  bool   is_right_dedicated       = sd_navigation_city_ && sd_navigation_city_->IsEgoInDedicatedRightLane();
  double dis_threshold            = lane_belong_traffic_light_threshold;
  if (is_right_dedicated) {
    dis_threshold = dis_crosswalk_right_dedicated;
  }

  const auto &traffic_light = GetTrafficLightBaseTurnType(lane_rhs.turn_type);
  const auto &polygon       = GetTheElementOfMap(routing_map_ptr_->cross_walks, lane_rhs.cross_walks.front());

  if (polygon.points().size() < 3) {
    return false;
  }
  double max_x = std::numeric_limits<double>::lowest();
  for (const auto &point : polygon.points()) {
    if (max_x < point.x()) {
      max_x = point.x();
    }
  }
  if (max_x < -3.0) {
    TRAFFIC_LOG << fmt::format("max_x:{:.2f}", max_x);
    return false;
  }

  lane_has_crosswald = polygon.HasOverlap(
      LineSegment2d{{lane_rhs.points.begin()->x, lane_rhs.points.begin()->y}, {lane_rhs.points.rbegin()->x, lane_rhs.points.rbegin()->y}});
  double dis_polygon          = traffic_light ? polygon.DistanceTo(Vec2d{(*traffic_light).position.x, (*traffic_light).position.y})
                                              : std::numeric_limits<double>::infinity();
  bool   is_in_small_junction = !prev_lane_is_in_junction && !succ_lane_is_in_junction && lane_has_crosswald &&
                              (dis_polygon < dis_threshold || is_light_block_failed);
  if (is_in_small_junction) {
    double min_x = std::numeric_limits<double>::infinity();
    for (const auto &poi : polygon.points()) {
      min_x = min_x > poi.x() ? poi.x() : min_x;
    }
    if (min_x - traffic_light->position.x > 10 && traffic_light->position.x < 50.0) {
      TRAFFIC_LOG << fmt::format("crosswalk_not_link_traffic_light min_x:{:.2f} traffic_pos:{:.2f}", min_x, traffic_light->position.x);
      is_in_small_junction = false;
    }
  }
  // bool is_in_small_junction = !prev_lane_is_in_junction && lane_has_crosswald && (dis_polygon < dis_threshold || is_light_block_failed);
  TRAFFIC_LOG << fmt::format("crosswalk lane_id:{} intersection:{:d}  dis_polygon:{:.2f}  block:{:d}  right_only:{:d} res:{:d} counter:{}",
                             lane_rhs.id, lane_has_crosswald, dis_polygon, is_light_block_failed, is_right_dedicated, is_in_small_junction,
                             routing_map_ptr_->header.cycle_counter);
  return is_in_small_junction;
}

bool TrafficLightMapping::LaneStoplineTrafficLight(uint64_t section_id, const LaneInfo &lane_rhs, double min_x_threshold,
                                                   std::optional<TrfObjectInfo> tra_obj) {
  auto find_stopline = maybe_junctions_.find(section_id);
  if (find_stopline == maybe_junctions_.end() || lane_rhs.points.size() < 2) {
    return false;
  }
  bool is_light_block_failed = traffic_lights_.GetLightState(lane_rhs.turn_type).color == message::sensor::TLC_BLOCK_FAILED;
  bool is_right_dedicated    = sd_navigation_city_ && sd_navigation_city_->IsEgoInDedicatedRightLane();

  constexpr double extend_lenght = 4.0;

  double dis_threshold = lane_belong_traffic_light_threshold;
  if (is_right_dedicated) {
    dis_threshold = dis_crosswalk_right_dedicated;
  }

  std::optional<TrfObjectInfo> traffic_light = std::move(tra_obj);
  if (!traffic_light) {
    traffic_light = GetTrafficLightBaseTurnType(lane_rhs.turn_type);
  }

  Vec2d start_p{lane_rhs.points.begin()->x, lane_rhs.points.begin()->y};
  Vec2d end_p{lane_rhs.points.rbegin()->x, lane_rhs.points.rbegin()->y};

  if (start_p.x() < min_x_threshold || end_p.x() < min_x_threshold) {
    TRAFFIC_LOG << fmt::format("start_x:{:.2f}  end_x:{:.2f}", start_p.x(), end_p.x());
    return false;
  }

  LineSegment2d lane_segment_t{start_p, end_p};
  LineSegment2d lineseg_stopline{start_p - lane_segment_t.unit_direction() * extend_lenght,
                                 end_p + lane_segment_t.unit_direction() * extend_lenght};

  bool   lane_through_stopline{false};
  double dis_polygon{std::numeric_limits<double>::infinity()};
  auto   range = maybe_junctions_.equal_range(section_id);

  bool is_valid_traffic_light = true;
  for (auto it = range.first; it != range.second; ++it) {
    if (it->second.points().size() < 3) {
      continue;
    }
    lane_through_stopline = it->second.HasOverlap(lineseg_stopline);
    if (lane_through_stopline) {
      if (traffic_light) {
        dis_polygon  = it->second.DistanceTo(Vec2d{(*traffic_light).position.x, (*traffic_light).position.y});
        double min_x = std::numeric_limits<double>::infinity();
        for (const auto &poi : it->second.points()) {
          min_x = min_x > poi.x() ? poi.x() : min_x;
        }
        if (min_x - traffic_light->position.x > 10 && traffic_light->position.x < 50.0) {
          TRAFFIC_LOG << fmt::format("stopline_not_link_traffic_light min_x:{:.2f}  traffic_light_x:{:.2f}", min_x,
                                     traffic_light->position.x);
          is_valid_traffic_light = false;
        }
      }
      break;
    }
  }
  bool lane_has_stop_line = lane_through_stopline && (is_light_block_failed || dis_polygon < dis_threshold) && is_valid_traffic_light;
  TRAFFIC_LOG << fmt::format("stopline  lane_id:{} intersection:{:d}  dis_poly:{:.2f} block:{:d} right_only:{:d} res:{:d} counter:{:d}",
                             lane_rhs.id, lane_through_stopline, dis_polygon, is_light_block_failed, is_right_dedicated, lane_has_stop_line,
                             routing_map_ptr_->header.cycle_counter);
  return lane_has_stop_line;
}

uint64_t TrafficLightMapping::GetHisTimes(TurnType turn_type, uint64_t id) {
  const auto &seq_his = history_ids_[turn_type];
  uint64_t    res     = 0;
  for (const auto &[_, id_t] : seq_his) {
    if (id_t == id) {
      res++;
    }
  }
  return res;
}

void TrafficLightMapping::UpdateHistoryObjId() {
  if (!perception_traffic_light_info_) {
    return;
  }
  uint64_t seq_num       = perception_traffic_light_info_->header.cycle_counter;
  auto     UpdateTurnHis = [&](const TrafficLight &traffic_light, TurnType turn_type) {
    auto                 &seq = history_ids_[turn_type];
    std::vector<uint64_t> ids;
    for (const auto &[_, id_t] : seq) {
      ids.emplace_back(id_t);
    }
    LD_TRAFFIC_LOG << fmt::format("seq_num:{} turn_type:{}  ids:{}", seq_num, magic_enum::enum_name(turn_type), ids);
    if (traffic_light.traffic_obj_info) {
      if (!seq.empty() && seq.back().first == seq_num) {
        seq.pop_back();
      }
      seq.emplace_back(seq_num, traffic_light.traffic_obj_info->id);
    } else {
      seq.emplace_back(seq_num, 0);
    }
  };
  UpdateTurnHis(traffic_lights_ld_.straight, TurnType::NO_TURN);
  UpdateTurnHis(traffic_lights_ld_.left, TurnType::LEFT_TURN);
  UpdateTurnHis(traffic_lights_ld_.right, TurnType::RIGHT_TURN);
  UpdateTurnHis(traffic_lights_ld_.u_turn, TurnType::U_TURN);
}

void TrafficLightMapping::PrevStopWhenLeftWait(const RoutingMapPtr &routing_map, const RoutingMapPtr &routing_map_output,
                                               uint64_t junction_id, double distance_to_junction) {
  auto it_section = std::find_if(routing_map->route.sections.begin(), routing_map->route.sections.end(),
                                 [&routing_map](const auto &section) { return routing_map->route.navi_start.section_id == section.id; });
  if (it_section == routing_map->route.sections.end()) {
    return;
  }
  if (distance_to_junction > 40) {
    return;
  }
  if (!traffic_lights_.left.traffic_obj_info) {
    return;
  }
  if (!junction_orin_vertical_) {
    return;
  }
  const auto &left_obj      = traffic_lights_.left.traffic_obj_info;
  uint64_t    left_his_time = GetHisTimes(TurnType::LEFT_TURN, left_obj->id);
  Point3DD    obj_pos{left_obj->position.x, left_obj->position.y, 0.0};
  auto        obj_pos_new = TransformCoordinate(obj_pos, *junction_orin_vertical_);
  LD_TRAFFIC_LOG << fmt::format("left_obj_id:{} left_time:{} raw_pos:{:.2f},{:.2f}  new_pos:{:.2f},{:.2f}", left_obj->id, left_his_time,
                                left_obj->position.x, left_obj->position.y, obj_pos_new.x, obj_pos_new.y);
  if (obj_pos_new.x < 10 || left_his_time < 3) {
    return;
  }

  LD_TRAFFIC_LOG << "ego_lane_id:" << ego_ld_lane_id_;
  if (ego_ld_lane_id_ == 0) {
    return;
  }
  const auto &lanes       = routing_map->lanes;
  const auto &it_ego_lane = std::find_if(lanes.begin(), lanes.end(), [&](const LaneInfo &rhs) { return rhs.id == ego_ld_lane_id_; });
  if (it_ego_lane == lanes.end()) {
    LD_TRAFFIC_LOG << "not_find_ego_lane.";
    return;
  }
  // if (it_ego_lane->section_id != it_section->id) {
  //   LD_TRAFFIC_LOG << fmt::format("ego_lane_section_id:{}  it_section_id:{}", it_ego_lane->section_id, it_section->id);
  //   return;
  // }
  if (it_ego_lane->next_lane_ids.empty()) {
    return;
  }
  std::vector<uint64_t> next_lane_ids{it_ego_lane->id};

  std::vector<const LaneInfo *> left_wait_lane;
  std::vector<uint64_t>         id_tmp;
  bool                          find_junction = false;

  for (; it_section != routing_map->route.sections.end(); it_section++) {
    LD_TRAFFIC_LOG << fmt::format("section_id:{} next_lane_id:{}", it_section->id, next_lane_ids);
    auto it_lane = lanes.end();
    for (uint64_t id_t : it_section->lane_ids) {
      it_lane = std::find_if(lanes.begin(), lanes.end(), [id_t](const LaneInfo &rhs) { return rhs.id == id_t; });
      if (it_lane == lanes.end()) {
        LD_TRAFFIC_LOG << fmt::format("not_find_next_lane:{}", next_lane_ids);
        return;
      }
      if (it_lane->junction_id == junction_id) {
        find_junction = true;
      }
      if (std::find(next_lane_ids.begin(), next_lane_ids.end(), id_t) == next_lane_ids.end()) {
        continue;
      }
      break;
    }
    if (it_lane == lanes.end()) {
      LD_TRAFFIC_LOG << "not_find_next_lane.";
      continue;
    }

    if (!left_wait_lane.empty() && it_lane->junction_id == junction_id && it_lane->type != LaneType::LANE_LEFT_WAIT) {
      break;
    }
    if (it_lane->junction_id != junction_id && find_junction) {
      break;
    }
    if (it_lane->junction_id == junction_id && it_lane->type == LaneType::LANE_LEFT_WAIT) {
      id_tmp.emplace_back(it_lane->id);
      left_wait_lane.emplace_back(&(*it_lane));
      find_junction = true;
    }
    if (!it_lane->next_lane_ids.empty()) {
      next_lane_ids = it_lane->next_lane_ids;
    }
  }
  if (left_wait_lane.empty()) {
    return;
  }
  LD_TRAFFIC_LOG << fmt::format("left_wait_ids:{}", id_tmp);

  std::vector<Vec2d> points;
  for (auto &it : left_wait_lane) {
    for (const auto &point : it->points) {
      points.emplace_back(point.x, point.y);
    }
  }
  MultiLineSegment line_seg;
  line_seg.Init(points);
  double delta_dis    = 1.0;
  double dis_stopline = 0.0;
  for (; dis_stopline < line_seg.Length(); dis_stopline += delta_dis) {
    auto            val = line_seg.GetPointAtDistance(dis_stopline);
    TransformParams trans_orin;
    trans_orin.tx    = val.first.x();
    trans_orin.ty    = val.first.y();
    trans_orin.theta = val.second;
    Point3DD pos_new = TransformCoordinate(obj_pos, trans_orin);
    double   angle   = atan2(pos_new.y, pos_new.x) * RAD_TO_DEG;
    LD_TRAFFIC_LOG << fmt::format("dis_stopline:{:.2f} orin:[{:.2f},{:.2f},{:.2f}] pos_new:[{:.2f},{:.2f}] angle:{:.2f}", dis_stopline,
                                  trans_orin.tx, trans_orin.ty, trans_orin.theta, pos_new.x, pos_new.y, angle);
    if (std::fabs(angle) > 15) {
      break;
    }
  }
  double dis_curr = line_seg.Length() - dis_stopline;
  double res      = 0.0;
  if (left_distance_stopline_map_.count(junction_id) == 0) {
    res = dis_curr;
  } else {
    res = std::max(left_distance_stopline_map_[junction_id], dis_curr);
  }
  left_distance_stopline_map_[junction_id] = res;

  auto &lanes_output = routing_map_output->lanes;

  std::vector<uint64_t> ids_t_01;
  std::set<uint64_t>    next_section_ids;
  for (const auto &lane_t : left_wait_lane) {
    auto it_sec = std::find_if(routing_map->route.sections.begin(), routing_map->route.sections.end(),
                               [&lane_t](const auto &sec_t) { return lane_t->section_id == sec_t.id; });
    if (it_sec == routing_map->route.sections.end()) {
      continue;
    }
    ++it_sec;
    if (it_sec == routing_map->route.sections.end()) {
      continue;
    }
    next_section_ids.insert(it_sec->id);
  }
  for (const auto &sec_id_t : next_section_ids) {
    std::for_each(lanes_output.begin(), lanes_output.end(), [&](LaneInfo &rhs) {
      if (rhs.section_id == sec_id_t) {
        ids_t_01.emplace_back(rhs.id);
        rhs.distance_to_stopline = res;
      }
    });
  }
  LD_TRAFFIC_LOG << fmt::format("distance_to_stopline:{:.2f}  res:{:.2f} ids:{} sec_ids:{}", dis_curr, res, ids_t_01, next_section_ids);
}

bool TrafficLightMapping::IsSectionEnteringRoundabout(uint64_t section_id) {
  bool res_flag{false};
  if (routing_map_ptr_ == nullptr) {
    return res_flag;
  }
  const auto &sections = routing_map_ptr_->route.sections;
  auto it_sec = std::find_if(sections.begin(), sections.end(), [section_id](const SectionInfo &rhs) { return rhs.id == section_id; });
  if (it_sec == sections.end() || sections.empty()) {
    return res_flag;
  }

  int sec_idx = static_cast<int>(std::distance(sections.begin(), it_sec));
  if (IsLinkType(it_sec->link_type, byd::msg::orin::routing_map::SectionInfo::LT_ROUNDABOUT)) {
    return false;
  }
  if (sec_idx == 0 || sec_idx + 1 == static_cast<int>(sections.size())) {
    return false;
  }
  bool prev_is_roundabout = IsLinkType(sections.at(sec_idx - 1).link_type, byd::msg::orin::routing_map::SectionInfo::LT_ROUNDABOUT);
  bool next_is_roundabout = IsLinkType(sections.at(sec_idx + 1).link_type, byd::msg::orin::routing_map::SectionInfo::LT_ROUNDABOUT);
  if (!prev_is_roundabout && next_is_roundabout) {
    return true;
  }

  return res_flag;
}

void TrafficLightMapping::SetTrafficLight(const RoutingMapPtr &routing_map, const RoutingMapPtr &routing_map_output) {
  if (routing_map == nullptr || routing_map_output == nullptr) {
    TRAFFIC_LOG << fmt::format("routing_map is nullptr.routing_map:{:p}  routing_map_out:{:p}", fmt::ptr(routing_map),
                               fmt::ptr(routing_map_output));
    return;
  }

  double distance_to_junction = std::numeric_limits<double>::infinity();
  previous_junction_points_.clear();
  distance_to_junction_                          = std::numeric_limits<double>::infinity();
  double distance_to_section_over_first_junction = std::numeric_limits<double>::infinity();
  distance_to_section_over_first_junction_       = std::numeric_limits<double>::infinity();
  first_section_over_juntion_                    = nullptr;

  TRAFFIC_LOG << "section_id:" << routing_map->route.navi_start.section_id << "  offset:" << routing_map->route.navi_start.s_offset;

  auto it_section = std::find_if(routing_map->route.sections.begin(), routing_map->route.sections.end(),
                                 [&routing_map](const auto &section) { return routing_map->route.navi_start.section_id == section.id; });
  if (it_section == routing_map->route.sections.end()) {
    TRAFFIC_LOG << "not find ego_section.";
    return;
  }

  auto       &lanes        = routing_map->lanes;
  auto       &lanes_output = routing_map_output->lanes;
  const auto &stop_lines   = routing_map_output->stop_lines;

  bool section_is_junction_prev    = false;
  bool section_is_junction_current = false;
  bool stop_count_dis              = false;

  std::vector<LaneInfo> lanes_traffic_all;
  TRAFFIC_LOG << fmt::format(
      "section_id:1  lane_id:2  is_virtual_in_junction:3  lane_has_stop_line:5  dis_to_traffic_light:6 is_Uturn_valid:7  "
      "is_in_small_junction:10   turn_type:11  is_no_virtual_junction_lane:14");
  GetJunctionIdsPool(routing_map);

  // reset traffic_lights.
  MatchLightAndJunction(routing_map);
  FindBestLight(routing_map);
  if ((traffic_lights_prev_ld_.straight.is_valid && !traffic_lights_ld_.straight.is_valid) ||
      (traffic_lights_prev_ld_.left.is_valid && !traffic_lights_ld_.left.is_valid) ||
      (traffic_lights_prev_ld_.u_turn.is_valid && !traffic_lights_ld_.u_turn.is_valid) ||
      (traffic_lights_prev_ld_.right.is_valid && !traffic_lights_ld_.right.is_valid)) {
    TRAFFIC_LOG << fmt::format("stay_prev_state.");
    StayPreviousState(traffic_lights_prev_ld_.straight, traffic_lights_ld_.straight);
    StayPreviousState(traffic_lights_prev_ld_.left, traffic_lights_ld_.left);
    StayPreviousState(traffic_lights_prev_ld_.u_turn, traffic_lights_ld_.u_turn);
    StayPreviousState(traffic_lights_prev_ld_.right, traffic_lights_ld_.right);
  }
  info_traffic_lights_ += "  use_LD_traffic_light.";
  IsLightBlockFailed(traffic_lights_prev_ld_.left, traffic_lights_ld_.left);
  IsLightBlockFailed(traffic_lights_prev_ld_.u_turn, traffic_lights_ld_.u_turn);
  IsLightBlockFailed(traffic_lights_prev_ld_.right, traffic_lights_ld_.right);
  IsLightBlockFailed(traffic_lights_prev_ld_.straight, traffic_lights_ld_.straight);
  uint32_t cyber_counter = perception_traffic_light_info_ ? perception_traffic_light_info_->header.cycle_counter : 0;
  LD_TRAFFIC_LOG << fmt::format("vis_seq:{} S {}", cyber_counter, traffic_lights_ld_.straight);
  LD_TRAFFIC_LOG << fmt::format("vis_seq:{} L {}", cyber_counter, traffic_lights_ld_.left);
  LD_TRAFFIC_LOG << fmt::format("vis_seq:{} U {}", cyber_counter, traffic_lights_ld_.u_turn);
  LD_TRAFFIC_LOG << fmt::format("vis_seq:{} R {}", cyber_counter, traffic_lights_ld_.right);
  if (!cloud_traffic_light_) {
    traffic_lights_ = traffic_lights_ld_;
  } else {
    LD_TRAFFIC_LOG << fmt::format("vis_counter:{} cloud_traffic_straight {}", cyber_counter, traffic_lights_.straight);
    LD_TRAFFIC_LOG << fmt::format("vis_counter:{} cloud_traffic_left     {}", cyber_counter, traffic_lights_.left);
    LD_TRAFFIC_LOG << fmt::format("vis_counter:{} cloud_traffic_uturn    {}", cyber_counter, traffic_lights_.u_turn);
    LD_TRAFFIC_LOG << fmt::format("vis_counter:{} cloud_traffic_right    {}", cyber_counter, traffic_lights_.right);
  }
  traffic_lights_prev_ld_ = traffic_lights_ld_;
  UpdateHistoryObjId();
  SetAllLightStatus();

  auto     find_match  = MatchTrafficLightAndJunctionStopline(routing_map);
  bool     is_junction = false;
  uint64_t target_id   = 0;

  bool is_junction_match{false};
  if (find_match) {
    double dis_route_temp    = 0.0;
    double match_section_pos = 0.0;
    double match_section_len = 0.0;
    double ego_section_pos   = 0.0;
    bool   is_find_ego       = false;

    is_junction = std::get<2>(*find_match);
    target_id   = std::get<0>(*find_match);
    TRAFFIC_LOG << fmt::format("match_{}_id:{}", is_junction ? "junction" : "section", target_id);

    for (auto &section : routing_map->route.sections) {
      double dis_raw = dis_route_temp;
      dis_route_temp += section.length;
      TRAFFIC_LOG << fmt::format("section_id:{} pos_before:{:.2f}  pos:{:.2f} len:{:.2f}", section.id, dis_raw, dis_route_temp,
                                 section.length);
      for (auto lane_id : section.lane_ids) {
        auto it_lane = std::find_if(lanes.begin(), lanes.end(), [&lane_id](const auto &lane_t) { return lane_t.id == lane_id; });
        if (it_lane == lanes.end() || it_lane->type == LaneType::LANE_NON_MOTOR) {
          continue;
        }
        bool stopline_valid    = LaneStoplineTrafficLight(section.id, *it_lane, -50);
        bool is_junction_valid = is_junction && target_id == it_lane->junction_id;
        bool is_section_valid  = !is_junction && target_id == section.id && stopline_valid;
        if (!is_junction_valid && !is_section_valid) {
          // LD_TRAFFIC_LOG << fmt::format("is_junction:{:d}  target_id:{} stopline_valid:{:d}", is_junction, target_id, stopline_valid);
          continue;
        }
        double dis_ego_junction = is_find_ego ? dis_route_temp - ego_section_pos - section.length : std::numeric_limits<double>::infinity();
        LD_TRAFFIC_LOG << "lane_id:" << it_lane->id
                       << fmt::format(" junction valid:{} id:{}  dis:{:.2f}  lane_type:{}", is_junction_valid, it_lane->junction_id,
                                      dis_ego_junction, it_lane->type);
        if (auto lane_out = SetLaneTrafficInfo(*it_lane, lanes_output, stop_lines); lane_out.has_value()) {
          match_section_pos = dis_route_temp;
          match_section_len = section.length;
          is_junction_match = true;
          lanes_traffic_all.emplace_back(*lane_out);
        } else if (is_junction_valid && it_lane->junction_id != 0 && dis_ego_junction > -20 && dis_ego_junction < 80 &&
                   (it_lane->type == LaneType::LANE_VIRTUAL_JUNCTION || it_lane->type == LaneType::LANE_VIRTUAL_COMMON)) {
          auto it_t =
              std::find_if(lanes_output.begin(), lanes_output.end(), [&it_lane](const LaneInfo &rhs) { return rhs.id == it_lane->id; });
          if (it_t != lanes_output.end()) {
            if (traffic_lights_.left.traffic_obj_info && it_lane->turn_type == TurnType::NO_TURN) {
              it_t->light_info.maybe_exist_light = true;
            } else if (traffic_lights_.straight.traffic_obj_info && it_lane->turn_type == TurnType::LEFT_TURN) {
              it_t->light_info.maybe_exist_light = true;
            } else if (traffic_lights_.straight.traffic_obj_info && it_lane->turn_type == TurnType::U_TURN) {
              it_t->light_info.maybe_exist_light = true;
            }
            if (it_t->light_info.maybe_exist_light) {
              LD_TRAFFIC_LOG << fmt::format("lane_id:{}  maybe_exist_light=1.", it_t->id);
            }
          }
        }
      }
      if (section.id == it_section->id) {
        ego_section_pos = dis_route_temp + routing_map->route.navi_start.s_offset - section.length;
        is_find_ego     = true;
      }
      if (is_find_ego) {
        LD_TRAFFIC_LOG << fmt::format("is_find_ego:{:d}  dis:{:.2f},{:.2f},{:.2f}", is_find_ego, dis_route_temp, ego_section_pos,
                                      dis_route_temp - ego_section_pos);
      }
      // if (is_find_ego && dis_route_temp - ego_section_pos > 1000) {
      //   break;
      // }
      if (dis_route_temp > 1000) {
        break;
      }
    }
    match_section_pos -= match_section_len;
    dis_ego_light_junction_ = match_section_pos - ego_section_pos;
    TRAFFIC_LOG << fmt::format("match_section_pos:{:.2f}  ego_section_pos:{:.2f} dis_ego_light_junction_:{:.2f} ", match_section_pos,
                               ego_section_pos, dis_ego_light_junction_);
  }

  bool is_virtual_lane_light{false};
  for (; it_section != routing_map->route.sections.end(); it_section++) {
    section_is_junction_current  = false;
    bool is_section_has_junction = false;
    for (auto lane_id : it_section->lane_ids) {
      auto it_lane = std::find_if(lanes.begin(), lanes.end(), [&lane_id](const auto &lane_t) { return lane_t.id == lane_id; });
      if (it_lane == lanes.end() || it_lane->type == LaneType::LANE_NON_MOTOR) {
        continue;
      }
      is_section_has_junction              = it_lane->junction_id != 0;
      double min_distance_to_traffic_light = LaneNearDisToJunction(*it_lane);

      bool is_light_block_failed       = traffic_lights_.GetLightState(it_lane->turn_type).color == message::sensor::TLC_BLOCK_FAILED;
      bool lane_is_virtual_in_junction = it_lane->type == message::env_model::LaneType::LANE_VIRTUAL_JUNCTION &&
                                         it_lane->junction_id != 0 &&
                                         (min_distance_to_traffic_light < lane_virtual_dis_threshold || is_light_block_failed);
      bool is_in_small_junction = LaneCrossHasTrafficLight(*it_lane);
      bool lane_has_stop_line   = LaneStoplineTrafficLight(it_section->id, *it_lane);

      bool is_no_virtual_junction_lane{false};
      if (ldmap_junction_ids_.count(it_lane->junction_id) > 0) {
        double dis = DistanceToJunction(ldmap_junction_ids_[it_lane->junction_id], traffic_lights_.GetLightState(it_lane->turn_type));
        TRAFFIC_LOG << fmt::format("junction_id:{}  dis:{:.2f}  turn:{}", it_lane->junction_id, dis,
                                   magic_enum::enum_name(it_lane->turn_type));
        if (dis < dis_novirtual_light) {
          is_no_virtual_junction_lane = true;
        }
      }

      bool is_uturn_valid =
          (it_lane->turn_type == TurnType::U_TURN) &&
          ((it_lane->junction_id == 0 && min_distance_to_traffic_light < lane_uturn_dis_threshold) ||
           (it_lane->junction_id != 0 && (min_distance_to_traffic_light < lane_virtual_dis_threshold || is_light_block_failed)));

      if (ldmap_junction_ids_.count(it_lane->junction_id) > 0 || prev_find_small_sections_.count(it_section->id) > 0 ||
          it_lane->junction_id) {
        TRAFFIC_LOG << "lane_id:" << it_lane->id << " section_id:" << it_lane->section_id << " has_junction:" << it_lane->junction_id;
        section_is_junction_current = true;
      }
      TRAFFIC_LOG << fmt::format("1:{}  2:{}  3:{:d}  5:{:d}  6:{{{:.2f}}} 8:{:d} 10:{:d} 11:{} 14:{:d}", it_section->id, it_lane->id,
                                 lane_is_virtual_in_junction, lane_has_stop_line, min_distance_to_traffic_light, is_uturn_valid,
                                 is_in_small_junction, it_lane->turn_type, is_no_virtual_junction_lane);

      if (!lane_is_virtual_in_junction && !is_in_small_junction && !is_no_virtual_junction_lane && !lane_has_stop_line &&
          it_lane->turn_type != TurnType::RIGHT_TURN && it_lane->turn_type != TurnType::LEFT_TURN &&
          it_lane->turn_type != TurnType::U_TURN) {
        continue;
      }
      if (lane_is_virtual_in_junction) {
        is_virtual_lane_light = true;
      }
      // auto [it, inserted] = prev_find_small_sections_.insert(it_section->id);
      // if (inserted) {
      // TRAFFIC_LOG << fmt::format("section_id:{}  is_stored_in_set.", it_section->id);
      // }
      section_is_junction_current = true;

      if (it_lane->turn_type == TurnType::U_TURN && !is_uturn_valid && !is_no_virtual_junction_lane) {
        continue;
      }
      if (is_junction_match) {
        TRAFFIC_LOG << "lane_light_has_been_set.";
        continue;
      }
      if (auto lane_out = SetLaneTrafficInfo(*it_lane, lanes_output, stop_lines); lane_out.has_value()) {
        lanes_traffic_all.emplace_back(*lane_out);
      }
    }
    // when the first junction is found.
    if (std::isinf(distance_to_junction)) {
      distance_to_junction = 0.0;
    }
    double dis_t = distance_to_junction;
    if (section_is_junction_current && !section_is_junction_prev) {
      stop_count_dis       = true;
      is_section_junction_ = is_section_has_junction;
      distance_to_junction -= routing_map->route.navi_start.s_offset;
      distance_to_section_over_first_junction = distance_to_junction;
      TRAFFIC_LOG << "find_junction section:" << it_section->id << "  length:" << it_section->length
                  << "  s_offset:" << routing_map->route.navi_start.s_offset << "  distance_to_junction:" << distance_to_junction
                  << "  raw_dis:" << dis_t;
    } else if (!stop_count_dis) {
      distance_to_junction += it_section->length;
      TRAFFIC_LOG << "no_find_junction section:" << it_section->id << "  length:" << it_section->length
                  << "  distance_to_junction:" << distance_to_junction << "  raw_dis:" << dis_t;
    }
    if (!std::isinf(distance_to_section_over_first_junction)) {
      distance_to_section_over_first_junction += it_section->length;
    }
    // check the first junction has been expired .
    if (!section_is_junction_current && section_is_junction_prev) {
      if (junction_ids_vec_.empty() || junction_ids_vec_.back() != it_section->id) {
        TRAFFIC_LOG << "junction_ids_vec_emplace_back_section_id:" << it_section->id;
        junction_ids_vec_.emplace_back(it_section->id);
      }
      if (first_section_over_juntion_ == nullptr) {
        first_section_over_juntion_ = &(*it_section);
      }
      break;
    }
    section_is_junction_prev = section_is_junction_current;
  }
  if (is_virtual_lane_light) {
    ProcessNotRoutingLanes(routing_map_output, lanes_traffic_all);
  }
  SetJunctionPoints(routing_map, lanes_traffic_all);
  if (is_junction) {
    PrevStopWhenLeftWait(routing_map, routing_map_output, target_id, distance_to_junction);
  }

  if (stop_count_dis) {
    if (distance_to_junction >= 0.0) {
      distance_to_junction_ = distance_to_junction;
      if (!is_junction_match) {
        dis_ego_light_junction_ = distance_to_junction;
      }
    } else {
      distance_to_junction_ = 0.0;
    }
    distance_to_section_over_first_junction_ = distance_to_section_over_first_junction;
  }
  TRAFFIC_LOG << fmt::format("is_match:{:d}  dis:{:.2f}", is_junction_match, dis_ego_light_junction_);
  TRAFFIC_LOG << fmt::format("routing_map_counter:{}  raw_distance_to_junction:{:.2f}   output_distance_to_junction:{:.2f}",
                             routing_map->header.cycle_counter, distance_to_junction, distance_to_junction_);
  // SetE2EInfo(routing_map_output, lanes_traffic_all);
}

bool TrafficLightMapping::IsValid(const TrafficLight &light) {
  return light.is_valid && light.traffic_obj_info;
};

void TrafficLightMapping::ValidateComparePrevLight(const TrafficLight                                &light_previous,
                                                   std::vector<std::vector<TrfObjectInfo>::iterator> &valid_objs) {
  if (!IsValid(light_previous)) {
    TRAFFIC_REC_LOG << "light_previous_is_invalid.";
    return;
  }

  constexpr double dis_max   = 20.0;
  const auto       DisPoints = [](cem::message::common::Point3DD lhs, cem::message::common::Point3DD rhs) {
    double delta_x = lhs.x - rhs.x;
    double delta_y = lhs.y - rhs.y;
    return std::sqrt(delta_x * delta_x + delta_y * delta_y);
  };

  auto DisTooFar = [DisPoints, dis_max](const TrafficLight &light_previous, const TrfObjectInfo &light_current_obj) {
    double dis_temp = DisPoints(light_current_obj.position, light_previous.traffic_obj_info->position);
    TRAFFIC_REC_LOG << fmt::format("prev_obj:{}  pos:{:.2f}-{:.2f}  cur_obj:{}  pos:{:.2f}-{:.2f} dis_temp:{:.2f}  dis_max:{:.2f}",
                                   light_previous.traffic_obj_info->id, light_previous.traffic_obj_info->position.x,
                                   light_previous.traffic_obj_info->position.y, light_current_obj.id, light_current_obj.position.x,
                                   light_current_obj.position.y, dis_temp, dis_max);
    return dis_temp > dis_max;
  };
  valid_objs.erase(std::remove_if(valid_objs.begin(), valid_objs.end(),
                                  [&light_previous, DisTooFar](const auto &it) { return DisTooFar(light_previous, *it); }),
                   valid_objs.end());
}

bool TrafficLightMapping::TransformPostionOfObjs(double timestamp) {
  if (!IsValid(traffic_lights_previous_.left) && !IsValid(traffic_lights_previous_.u_turn) && !IsValid(traffic_lights_previous_.straight) &&
      !IsValid(traffic_lights_previous_.right)) {
    return true;
  }
  const auto &T_prev_to_current = GetTransformMatrix(timestamp);
  if (!T_prev_to_current) {
    return false;
  }
  auto PosTrans = [&T_prev_to_current, this](TrafficLight &light) {
    if (!this->IsValid(light)) {
      return;
    }
    Eigen::Vector3d point_src(light.traffic_obj_info->position.x, light.traffic_obj_info->position.y, light.traffic_obj_info->position.z);
    auto            Pos                = *T_prev_to_current * point_src;
    light.traffic_obj_info->position.x = Pos.x();
    light.traffic_obj_info->position.y = Pos.y();
    light.traffic_obj_info->position.z = Pos.z();
  };
  PosTrans(traffic_lights_previous_.left);
  PosTrans(traffic_lights_previous_.u_turn);
  PosTrans(traffic_lights_previous_.straight);
  PosTrans(traffic_lights_previous_.right);
  return true;
}

std::optional<Eigen::Isometry3d> TrafficLightMapping::GetTransformMatrix(double timestamp) {
  LocalizationPtr target_pose = nullptr;
  SensorDataManager::Instance()->GetLatestSensorFrame(timestamp, 0.05, target_pose);
  LocalizationPtr measurement_pose = nullptr;
  SensorDataManager::Instance()->GetLatestSensorFrame(time_previous_, 0.05, measurement_pose);
  if (!(target_pose && measurement_pose)) {
    return std::nullopt;
  }

  return CalcRotateTranslateMatrix(measurement_pose, target_pose);
}

void TrafficLightMapping::SetSrTrafficLight(const RoutingMapPtr &routing_map_output) {
  if (routing_map_output == nullptr) {
    return;
  }
  routing_map_output->traffic_lights.clear();
  for (const auto &obj : perception_traffic_lights_) {
    if (obj.position.x < 10) {
      continue;
    }
    if (obj.attributes.traffic_light_color != TrafficLightColorType::TLC_GREEN &&
        obj.attributes.traffic_light_color != TrafficLightColorType::TLC_RED &&
        obj.attributes.traffic_light_color != TrafficLightColorType::TLC_YELLOW) {
      TransSrTrafficLight(obj, routing_map_output->traffic_lights.emplace_back(), 55, 19);
      continue;
    }
    TRAFFIC_LOG << fmt::format("sr_id:{}  color:{:d}", obj.id, obj.attributes.traffic_light_color);
    switch (obj.attributes.traffic_light_shape) {
      case TrafficLightShapeType::TLS_CIRCULAR:
      case TrafficLightShapeType::TLS_HEART_SHAPE:
      case TrafficLightShapeType::TLS_RECTANGLE:
        TransSrTrafficLight(obj, routing_map_output->traffic_lights.emplace_back(), 1, 1);
        break;
      case TrafficLightShapeType::TLS_UP_ARROW:
        TransSrTrafficLight(obj, routing_map_output->traffic_lights.emplace_back(), 2, 4);
        break;
      case TrafficLightShapeType::TLS_UP_LEFT_ARROW:
        TransSrTrafficLight(obj, routing_map_output->traffic_lights.emplace_back(), 3, 4);
        TransSrTrafficLight(obj, routing_map_output->traffic_lights.emplace_back(), 3, 2);
        break;
      case TrafficLightShapeType::TLS_UP_RIGHT_ARROW:
        TransSrTrafficLight(obj, routing_map_output->traffic_lights.emplace_back(), 4, 4);
        TransSrTrafficLight(obj, routing_map_output->traffic_lights.emplace_back(), 4, 3);
        break;
      case TrafficLightShapeType::TLS_LEFT_ARROW:
      case TrafficLightShapeType::TLS_TURN_ARROUND_ARROW:
      case TrafficLightShapeType::TLS_LEFT_TURN_ARROUND_ARROW:
        TransSrTrafficLight(obj, routing_map_output->traffic_lights.emplace_back(), 5, 2);
        break;
      case TrafficLightShapeType::TLS_RIGHT_ARROW:
        TransSrTrafficLight(obj, routing_map_output->traffic_lights.emplace_back(), 6, 3);
        break;
      default:
        TransSrTrafficLight(obj, routing_map_output->traffic_lights.emplace_back(), 7, 19);
        break;
    }
  }
}

void TrafficLightMapping::TransSrTrafficLight(const TrfObjectInfo &ori_light, cem::message::env_model::TrafficLightMap &out_light,
                                              uint64_t id, uint32_t shape) {
  out_light.id              = id;
  out_light.center_position = ori_light.position;
  out_light.light_countdown = ori_light.attributes.traffic_light_num;
  out_light.shape           = shape;
  out_light.light_status    = LightColorConvert(ori_light.attributes.traffic_light_color);
}

void TrafficLightMapping::SetLightsTurn(const std::vector<TrafficLightShapeType> &shapes, LightTurn turn) {
  for (auto &obj : perception_traffic_lights_) {
    auto it = std::find(shapes.begin(), shapes.end(), obj.attributes.traffic_light_shape);
    if (it != shapes.end()) {
      obj.turn.emplace_back(turn);
    }
  }
}

void TrafficLightMapping::SetTrafficLightColor() {
  for (auto &obj : perception_traffic_lights_) {
    if (obj.attributes.traffic_light_flashing) {
      switch (obj.attributes.traffic_light_color) {
        case TrafficLightColorType::TLC_RED:
          obj.attributes.traffic_light_color = TrafficLightColorType::TLC_RED;
          break;
        case TrafficLightColorType::TLC_GREEN:
          obj.attributes.traffic_light_color = TrafficLightColorType::TLC_GREEN_FLASHING;
          break;
        case TrafficLightColorType::TLC_YELLOW:
          obj.attributes.traffic_light_color = TrafficLightColorType::TLC_YELLOW_FLASHING;
          break;
        default:
          break;
      };
    }
  }
}

void TrafficLightMapping::FilterLightsByPosition() {
  if (perception_traffic_lights_.empty()) {
    return;
  }
  constexpr double dis_max_lateral   = 20.0;
  constexpr double dis_min_longitude = -10.0;
  constexpr double dis_max_longitude = 200.0;
  std::sort(perception_traffic_lights_.begin(), perception_traffic_lights_.end(), [](const auto &lhs, const auto &rhs) {
    if (std::fabs(lhs.position.x - rhs.position.x) < 1.0) {
      return lhs.position.y < rhs.position.y;
    }
    return lhs.position.x < rhs.position.x;
  });
  std::vector<std::size_t> need_remove_indexes;
  for (std::size_t idx = 0; idx != perception_traffic_lights_.size(); idx++) {
    const auto &obj = perception_traffic_lights_[idx];
    if (std::fabs(obj.position.y) > dis_max_lateral) {
      TRAFFIC_REC_LOG << fmt::format("obj_id:{}  deleted_y_reason:{:2f}", obj.id, obj.position.y);
      need_remove_indexes.push_back(idx);
    } else if (obj.position.x > dis_max_longitude) {
      TRAFFIC_REC_LOG << fmt::format("obj_id:{}  deleted_x_reason:{:2f}", obj.id, obj.position.x);
      need_remove_indexes.push_back(idx);
    } else if (obj.position.x < dis_min_longitude) {
      TRAFFIC_REC_LOG << fmt::format("obj_id:{}  deleted_x_reason_less_zero:{:2f}", obj.id, obj.position.x);
      need_remove_indexes.push_back(idx);
    }
  }
  for (auto it = need_remove_indexes.rbegin(); it != need_remove_indexes.rend(); ++it) {
    perception_traffic_lights_.erase(perception_traffic_lights_.begin() + static_cast<int64_t>(*it));
  }

  if (perception_traffic_lights_.size() < 2) {
    return;
  }

  constexpr double max_x_gap  = 80.0;
  double           base_x_dis = perception_traffic_lights_.begin()->position.x;
  perception_traffic_lights_.erase(
      std::remove_if(perception_traffic_lights_.begin(), perception_traffic_lights_.end(),
                     [&](const TrfObjectInfo &traffic_light) { return traffic_light.position.x - base_x_dis > max_x_gap; }),
      perception_traffic_lights_.end());
}

std::vector<std::vector<TrfObjectInfo>::iterator> TrafficLightMapping::SatisfyTrafficLight(const std::vector<TrafficLightShapeType> &shapes,
                                                                                           const std::vector<TrafficLightColorType> &colors,
                                                                                           LightTurn                                 turn,
                                                                                           const TrafficLight &previous_light) {
  std::vector<std::vector<TrfObjectInfo>::iterator> res;
  for (TrafficLightShapeType type_val : shapes) {
    for (TrafficLightColorType color_val : colors) {
      for (auto it_light = perception_traffic_lights_.begin(); it_light != perception_traffic_lights_.end(); ++it_light) {
        const auto &obj = *it_light;
        auto        it  = std::find(obj.turn.begin(), obj.turn.end(), turn);
        if ((it != obj.turn.end() || obj.turn.empty()) && obj.attributes.traffic_light_shape == type_val &&
            obj.attributes.traffic_light_color == color_val) {
          TRAFFIC_REC_LOG << fmt::format("match_obj_find  turn:{:<12} obj.id:{}", magic_enum::enum_name(turn), obj.id);
          res.emplace_back(it_light);
        }
      }
    }
  }

  if (!res.empty() && shapes.front() != TrafficLightShapeType::TLS_UNKNOWN && shapes.front() != TrafficLightShapeType::TLS_OTHER_SHAPE) {
    ValidateComparePrevLight(previous_light, res);
  }

  return res;
}

bool TrafficLightMapping::TrafficLightPositionTrack(std::vector<std::vector<TrfObjectInfo>::iterator> &objs,
                                                    const TrafficLight                                &previous_light) {
  if (objs.empty()) {
    return false;
  };
  if (!previous_light.is_valid || !previous_light.traffic_obj_info) {
    return true;
  }
  constexpr double max_dis_between_prev = 25.0;
  objs.erase(std::remove_if(objs.begin(), objs.end(),
                            [&](const std::vector<TrfObjectInfo>::iterator &it) {
                              double delta_x  = it->position.x - previous_light.traffic_obj_info->position.x;
                              double delta_y  = it->position.y - previous_light.traffic_obj_info->position.y;
                              double dis_prev = std::sqrt(delta_x * delta_x + delta_y * delta_y);
                              if (dis_prev > max_dis_between_prev) {
                                GEOMETRY_LOG << fmt::format("obj_id:{}  dis_prev:{:.2f}  ", it->id, dis_prev);
                                return true;
                              }
                              return false;
                            }),
             objs.end());

  return true;
}

TrfObjectInfo TrafficLightMapping::FindOptObj(const TrafficLight                                &previous_light,
                                              std::vector<std::vector<TrfObjectInfo>::iterator> &valid_objs) {
  if (valid_objs.empty()) {
    return {};
  }
  if (previous_light.is_valid && previous_light.traffic_obj_info) {
    const auto              &prev_pos = previous_light.traffic_obj_info->position;
    std::vector<std::size_t> idx_vec;
    constexpr double         gap_dis = 10.0;
    double                   min_dis = std::numeric_limits<double>::infinity();
    size_t                   min_idx = 0;
    for (std::size_t idx = 0; idx < valid_objs.size(); idx++) {
      const auto &obj     = *(valid_objs[idx]);
      double      delta_x = obj.position.x - prev_pos.x;
      double      delta_y = obj.position.y - prev_pos.y;
      double      dis_val = std::sqrt(delta_x * delta_x + delta_y * delta_y);
      if (dis_val < gap_dis) {
        idx_vec.emplace_back(idx);
      }
      if (min_dis > dis_val) {
        min_dis = dis_val;
        min_idx = idx;
      }
    }
    if (idx_vec.size() == 1) {
      TRAFFIC_REC_LOG << fmt::format("find_min_distance_id:{}", valid_objs[idx_vec.front()]->id);
      return *(valid_objs[idx_vec.front()]);
    }
    if (idx_vec.size() >= 2 && !std::isinf(min_dis)) {
      TRAFFIC_REC_LOG << fmt::format("find_>2_min_distance_id:{}", valid_objs[min_idx]->id);
      return *(valid_objs[min_idx]);
    }
  }
  if (valid_objs.size() >= 2) {
    auto sort_vec = valid_objs;
    std::sort(sort_vec.begin(), sort_vec.end(),
              [](const auto &lhs, const auto &rhs) { return std::fabs(lhs->position.y) < std::fabs(rhs->position.y); });
    TRAFFIC_REC_LOG << fmt::format("id:{}-{}  pos:{:.2f}-{:.2f}  pos:{:.2f}-{:.2f}", sort_vec[0]->id, sort_vec[1]->id,
                                   sort_vec[0]->position.x, sort_vec[0]->position.y, sort_vec[1]->position.x, sort_vec[1]->position.y);
    if (std::fabs(sort_vec[1]->position.y - sort_vec[0]->position.y) > 3.0) {
      return *sort_vec[0];
    }
    if (std::fabs(sort_vec[0]->position.x - sort_vec[1]->position.x) > 30.0) {
      return sort_vec[0]->position.x > sort_vec[1]->position.x ? *sort_vec[1] : *sort_vec[0];
    }
  }
  TRAFFIC_REC_LOG << fmt::format("default_id:{}", valid_objs.front()->id);
  return *(valid_objs.front());
};

bool TrafficLightMapping::IsSameLight(const TrfObjectInfo &lhs, const TrfObjectInfo &rhs) {
  switch (lhs.attributes.traffic_light_color) {
    case TrafficLightColorType::TLC_RED_FLASHING: {
      return rhs.attributes.traffic_light_color == TrafficLightColorType::TLC_RED_FLASHING;
    }
    case TrafficLightColorType::TLC_GREEN_FLASHING: {
      return rhs.attributes.traffic_light_color == TrafficLightColorType::TLC_GREEN_FLASHING;
    }
    case TrafficLightColorType::TLC_YELLOW_FLASHING: {
      return rhs.attributes.traffic_light_color == TrafficLightColorType::TLC_YELLOW_FLASHING;
    }
    default:
      return false;
  }
}

cem::message::sensor::TrafficLightColorType TrafficLightMapping::IsFlashLight(const TrfObjectInfo &lhs) {
  if (!lhs.attributes.traffic_light_flashing) {
    return lhs.attributes.traffic_light_color;
  }
  bool                  is_light_flash = true;
  std::vector<uint64_t> set_ids;
  for (const auto &obj : perception_traffic_lights_) {
    if (obj.id == lhs.id) {
      continue;
    }
    Vec2d  lhs_pos{lhs.position.x, lhs.position.y};
    Vec2d  rhs_pos{obj.position.x, obj.position.y};
    double dis_delta = lhs_pos.DistanceTo(rhs_pos);
    if (dis_delta > 8.0) {
      continue;
    }
    if (lhs.attributes.traffic_light_direction != obj.attributes.traffic_light_direction) {
      continue;
    }
    if (lhs.attributes.traffic_light_shape != obj.attributes.traffic_light_shape) {
      continue;
    }
    if (!IsSameLight(lhs, obj)) {
      set_ids.emplace_back(obj.id);
      is_light_flash = false;
    }
  }
  if (is_light_flash) {
    return lhs.attributes.traffic_light_color;
  }
  LD_TRAFFIC_LOG << fmt::format("obj_id:{}  not_flash_ids:{}  orin_color:{}  remove_flash.", lhs.id, set_ids,
                                magic_enum::enum_name(lhs.attributes.traffic_light_color));

  switch (lhs.attributes.traffic_light_color) {
    case TrafficLightColorType::TLC_RED_FLASHING: {
      return TrafficLightColorType::TLC_RED;
    }
    case TrafficLightColorType::TLC_GREEN_FLASHING: {
      return TrafficLightColorType::TLC_GREEN;
    }
    case TrafficLightColorType::TLC_YELLOW_FLASHING: {
      return TrafficLightColorType::TLC_YELLOW;
    }
    default:
      return lhs.attributes.traffic_light_color;
  }
}

std::vector<TrafficLightShapeType> TrafficLightMapping::GetTurnType(TurnType turn_type) {
  switch (turn_type) {
    case TurnType::LEFT_TURN:
      return left_shapes_;
    case TurnType::U_TURN:
      return uturn_shapes_;
    case TurnType::NO_TURN:
      return straight_shapes_;
    case TurnType::RIGHT_TURN:
      return right_shapes_;
    default:
      return {};
  }
  return {};
};

void TrafficLightMapping::SetLightT(const TrfObjectInfo &obj, TrafficLight &traffic_light_t) {
  std::vector<TrfObjectInfo> other_objs;

  auto turn_types = GetTurnType(traffic_light_t.turn_type);
  if (traffic_light_t.turn_type != TurnType::RIGHT_TURN) {
    turn_types.emplace_back(TLS_CIRCULAR);
  }
  std::vector<uint64_t> vec_ids;

  other_objs.emplace_back(obj);
  vec_ids.emplace_back(obj.id);
  for (const auto &obj_tmp : perception_traffic_lights_) {
    if (obj.id == obj_tmp.id) {
      continue;
    }
    if (obj_tmp.traffic_light_box_id != obj.traffic_light_box_id) {
      continue;
    }
    if (obj_tmp.attributes.traffic_light_direction != TrafficLightDirectionType::TLD_BACK) {
      continue;
    }
    if (std::find(turn_types.begin(), turn_types.end(), obj_tmp.attributes.traffic_light_shape) == turn_types.end()) {
      continue;
    }
    vec_ids.emplace_back(obj_tmp.id);
    other_objs.emplace_back(obj_tmp);
  }
  if (other_objs.size() > 1 && obj.track_status == TrackStatus::COASTED) {
    std::sort(other_objs.begin(), other_objs.end(), [](const TrfObjectInfo &lhs, const TrfObjectInfo &rhs) {
      auto GetPriority = [](TrackStatus status_t) {
        switch (status_t) {
          case TrackStatus::UPDATED:
            return 0;
          case TrackStatus::NEW:
            return 1;
          case TrackStatus::COASTED:
            return 2;
          case TrackStatus::INVALID:
            return 3;
          default:
            return 4;
        }
      };
      return GetPriority(lhs.track_status) < GetPriority(rhs.track_status);
    });
    LD_TRAFFIC_LOG << fmt::format("vec_ids:{}  front_obj:{} obj:{}", vec_ids, other_objs.front().id, obj.id);
  }
  auto obj_rev = other_objs.front();

  LD_TRAFFIC_LOG << "set_final_obj_id:" << obj_rev.id;
  traffic_light_t.is_valid                   = true;
  traffic_light_t.traffic_light_flashing     = obj_rev.attributes.traffic_light_flashing;
  traffic_light_t.traffic_light_num          = obj_rev.attributes.traffic_light_num;
  traffic_light_t.color                      = obj_rev.attributes.traffic_light_color;  // IsFlashLight(obj_rev)
  traffic_light_t.traffic_obj_info           = std::make_shared<TrfObjectInfo>(obj_rev);
  traffic_light_t.yellow_flashing_start_time = obj_rev.yellow_flashing_start_time;

  if (perception_traffic_light_info_) {
    traffic_light_t.perception_seq_num = perception_traffic_light_info_->header.cycle_counter;
  }
};

void TrafficLightMapping::SetTrafficLights(LightTurn turn, const std::vector<TrafficLightShapeType> &shapes,
                                           const TrafficLight &previous_light, TrafficLight &traffic_light) {
  if (perception_traffic_lights_.empty()) {
    return;
  }

  traffic_light.is_valid           = true;
  traffic_light.perception_seq_num = perception_traffic_light_info_->header.cycle_counter;
  auto shapes_add                  = shapes;
  if (turn != LightTurn::RIGHT) {
    shapes_add.emplace_back(TrafficLightShapeType::TLS_CIRCULAR);
  }
  auto shapes_all = shapes_add;
  shapes_all.emplace_back(TrafficLightShapeType::TLS_UNKNOWN);
  shapes_all.emplace_back(TrafficLightShapeType::TLS_OTHER_SHAPE);

  std::size_t unknown_shape_obj_num =
      std::count_if(perception_traffic_lights_.begin(), perception_traffic_lights_.end(), [](const auto &light) {
        return light.attributes.traffic_light_shape == TrafficLightShapeType::TLS_UNKNOWN ||
               light.attributes.traffic_light_shape == TrafficLightShapeType::TLS_OTHER_SHAPE;
      });
  bool is_exist_flash = std::find_if(perception_traffic_lights_.begin(), perception_traffic_lights_.end(), [](const TrfObjectInfo &light) {
                          return light.attributes.traffic_light_flashing;
                        }) != perception_traffic_lights_.end();

  auto valid_objs = SatisfyTrafficLight(
      shapes,
      {TrafficLightColorType::TLC_GREEN, TrafficLightColorType::TLC_RED, TrafficLightColorType::TLC_YELLOW,
       TrafficLightColorType::TLC_GREEN_FLASHING, TrafficLightColorType::TLC_YELLOW_FLASHING, TrafficLightColorType::TLC_RED_FLASHING},
      turn, previous_light);

  if (!valid_objs.empty()) {
    SetLightT(FindOptObj(previous_light, valid_objs), traffic_light);
    traffic_light.traffic_reason = byd::msg::orin::routing_map::LaneInfo::NEW_ARROW_LIGHT;
  } else if (valid_objs = SatisfyTrafficLight({TrafficLightShapeType::TLS_CIRCULAR},
                                              {TrafficLightColorType::TLC_GREEN, TrafficLightColorType::TLC_RED,
                                               TrafficLightColorType::TLC_YELLOW, TrafficLightColorType::TLC_GREEN_FLASHING,
                                               TrafficLightColorType::TLC_YELLOW_FLASHING, TrafficLightColorType::TLC_RED_FLASHING},
                                              turn, previous_light);
             turn != LightTurn::RIGHT && !valid_objs.empty()) {
    SetLightT(FindOptObj(previous_light, valid_objs), traffic_light);
    traffic_light.traffic_reason = byd::msg::orin::routing_map::LaneInfo::NEW_CIRCLE_LIGHT;
  } else if (turn == LightTurn::RIGHT) {
    if (previous_light.is_valid && previous_light.traffic_reason != byd::msg::orin::routing_map::LaneInfo::SET_DEFAULT_OBJ &&
        traffic_light.perception_seq_num - previous_light.perception_seq_num <= max_stay_frames_with_info) {
      TRAFFIC_REC_LOG << "stay_prev_info turn:" << magic_enum::enum_name(turn);
      traffic_light                = previous_light;
      traffic_light.traffic_reason = byd::msg::orin::routing_map::LaneInfo::STAY_PREV;
      traffic_light.stay_prev_counter++;
    } else {
      traffic_light.traffic_light_flashing = false;
      traffic_light.traffic_light_num      = 1000;
      traffic_light.color                  = TLC_GREEN;
      traffic_light.traffic_reason         = byd::msg::orin::routing_map::LaneInfo::SET_DEFAULT_OBJ;
    }
  } else if (valid_objs =
                 SatisfyTrafficLight({TrafficLightShapeType::TLS_UNKNOWN, TrafficLightShapeType::TLS_OTHER_SHAPE},
                                     {TrafficLightColorType::TLC_GREEN, TrafficLightColorType::TLC_GREEN_FLASHING}, turn, previous_light);
             !valid_objs.empty() && valid_objs.size() == unknown_shape_obj_num) {
    SetLightT(FindOptObj(previous_light, valid_objs), traffic_light);
    traffic_light.color                  = is_exist_flash ? message::sensor::TLC_GREEN_FLASHING : message::sensor::TLC_GREEN;
    traffic_light.traffic_light_flashing = is_exist_flash;
    traffic_light.traffic_reason         = byd::msg::orin::routing_map::LaneInfo::NEW_UNKNOWN_GREEN;
  } else if (valid_objs =
                 SatisfyTrafficLight({TrafficLightShapeType::TLS_UNKNOWN, TrafficLightShapeType::TLS_OTHER_SHAPE},
                                     {TrafficLightColorType::TLC_RED, TrafficLightColorType::TLC_RED_FLASHING}, turn, previous_light);
             !valid_objs.empty() && valid_objs.size() == unknown_shape_obj_num) {
    SetLightT(FindOptObj(previous_light, valid_objs), traffic_light);
    traffic_light.color                  = is_exist_flash ? message::sensor::TLC_RED_FLASHING : message::sensor::TLC_RED;
    traffic_light.traffic_light_flashing = is_exist_flash;
    traffic_light.traffic_reason         = byd::msg::orin::routing_map::LaneInfo::NEW_UNKNOWN_GREEN;
  } else if (valid_objs =
                 SatisfyTrafficLight({TrafficLightShapeType::TLS_UNKNOWN, TrafficLightShapeType::TLS_OTHER_SHAPE},
                                     {TrafficLightColorType::TLC_YELLOW, TrafficLightColorType::TLC_YELLOW_FLASHING}, turn, previous_light);
             !valid_objs.empty() && valid_objs.size() == unknown_shape_obj_num) {
    SetLightT(FindOptObj(previous_light, valid_objs), traffic_light);
    traffic_light.color                  = is_exist_flash ? message::sensor::TLC_YELLOW_FLASHING : message::sensor::TLC_YELLOW;
    traffic_light.traffic_light_flashing = is_exist_flash;
    traffic_light.traffic_reason         = byd::msg::orin::routing_map::LaneInfo::NEW_UNKNOWN_GREEN;
  } else if (valid_objs = SatisfyTrafficLight({TrafficLightShapeType::TLS_UNKNOWN, TrafficLightShapeType::TLS_OTHER_SHAPE},
                                              {TrafficLightColorType::TLC_RED, TrafficLightColorType::TLC_YELLOW,
                                               TrafficLightColorType::TLC_GREEN, TrafficLightColorType::TLC_GREEN_FLASHING,
                                               TrafficLightColorType::TLC_YELLOW_FLASHING, TrafficLightColorType::TLC_RED_FLASHING},
                                              turn, previous_light);
             !valid_objs.empty() && valid_objs.size() == unknown_shape_obj_num) {
    SetLightT(FindOptObj(previous_light, valid_objs), traffic_light);
    traffic_light.color          = message::sensor::TLC_BLURRING_MODE;
    traffic_light.traffic_reason = byd::msg::orin::routing_map::LaneInfo::NEW_BLURRING_SHAPE;
  } else if (valid_objs = SatisfyTrafficLight(
                 shapes_all, {TrafficLightColorType::TLC_OTHER, TrafficLightColorType::TLC_BLACK, TrafficLightColorType::TLC_UNKNOWN}, turn,
                 previous_light);
             !valid_objs.empty()) {
    traffic_light.traffic_light_flashing = false;
    traffic_light.traffic_light_num      = 1000;
    traffic_light.color                  = message::sensor::TLC_BLURRING_MODE;
    traffic_light.traffic_reason         = byd::msg::orin::routing_map::LaneInfo::NEW_BLURRING_COLOR;
  } else if (previous_light.is_valid && previous_light.traffic_reason != byd::msg::orin::routing_map::LaneInfo::SET_DEFAULT_OBJ &&
             traffic_light.perception_seq_num - previous_light.perception_seq_num <= max_stay_frames_with_info) {
    TRAFFIC_REC_LOG << "stay_prev_info turn:" << magic_enum::enum_name(turn);
    traffic_light                = previous_light;
    traffic_light.traffic_reason = byd::msg::orin::routing_map::LaneInfo::STAY_PREV;
    traffic_light.stay_prev_counter++;
  } else {
    TRAFFIC_REC_LOG << "no find any match info. turn:" << static_cast<int>(turn);
    traffic_light.traffic_reason = byd::msg::orin::routing_map::LaneInfo::UNKNOWN_STATE;
  }
}

void TrafficLightMapping::FillPerceptionTrafficLights() {
  SensorDataManager::Instance()->GetLatestSensorFrame(perception_traffic_light_info_);
  if (!perception_traffic_light_info_) {
    return;
  }

  // if (fabs(timestamp_ - traffic_light_info->header.timestamp) < 1e-3) {
  //   return;
  // }

  TRAFFIC_REC_LOG << fmt::format("traffic_light_info_cycle_counter:{}  publish_time:{:.3f}  measurement_time:{:.3f} ",
                                 perception_traffic_light_info_->header.cycle_counter,
                                 perception_traffic_light_info_->header.recv_timestamp, perception_traffic_light_info_->header.timestamp);
  timestamp_                    = perception_traffic_light_info_->header.timestamp;
  cycle_counter_perception_obj_ = perception_traffic_light_info_->header.cycle_counter;
  perception_traffic_lights_.clear();
  for (const auto &obj : perception_traffic_light_info_->objects) {
    if (obj.type != ObjectType::TRAFFIC_LIGHT) {
      continue;
    }
    if (obj.attributes.traffic_light_direction != TrafficLightDirectionType::TLD_BACK) {
      TRAFFIC_REC_LOG << fmt::format("invalid_need_remove_light  direction_reason id:{}  direction:{}", obj.id,
                                     obj.attributes.traffic_light_direction);
      continue;
    }
    if (std::isnan(obj.position.x) || std::isnan(obj.position.y)) {
      TRAFFIC_REC_LOG << fmt::format("invalid_need_remove_light nan_reason obj id:{} is pos_nan_value. pos x:{} y:{}  z:{}", obj.id,
                                     obj.position.x, obj.position.y, obj.position.z);
      continue;
    }

    if (obj.position.x > 180 || obj.position.x < -2) {
      TRAFFIC_REC_LOG << fmt::format("invalid_because_long_dis  obj:{} x:{:.2f}", obj.id, obj.position.x);
      continue;
    }

    if (FilterTrafficLightShape(obj.attributes.traffic_light_shape)) {
      TRAFFIC_REC_LOG << fmt::format("invalid_need_remove_light  shape_reason id:{}  shape:{}", obj.id, obj.attributes.traffic_light_shape);
      continue;
    }

    bool is_arrow_straight =
        std::find(straight_shapes_.begin(), straight_shapes_.end(), obj.attributes.traffic_light_shape) != straight_shapes_.end();
    bool is_arrow_left  = std::find(left_shapes_.begin(), left_shapes_.end(), obj.attributes.traffic_light_shape) != left_shapes_.end();
    bool is_arrow_right = std::find(right_shapes_.begin(), right_shapes_.end(), obj.attributes.traffic_light_shape) != right_shapes_.end();
    bool is_arrow_uturn = std::find(uturn_shapes_.begin(), uturn_shapes_.end(), obj.attributes.traffic_light_shape) != uturn_shapes_.end();
    if (is_arrow_straight || is_arrow_left || is_arrow_right || is_arrow_uturn) {
      if ((obj.det_source == cem::message::sensor::MONO_FRONT_NARROW && obj.shape_conf < 0.7) ||
          (obj.det_source == cem::message::sensor::MONO_FRONT_WIDE && obj.shape_conf < 0.5)) {
        TRAFFIC_REC_LOG << fmt::format("invalid_need_remove_light obj_id:{} camera_source:{} shape_conf:{:.2f}", obj.id,
                                       magic_enum::enum_name(obj.det_source), obj.shape_conf);
        continue;
      }
    }

    TRAFFIC_REC_LOG << fmt::format(
        "normal_traffic_light_id:{}  position:[{:.2f},{:.2f},{:.2f}] shape:{}  color:{}  num:{} is_flash:{:d} direction:{} track_age:{} "
        "direction_conf:{:.2f} shape_conf:{:.2f} det_source:{}",
        obj.id, obj.position.x, obj.position.y, obj.position.z, magic_enum::enum_name(obj.attributes.traffic_light_shape),
        magic_enum::enum_name(obj.attributes.traffic_light_color), obj.attributes.traffic_light_num, obj.attributes.traffic_light_flashing,
        magic_enum::enum_name(obj.attributes.traffic_light_direction), obj.track_age, obj.direction_conf, obj.shape_conf,
        magic_enum::enum_name(obj.det_source));

    perception_traffic_lights_.push_back(obj);
    perception_traffic_lights_.back().publish_time = perception_traffic_light_info_->header.timestamp;
  }
  traffic_lights_.header.timestamp      = perception_traffic_light_info_->header.timestamp;
  traffic_lights_.header.recv_timestamp = perception_traffic_light_info_->header.recv_timestamp;
  traffic_lights_.header.cycle_counter  = perception_traffic_light_info_->header.cycle_counter;
}

bool TrafficLightMapping::IsUnreliableRoadCrossJunction() {
  if (first_section_over_juntion_ != nullptr) {
    return first_section_over_juntion_->road_class >= cem::message::env_model::RoadClass::TOWN_ROAD;
  } else {
    return false;
  }
}

}  // namespace fusion
}  // namespace cem
