#include "cyberRT_adapter.h"
#include "base/sensor_data_manager.h"

namespace cem {
namespace fusion {
#if defined(MW_TYPE_CyberRT)

std::map<cem::message::sensor::TrafficLightColorType, cem::message::env_model::TlaColor> EnumMap2TlaColor = {
    {cem::message::sensor::TrafficLightColorType::TLC_UNKNOWN, cem::message::env_model::TlaColor::COLOR_NO_DISPLAY},
    {cem::message::sensor::TrafficLightColorType::TLC_RED, cem::message::env_model::TlaColor::RED},
    {cem::message::sensor::TrafficLightColorType::TLC_GREEN, cem::message::env_model::TlaColor::GREEN},
    {cem::message::sensor::TrafficLightColorType::TLC_YELLOW, cem::message::env_model::TlaColor::YELLOW},
};

std::map<byd::msg::env_model::LaneMarkerLocation, cem::message::env_model::TrfLaneMarkerPosition> EnumMap2LaneMarkerPosition = {
    {byd::msg::env_model::LaneMarkerLocation::LM_LOC_INVALID, cem::message::env_model::TrfLaneMarkerPosition::TRF_LMP__UNKNOWN},
    {byd::msg::env_model::LaneMarkerLocation::LM_LOC_LEFT_FIRST, cem::message::env_model::TrfLaneMarkerPosition::TRF_LMP__FIRST_LEFT},
    {byd::msg::env_model::LaneMarkerLocation::LM_LOC_RIGHT_FIRST, cem::message::env_model::TrfLaneMarkerPosition::TRF_LMP__FIRST_RIGHT},
    {byd::msg::env_model::LaneMarkerLocation::LM_LOC_LEFT_SECOND, cem::message::env_model::TrfLaneMarkerPosition::TRF_LMP__SECOND_LEFT},
    {byd::msg::env_model::LaneMarkerLocation::LM_LOC_RIGHT_SECOND, cem::message::env_model::TrfLaneMarkerPosition::TRF_LMP__SECOND_RIGHT},
    {byd::msg::env_model::LaneMarkerLocation::LM_LOC_LEFT_THIRD, cem::message::env_model::TrfLaneMarkerPosition::TRF_LMP__THIRD_LEFT},
    {byd::msg::env_model::LaneMarkerLocation::LM_LOC_RIGHT_THIRD, cem::message::env_model::TrfLaneMarkerPosition::TRF_LMP__THIRD_RIGHT},
    {byd::msg::env_model::LaneMarkerLocation::LM_LOC_LEFT_FOURTH, cem::message::env_model::TrfLaneMarkerPosition::TRF_LMP__FOURTH_LEFT},
    {byd::msg::env_model::LaneMarkerLocation::LM_LOC_RIGHT_FOURTH, cem::message::env_model::TrfLaneMarkerPosition::TRF_LMP__FOURTH_RIGHT},
    {byd::msg::env_model::LaneMarkerLocation::LM_LOC_CROSSING, cem::message::env_model::TrfLaneMarkerPosition::TRF_LMP__OTHER},
    {byd::msg::env_model::LaneMarkerLocation::LM_LOC_LEFT_FIFTH, cem::message::env_model::TrfLaneMarkerPosition::TRF_LMP__FIFTH_LEFT},
    {byd::msg::env_model::LaneMarkerLocation::LM_LOC_RIGHT_FIFTH, cem::message::env_model::TrfLaneMarkerPosition::TRF_LMP__FIFTH_RIGHT},
    {byd::msg::env_model::LaneMarkerLocation::LM_LOC_LEFT_SIXTH, cem::message::env_model::TrfLaneMarkerPosition::TRF_LMP__SIXTH_LEFT},
    {byd::msg::env_model::LaneMarkerLocation::LM_LOC_RIGHT_SIXTH, cem::message::env_model::TrfLaneMarkerPosition::TRF_LMP__SIXTH_RIGHT},
    {byd::msg::env_model::LaneMarkerLocation::LM_LOC_LEFT_SEVENTH, cem::message::env_model::TrfLaneMarkerPosition::TRF_LMP__SEVENTH_LEFT},
    {byd::msg::env_model::LaneMarkerLocation::LM_LOC_RIGHT_SEVENTH, cem::message::env_model::TrfLaneMarkerPosition::TRF_LMP__SEVENTH_RIGHT},
    {byd::msg::env_model::LaneMarkerLocation::LM_LOC_LEFT_EIGHTH, cem::message::env_model::TrfLaneMarkerPosition::TRF_LMP__OTHER},
    {byd::msg::env_model::LaneMarkerLocation::LM_LOC_RIGHT_EIGHTH, cem::message::env_model::TrfLaneMarkerPosition::TRF_LMP__OTHER}};

std::map<byd::msg::env_model::LaneMarkerType, BevLaneMarkerType> EnumMap2LaneMarkerType = {
    {byd::msg::env_model::LaneMarkerType::LM_TYPE_UNKNOWN, BevLaneMarkerType::BEV_LMT__UNDECIDED},
    {byd::msg::env_model::LaneMarkerType::LM_TYPE_DASHED, BevLaneMarkerType::BEV_LMT__DASHED},
    {byd::msg::env_model::LaneMarkerType::LM_TYPE_SOLID, BevLaneMarkerType::BEV_LMT__SOLID},
    {byd::msg::env_model::LaneMarkerType::LM_TYPE_DOUBLE_DASHED, BevLaneMarkerType::BEV_LMT__DOUBLE_DASHED_DASHED},
    {byd::msg::env_model::LaneMarkerType::LM_TYPE_DOUBLE_SOLID, BevLaneMarkerType::BEV_LMT__DOUBLE_SOLID_SOLID},
    {byd::msg::env_model::LaneMarkerType::LM_TYPE_DASHED_SOLID, BevLaneMarkerType::BEV_LMT__DOUBLE_DASHED_SOLID},
    {byd::msg::env_model::LaneMarkerType::LM_TYPE_SOLID_DASHED, BevLaneMarkerType::BEV_LMT__DOUBLE_SOLID_DASHED},
    {byd::msg::env_model::LaneMarkerType::LM_TYPE_INVALID, BevLaneMarkerType::BEV_LMT__INVALID},
    {byd::msg::env_model::LaneMarkerType::RM_TYPE_CROSSWALK, BevLaneMarkerType::BEV_RMT__CROSSWALK},
    {byd::msg::env_model::LaneMarkerType::RM_TYPE_STRAIGHT, BevLaneMarkerType::BEV_RMT__STRAIGHT},
    {byd::msg::env_model::LaneMarkerType::RM_TYPE_LEFT, BevLaneMarkerType::BEV_RMT__LEFT},
    {byd::msg::env_model::LaneMarkerType::RM_TYPE_RIGHT, BevLaneMarkerType::BEV_RMT__RIGHT},
    {byd::msg::env_model::LaneMarkerType::RM_TYPE_TURNING, BevLaneMarkerType::BEV_RMT__TURNING},
    {byd::msg::env_model::LaneMarkerType::RM_TYPE_STRAIGHT_LEFT, BevLaneMarkerType::BEV_RMT__STRAIGHT_LEFT},
    {byd::msg::env_model::LaneMarkerType::RM_TYPE_STRAIGHT_RIGHT, BevLaneMarkerType::BEV_RMT__STRAIGHT_RIGHT},
    {byd::msg::env_model::LaneMarkerType::RM_TYPE_STRAIGHT_LEFT_RIGHT, BevLaneMarkerType::BEV_RMT__STRAIGHT_LEFT_RIGHT},
    {byd::msg::env_model::LaneMarkerType::RM_TYPE_LEFT_RIGHT, BevLaneMarkerType::BEV_RMT__LEFT_RIGHT},
    {byd::msg::env_model::LaneMarkerType::RM_TYPE_STRAIGHT_TURNING, BevLaneMarkerType::BEV_RMT__STRAIGHT_TURNING},
    {byd::msg::env_model::LaneMarkerType::RM_TYPE_LEFT_TURNING, BevLaneMarkerType::BEV_RMT__LEFT_TURNING},
    {byd::msg::env_model::LaneMarkerType::RM_TYPE_IMPORT, BevLaneMarkerType::BEV_RMT__IMPORT},
    {byd::msg::env_model::LaneMarkerType::RM_TYPE_EXPORT, BevLaneMarkerType::BEV_RMT__EXPORT},
    {byd::msg::env_model::LaneMarkerType::RM_TYPE_STOPLINE, BevLaneMarkerType::BEV_RMT__STOPLINE},
    {byd::msg::env_model::LaneMarkerType::RM_TYPE_DECELERATION_ZONE, BevLaneMarkerType::RM_TYPE_DECELERATION_ZONE},
    {byd::msg::env_model::LaneMarkerType::RM_TYPE_DIVERSION_ZONE, BevLaneMarkerType::RM_TYPE_DIVERSION_ZONE},
    {byd::msg::env_model::LaneMarkerType::RM_TYPE_INTERSECTION_ZONE, BevLaneMarkerType::RM_TYPE_INTERSECTION_ZONE},
    {byd::msg::env_model::LaneMarkerType::LM_TYPE_WIDEDASHED, BevLaneMarkerType::BEV_LMT__DASHED}};

std::map<byd::msg::env_model::LaneMarkerColor, BevLaneMarkerColor> EnumMap2LaneMarkerColor = {
    {byd::msg::env_model::LaneMarkerColor::LM_COLOR_WHITE, BevLaneMarkerColor::BEV_LMC__WHITE},
    {byd::msg::env_model::LaneMarkerColor::LM_COLOR_YELLOW, BevLaneMarkerColor::BEV_LMC__YELLOW},
    {byd::msg::env_model::LaneMarkerColor::LM_COLOR_UNKNOWN, BevLaneMarkerColor::BEV_LMC__UNKNOWN},
    {byd::msg::env_model::LaneMarkerColor::LM_COLOR_BLUE, BevLaneMarkerColor::BEV_LMC__BLUE},
    {byd::msg::env_model::LaneMarkerColor::LM_COLOR_GREEN, BevLaneMarkerColor::BEV_LMC__GREEN},
    {byd::msg::env_model::LaneMarkerColor::LM_COLOR_RED, BevLaneMarkerColor::BEV_LMC__RED},
};

std::map<byd::msg::env_model::RoadEdgeLocation, cem::message::env_model::TrafficRoadEdgePosition> EnumMap2RoadEdgePosition = {
    {byd::msg::env_model::RoadEdgeLocation::RE_LOC_INVALID, cem::message::env_model::TrafficRoadEdgePosition::TRE_REP__UNKNOWN},
    {byd::msg::env_model::RoadEdgeLocation::RE_LOC_LEFT, cem::message::env_model::TrafficRoadEdgePosition::TRE_REP__LEFT},
    {byd::msg::env_model::RoadEdgeLocation::RE_LOC_RIGHT, cem::message::env_model::TrafficRoadEdgePosition::TRE_REP__RIGHT},
};

std::map<byd::msg::env_model::RoadEdgeType, TrafficRoadEdgeType> EnumMap2RoadEdgeType = {
    {byd::msg::env_model::RoadEdgeType::ROADEDGE_TYPE_UNKNOWN, TrafficRoadEdgeType::TRE_RET__UNKNOWN},
    {byd::msg::env_model::RoadEdgeType::ROADEDGE_TYPE_FLAT, TrafficRoadEdgeType::TRE_RET__FLAT},
    {byd::msg::env_model::RoadEdgeType::ROADEDGE_TYPE_LOW, TrafficRoadEdgeType::TRE_RET__CURB},
    {byd::msg::env_model::RoadEdgeType::ROADEDGE_TYPE_HIGH, TrafficRoadEdgeType::TRE_RET__ELEVATED},
    {byd::msg::env_model::RoadEdgeType::ROADEDGE_TYPE_FENCE, TrafficRoadEdgeType::TRE_RET__CONE_AND_POLE},
};

std::map<byd::msg::env_model::RoadEdgeLocation, BevRoadEdgePosition> EnumMap2BevRoadEdgePosition = {
    {byd::msg::env_model::RoadEdgeLocation::RE_LOC_INVALID, BEV_REP__UNKNOWN},
    {byd::msg::env_model::RoadEdgeLocation::RE_LOC_LEFT, BEV_REP__LEFT},
    {byd::msg::env_model::RoadEdgeLocation::RE_LOC_RIGHT, BEV_REP__RIGHT},
};

std::unordered_map<
    SensorWarningType,
    byd::msg::orin::routing_map::SensorStatusInfo_SensorWarningType>
    SensorInfoWarningType = {
        {SensorWarningType::NO_WARNING,
         byd::msg::orin::routing_map::SensorStatusInfo_SensorWarningType::
             SensorStatusInfo_SensorWarningType_NO_WARNING},
        {SensorWarningType::CLOSE_TO_UNRELIABLE_ROAD,
         byd::msg::orin::routing_map::SensorStatusInfo_SensorWarningType::
             SensorStatusInfo_SensorWarningType_CLOSE_TO_UNRELIABLE_ROAD},
        {SensorWarningType::SWITCH_TO_HD_MAP_FAILURE,
         byd::msg::orin::routing_map::SensorStatusInfo_SensorWarningType::
             SensorStatusInfo_SensorWarningType_SWITCH_TO_HD_MAP_FAILURE}};

void AdaptInput2Internal(const std::shared_ptr<byd::msg::drivers::VehInfo> &raw_veh_signal, VehicleSignalPtr adapted_veh_signal) {
  adapted_veh_signal->header.timestamp      = raw_veh_signal->header().measurement_timestamp();
  adapted_veh_signal->header.recv_timestamp = GetMwTimeNowSec();
  adapted_veh_signal->steering_wheel_angle  = raw_veh_signal->steering_system_status().da_in_strgwhlang_sg() * 57.3 / 13.1;
  adapted_veh_signal->dynamic_yaw_rate      = raw_veh_signal->ego_motion_status().da_in_yawrate_sg();
  adapted_veh_signal->stand_still           = raw_veh_signal->ego_motion_status().da_in_vehstdstlsts_u8();
  adapted_veh_signal->average_speed         = raw_veh_signal->ego_motion_status().da_in_vehspd_sg();
  /**/
  if (1u == raw_veh_signal->body_system_status().da_in_turnindsts_u8()) {
    adapted_veh_signal->left_direction_light  = 1;
    adapted_veh_signal->right_direction_light = 0;
  } else if (2u == raw_veh_signal->body_system_status().da_in_turnindsts_u8()) {
    adapted_veh_signal->left_direction_light  = 0;
    adapted_veh_signal->right_direction_light = 1;
  } else {
    adapted_veh_signal->left_direction_light  = 0;
    adapted_veh_signal->right_direction_light = 0;
  }
  /**/
  adapted_veh_signal->lat_accel = raw_veh_signal->ego_motion_status().da_in_latacc_sg();
  adapted_veh_signal->lon_accel = raw_veh_signal->ego_motion_status().da_in_lgtacc_sg();

  if (0u == raw_veh_signal->propulsion_system_status().da_in_trmgearlvl_u8()) {
    adapted_veh_signal->gear_info = 0;
  } else if (1u == raw_veh_signal->propulsion_system_status().da_in_trmgearlvl_u8()) {
    adapted_veh_signal->gear_info = 1;
  } else if (2u == raw_veh_signal->propulsion_system_status().da_in_trmgearlvl_u8()) {
    adapted_veh_signal->gear_info = 3;
  } else if (3u == raw_veh_signal->propulsion_system_status().da_in_trmgearlvl_u8()) {
    adapted_veh_signal->gear_info = 4;
  } else if (4u == raw_veh_signal->propulsion_system_status().da_in_trmgearlvl_u8()) {
    adapted_veh_signal->gear_info = 2;
  } else {
    adapted_veh_signal->gear_info = 0;
  }
}

// Point2D internal(map_info)
void AdaptInput2Internal(const byd::msg::basic::Point2D &raw_point2d, Point2D *adapted_point) {
  adapted_point->x = raw_point2d.x();
  adapted_point->y = raw_point2d.y();
}

// Point internal(map_info)
void AdaptInput2Internal(const byd::msg::orin::routing_map::Point &raw_point, Point *adapted_point) {
  adapted_point->x         = raw_point.x();
  adapted_point->y         = raw_point.y();
  adapted_point->z         = raw_point.z();
  adapted_point->curvature = raw_point.curvature();
}

//Point3DD internal(map_info)
void AdaptInput2Internal(const byd::msg::basic::Point3D &raw_point, cem::message::common::Point3DD *adapt_point) {
  adapt_point->x = raw_point.x();
  adapt_point->y = raw_point.y();
  adapt_point->z = raw_point.z();
}

// SectionInfo(map_info)
void AdaptInput2Internal(const byd::msg::orin::routing_map::SectionInfo &raw_section_info, SectionInfo *adapted_section_info) {
  adapted_section_info->id     = raw_section_info.id();
  adapted_section_info->length = raw_section_info.length();

  adapted_section_info->lane_ids.reserve(raw_section_info.lane_ids().size());
  adapted_section_info->lane_num = raw_section_info.lane_ids().size();

  adapted_section_info->lane_ids.assign(raw_section_info.lane_ids().begin(), raw_section_info.lane_ids().end());

  adapted_section_info->points.reserve(raw_section_info.points().size());

  for (const auto &pt : raw_section_info.points()) {
    adapted_section_info->points.emplace_back(Point());
    auto &adapted_section_point = adapted_section_info->points.back();
    AdaptInput2Internal(pt, &adapted_section_point);
  };

  adapted_section_info->road_class = static_cast<cem::message::env_model::RoadClass>(raw_section_info.road_class());

  // 新增适配 predecessor_section_id_list
  adapted_section_info->predecessor_section_id_list.reserve(raw_section_info.predecessor_section_id_list().size());
  adapted_section_info->predecessor_section_id_list.assign(raw_section_info.predecessor_section_id_list().begin(),
                                                           raw_section_info.predecessor_section_id_list().end());

  // 新增适配 successor_section_id_list
  adapted_section_info->successor_section_id_list.reserve(raw_section_info.successor_section_id_list().size());
  adapted_section_info->successor_section_id_list.assign(raw_section_info.successor_section_id_list().begin(),
                                                         raw_section_info.successor_section_id_list().end());
  adapted_section_info->link_type = raw_section_info.link_type();
  adapted_section_info->lane_num  = adapted_section_info->lane_ids.size();
}

// NaviPosition internal(map_info)
void AdaptInput2Internal(const byd::msg::orin::routing_map::NaviPosition &raw_position, NaviPosition *adapted_position) {
  adapted_position->section_id = raw_position.section_id();
  adapted_position->s_offset   = raw_position.s_offset();
}

// RouteInfo internal(map_info)
void AdaptInput2Internal(const byd::msg::orin::routing_map::RouteInfo &raw_route, RouteInfo *adapted_route) {
  adapted_route->id        = raw_route.id();
  auto &adapted_navi_start = adapted_route->navi_start;
  AdaptInput2Internal(raw_route.navi_start(), &adapted_navi_start);

  adapted_route->sections.reserve(raw_route.sections().size());
  for (const auto &sec : raw_route.sections()) {
    adapted_route->sections.emplace_back(SectionInfo());
    auto &adapted_sec = adapted_route->sections.back();
    AdaptInput2Internal(sec, &adapted_sec);
  }

  // 适配 subpaths
  adapted_route->subpaths.reserve(raw_route.subpaths().size());
  for (const auto &raw_subpath : raw_route.subpaths()) {
    SubPath adapted_subpath;
    adapted_subpath.enter_section_id = raw_subpath.enter_section_id();

    adapted_subpath.sections.reserve(raw_subpath.sections().size());
    for (const auto &raw_section : raw_subpath.sections()) {
      SectionInfo section;
      AdaptInput2Internal(raw_section, &section);
      adapted_subpath.sections.push_back(std::move(section));
    }
    adapted_route->subpaths.push_back(std::move(adapted_subpath));
  }
  adapted_route->route_id = raw_route.route_id();
  adapted_route->extend_sections.reserve(raw_route.extend_sections().size());
  for (const auto &raw_section : raw_route.extend_sections()) {
    PathExtend extend_section;
    extend_section.from_section_idx = raw_section.from_section_idx();
    extend_section.from_section_id  = raw_section.from_section_id();
    extend_section.to_section_idx   = raw_section.to_section_idx();
    extend_section.sections.reserve(raw_section.sections().size());
    for (const auto &sec : raw_section.sections()) {
      SectionInfo section;
      AdaptInput2Internal(sec, &section);
      extend_section.sections.push_back(std::move(section));
    }
    adapted_route->extend_sections.push_back(std::move(extend_section));
  }
}

// StopLineInfo internal
void AdaptInput2Internal(const byd::msg::orin::routing_map::StopLineInfo &raw_stop_line, StopLineInfo *adapted_stop_line) {
  adapted_stop_line->id   = raw_stop_line.id();
  adapted_stop_line->type = raw_stop_line.type();

  adapted_stop_line->points.reserve(raw_stop_line.points().size());
  for (const auto &pt : raw_stop_line.points()) {
    adapted_stop_line->points.emplace_back(Point2D());
    auto &adapted_sl_point = adapted_stop_line->points.back();
    AdaptInput2Internal(pt, &adapted_sl_point);
  }
}

// JunctionInfo internal
void AdaptInput2Internal(const byd::msg::orin::routing_map::JunctionInfo &raw_junction_info, JunctionInfo *adapted_junction_info) {
  adapted_junction_info->id = raw_junction_info.id();
  adapted_junction_info->points.reserve(raw_junction_info.points().size());
  for (const auto &pt : raw_junction_info.points()) {
    adapted_junction_info->points.emplace_back(Point2D());
    auto &adapted_junction_point = adapted_junction_info->points.back();
    AdaptInput2Internal(pt, &adapted_junction_point);
  }
}

// CrossWalkInfo internal
void AdaptInput2Internal(const byd::msg::orin::routing_map::CrossWalkInfo &raw_cross_walk, CrossWalkInfo *adapted_cross_walk) {
  adapted_cross_walk->id = raw_cross_walk.id();
  adapted_cross_walk->points.reserve(raw_cross_walk.points().size());
  for (const auto &pt : raw_cross_walk.points()) {
    adapted_cross_walk->points.emplace_back(Point2D());
    auto &adapted_cwalk_point = adapted_cross_walk->points.back();
    AdaptInput2Internal(pt, &adapted_cwalk_point);
  }
}

// RoadBoundaryInfo internal(map_info)
void AdaptInput2Internal(const byd::msg::orin::routing_map::RoadBoundaryInfo &raw_road_bd, RoadBoundaryInfo *adapted_road_bd) {
  adapted_road_bd->id            = raw_road_bd.id();
  adapted_road_bd->boundary_type = static_cast<BoundaryType>(raw_road_bd.boundary_type());

  adapted_road_bd->points.reserve(raw_road_bd.points().size());
  for (const auto &pt : raw_road_bd.points()) {
    adapted_road_bd->points.emplace_back(Point2D());
    auto &adapted_road_point = adapted_road_bd->points.back();
    AdaptInput2Internal(pt, &adapted_road_point);
  }
}

// LaneBoundaryInfo internal(map_info)
void AdaptInput2Internal(const byd::msg::orin::routing_map::LaneBoundaryInfo &raw_lane_bd, LaneBoundaryInfo *adapted_lane_bd) {
  adapted_lane_bd->id        = raw_lane_bd.id();
  adapted_lane_bd->line_type = static_cast<LineType>(raw_lane_bd.line_type());

  adapted_lane_bd->points.reserve(raw_lane_bd.points().size());
  for (const auto &pt : raw_lane_bd.points()) {
    adapted_lane_bd->points.emplace_back(Point2D());
    auto &adapted_lane_point = adapted_lane_bd->points.back();
    AdaptInput2Internal(pt, &adapted_lane_point);
  }
}

// LaneInfo internal(map_info)
void AdaptInput2Internal(const byd::msg::orin::routing_map::LaneInfo &raw_lane_info, LaneInfo *adapted_lane_info) {
  adapted_lane_info->id            = raw_lane_info.id();
  adapted_lane_info->section_id    = raw_lane_info.section_id();
  adapted_lane_info->junction_id   = raw_lane_info.junction_id();
  adapted_lane_info->left_lane_id  = raw_lane_info.left_lane_id();
  adapted_lane_info->right_lane_id = raw_lane_info.right_lane_id();

  adapted_lane_info->next_lane_ids.reserve(raw_lane_info.next_lane_ids().size());

  adapted_lane_info->next_lane_ids.assign(raw_lane_info.next_lane_ids().begin(), raw_lane_info.next_lane_ids().end());

  adapted_lane_info->left_lane_boundary_ids.reserve(raw_lane_info.left_lane_boundary_ids().size());

  adapted_lane_info->left_lane_boundary_ids.assign(raw_lane_info.left_lane_boundary_ids().begin(),
                                                   raw_lane_info.left_lane_boundary_ids().end());

  adapted_lane_info->right_lane_boundary_ids.reserve(raw_lane_info.right_lane_boundary_ids().size());

  adapted_lane_info->right_lane_boundary_ids.assign(raw_lane_info.right_lane_boundary_ids().begin(),
                                                    raw_lane_info.right_lane_boundary_ids().end());

  adapted_lane_info->left_road_boundary_ids.reserve(raw_lane_info.left_road_boundary_ids().size());

  adapted_lane_info->left_road_boundary_ids.assign(raw_lane_info.left_road_boundary_ids().begin(),
                                                   raw_lane_info.left_road_boundary_ids().end());

  adapted_lane_info->right_road_boundary_ids.reserve(raw_lane_info.right_road_boundary_ids().size());

  adapted_lane_info->right_road_boundary_ids.assign(raw_lane_info.right_road_boundary_ids().begin(),
                                                    raw_lane_info.right_road_boundary_ids().end());

  // previous_lane_ids
  adapted_lane_info->previous_lane_ids.reserve(raw_lane_info.previous_lane_ids().size());
  adapted_lane_info->previous_lane_ids.assign(raw_lane_info.previous_lane_ids().begin(), raw_lane_info.previous_lane_ids().end());

  adapted_lane_info->type           = static_cast<LaneType>(raw_lane_info.type());
  adapted_lane_info->none_odd_type  = static_cast<NoneOddType>(raw_lane_info.none_odd_type());
  adapted_lane_info->turn_type      = static_cast<TurnType>(raw_lane_info.turn_type());
  adapted_lane_info->light_status   = LightStatus::NONE_LIGHT;  // static_cast<LightStatus>(raw_lane_info.light_status());
  adapted_lane_info->split_topology = static_cast<SplitTopology>(raw_lane_info.split_topology());
  adapted_lane_info->merge_topology = static_cast<MergeTopology>(raw_lane_info.merge_topology());

  adapted_lane_info->length          = raw_lane_info.length();
  adapted_lane_info->speed_limit     = raw_lane_info.speed_limit();
  adapted_lane_info->is_virtual      = raw_lane_info.is_virtual();
  adapted_lane_info->cnoa_is_virtual = raw_lane_info.is_virtual();
  adapted_lane_info->lane_seq        = raw_lane_info.lane_seq();

  adapted_lane_info->light_countdown = raw_lane_info.light_countdown();

  adapted_lane_info->cross_walks.reserve(raw_lane_info.cross_walks().size());

  adapted_lane_info->cross_walks.assign(raw_lane_info.cross_walks().begin(), raw_lane_info.cross_walks().end());

  adapted_lane_info->traffic_stop_lines.reserve(raw_lane_info.traffic_stop_lines().size());

  adapted_lane_info->traffic_stop_lines.assign(raw_lane_info.traffic_stop_lines().begin(), raw_lane_info.traffic_stop_lines().end());

  adapted_lane_info->points.reserve(raw_lane_info.points().size());

  for (const auto &pt : raw_lane_info.points()) {
    adapted_lane_info->points.emplace_back(Point());
    auto &adapted_lane_point = adapted_lane_info->points.back();
    AdaptInput2Internal(pt, &adapted_lane_point);
  }
  //exp_trajectory_ids
  adapted_lane_info->exp_trajectory_ids.reserve(raw_lane_info.exp_trajectory_ids().size());
  adapted_lane_info->exp_trajectory_ids.assign(raw_lane_info.exp_trajectory_ids().begin(), raw_lane_info.exp_trajectory_ids().end());

  adapted_lane_info->restricted_info = RestrictedInfo{};
  if (raw_lane_info.has_restricted_info()) {
    const auto &ri = raw_lane_info.restricted_info();
    if (ri.has_restricted_type()) {
      adapted_lane_info->restricted_info.restricted_type = static_cast<RestrictedType>(ri.restricted_type());
    }
    // if (ri.has_is_passable()) {
    //   adapted_lane_info->restricted_info.is_passable = ri.is_passable();
    // }
    if (ri.has_passable_env_state()) {
      adapted_lane_info->restricted_info.passable_env_state = ri.passable_env_state();
    }
  }
}

//traffic_light(map_info)
void AdaptInput2Internal(const byd::msg::orin::routing_map::TrafficLightInfo &raw_traffic_light, TrafficLightMap *adapted_traffic_light) {
  adapted_traffic_light->id = raw_traffic_light.id();
  AdaptInput2Internal(raw_traffic_light.center_position(), &adapted_traffic_light->center_position);
  for (const auto &pt : raw_traffic_light.bounding_box_geometry()) {
    adapted_traffic_light->bounding_box_geometry.emplace_back(Point3DD());
    auto &adapted_point = adapted_traffic_light->bounding_box_geometry.back();
    AdaptInput2Internal(pt, &adapted_point);
  }
  adapted_traffic_light->laneids.reserve(raw_traffic_light.laneids().size());
  adapted_traffic_light->laneids.assign(raw_traffic_light.laneids().begin(), raw_traffic_light.laneids().end());
  adapted_traffic_light->light_countdown = raw_traffic_light.light_countdown();
  adapted_traffic_light->shape           = raw_traffic_light.shape();
  adapted_traffic_light->light_status    = static_cast<LightStatus>(raw_traffic_light.light_status());
}

void AdaptInput2Internal(const byd::msg::orin::routing_map::EnvInfo &proto_env_info, cem::message::env_model::EnvInfo *internal_env_info) {
  internal_env_info->v2_valid                          = proto_env_info.v2_valid();
  internal_env_info->v2_has_navigation                 = proto_env_info.v2_has_navigation();
  internal_env_info->v2_dist_to_ramp                   = proto_env_info.v2_dist_to_ramp();
  internal_env_info->v2_dist_to_toll                   = proto_env_info.v2_dist_to_toll();
  internal_env_info->v2_dist_to_tunnel                 = proto_env_info.v2_dist_to_tunnel();
  internal_env_info->v2_dist_to_subpath                = proto_env_info.dist_to_subpath();
  internal_env_info->v2_dist_to_split_routelanenum_dec = proto_env_info.dist_to_split_routelanenum_dec();

  internal_env_info->v2_road_classes.reserve(proto_env_info.v2_road_classes().size());
  for (const auto &proto_road_class : proto_env_info.v2_road_classes()) {
    cem::message::env_model::V2RoadClass internal_road_class;
    internal_road_class.start      = proto_road_class.start();
    internal_road_class.end        = proto_road_class.end();
    internal_road_class.road_class = static_cast<cem::message::env_model::V2RoadClassType>(static_cast<int>(proto_road_class.road_class()));
    internal_env_info->v2_road_classes.push_back(std::move(internal_road_class));
  }

  internal_env_info->v2_turn_info.reserve(proto_env_info.v2_turn_info().size());
  for (const auto &proto_turn_info : proto_env_info.v2_turn_info()) {
    cem::message::env_model::V2TurnInfo internal_turn_info;
    internal_turn_info.id       = proto_turn_info.id();
    internal_turn_info.is_valid = proto_turn_info.is_valid();
    internal_turn_info.turn_type =
        static_cast<cem::message::env_model::V2TurnInfo::V2TurnType>(static_cast<int>(proto_turn_info.turn_type()));
    internal_turn_info.detail_turn_type =
        static_cast<cem::message::env_model::V2TurnInfo::V2DetailTurnType>(static_cast<int>(proto_turn_info.detail_turn_type()));
    internal_turn_info.dist = proto_turn_info.dist();

    cem::message::env_model::V2RoadInfo before_turn;
    before_turn.road_class =
        static_cast<cem::message::env_model::V2RoadClassType>(static_cast<int>(proto_turn_info.before_turn().road_class()));
    before_turn.lane_num           = proto_turn_info.before_turn().lane_num();
    internal_turn_info.before_turn = std::move(before_turn);

    cem::message::env_model::V2RoadInfo after_turn;
    after_turn.road_class =
        static_cast<cem::message::env_model::V2RoadClassType>(static_cast<int>(proto_turn_info.after_turn().road_class()));
    after_turn.lane_num           = proto_turn_info.after_turn().lane_num();
    internal_turn_info.after_turn = after_turn;

    internal_env_info->v2_turn_info.push_back(std::move(internal_turn_info));
  }
}

///SDRouteInfo
void AdaptInput2Internal(const byd::msg::orin::routing_map::SDRouteInfo &proto_sd_route_info,
                         cem::message::env_model::SDRouteInfo           *internal_env_sd_route_info) {

  internal_env_sd_route_info->id = proto_sd_route_info.id();

  auto AdaptedSDSectionInfo = [](const byd::msg::orin::routing_map::SDSectionInfo &proto_sd_section_info,
                                 cem::message::env_model::SDSectionInfo           *internal_sd_section_info) {
    internal_sd_section_info->id               = proto_sd_section_info.id();
    internal_sd_section_info->lane_num         = proto_sd_section_info.lane_num();
    internal_sd_section_info->length           = proto_sd_section_info.length();
    internal_sd_section_info->link_type        = proto_sd_section_info.link_type();
    internal_sd_section_info->exp_speed        = proto_sd_section_info.exp_speed();
    internal_sd_section_info->has_toll_station = proto_sd_section_info.has_toll_station();
    internal_sd_section_info->direction        = static_cast<SDDirectionType>(proto_sd_section_info.direction());
    if (internal_sd_section_info->points) {
      internal_sd_section_info->points->clear();
      for (auto &raw_pt : proto_sd_section_info.points()) {
        internal_sd_section_info->points->emplace_back(raw_pt.x(), raw_pt.y());
      }
    }

    internal_sd_section_info->predecessor_section_id_list.clear();
    for (auto &raw_predecessor_section_id : proto_sd_section_info.predecessor_section_id_list()) {
      internal_sd_section_info->predecessor_section_id_list.emplace_back(raw_predecessor_section_id);
    }

    internal_sd_section_info->reverse_lane_count = proto_sd_section_info.reverse_lane_count();
    internal_sd_section_info->road_class         = static_cast<SDRoadClass>(proto_sd_section_info.road_class());
    internal_sd_section_info->speed_limit        = proto_sd_section_info.speed_limit();
    internal_sd_section_info->successor_section_id_list.clear();
    for (auto &raw_successor_section_id : proto_sd_section_info.successor_section_id_list()) {
      internal_sd_section_info->successor_section_id_list.emplace_back(raw_successor_section_id);
    }

    internal_sd_section_info->lane_group_idx.clear();
    for (const auto &raw_lane_group_idx : proto_sd_section_info.lane_group_idx()) {
      SDLaneGroupIndex adapted_lane_group_idx;
      adapted_lane_group_idx.id                 = raw_lane_group_idx.id();
      adapted_lane_group_idx.start_range_offset = raw_lane_group_idx.start_range_offset();
      adapted_lane_group_idx.end_range_offset   = raw_lane_group_idx.end_range_offset();
      internal_sd_section_info->lane_group_idx.push_back(adapted_lane_group_idx);
    }

    if (proto_sd_section_info.has_has_junction()) {
      internal_sd_section_info->has_junction = proto_sd_section_info.has_junction();
    }

    if (proto_sd_section_info.has_link_type_extend()) {
      internal_sd_section_info->link_type_extend = proto_sd_section_info.link_type_extend();
    }
  };

  internal_env_sd_route_info->mpp_sections.reserve(proto_sd_route_info.mpp_sections().size());
  for (auto &raw_mpp_section : proto_sd_route_info.mpp_sections()) {
    internal_env_sd_route_info->mpp_sections.emplace_back(SDSectionInfo());
    auto &adapted_sd_section_info = internal_env_sd_route_info->mpp_sections.back();
    AdaptedSDSectionInfo(raw_mpp_section, &adapted_sd_section_info);
  }

  AdaptInput2Internal(proto_sd_route_info.navi_start(), &internal_env_sd_route_info->navi_start);

  auto AdaptedSDSubPath = [&](const byd::msg::orin::routing_map::SDSubPath &proto_sd_sub_path,
                              cem::message::env_model::SDSubPath           *internal_sd_sub_path) {
    internal_sd_sub_path->enter_section_id = proto_sd_sub_path.enter_section_id();
    internal_sd_sub_path->path_type        = static_cast<SDPathType>(proto_sd_sub_path.path_type());
    internal_sd_sub_path->sections.clear();
    for (auto &raw_section : proto_sd_sub_path.sections()) {
      internal_sd_sub_path->sections.emplace_back(SDSectionInfo());
      auto &adapted_sd_section_info = internal_sd_sub_path->sections.back();
      AdaptedSDSectionInfo(raw_section, &adapted_sd_section_info);
    }
  };

  internal_env_sd_route_info->subpaths.clear();
  for (auto &raw_sub_path : proto_sd_route_info.subpaths()) {
    internal_env_sd_route_info->subpaths.emplace_back(SDSubPath());
    auto &adapted_sub_path = internal_env_sd_route_info->subpaths.back();
    AdaptedSDSubPath(raw_sub_path, &adapted_sub_path);
  }

  // recommend_lane_list
  internal_env_sd_route_info->recommend_lane_list.clear();
  for (const auto &raw_recommend_lane_list : proto_sd_route_info.recommend_lane_list()) {
    internal_env_sd_route_info->recommend_lane_list.emplace_back(SDRecommendLaneGroup());
    auto &adapted_recommend_lane_list         = internal_env_sd_route_info->recommend_lane_list.back();
    adapted_recommend_lane_list.lane_group_id = raw_recommend_lane_list.lane_group_id();
    adapted_recommend_lane_list.recommend_lane.reserve(raw_recommend_lane_list.recommend_lane().size());
    for (const auto &proto_recommend_lane : raw_recommend_lane_list.recommend_lane()) {
      cem::message::env_model::SDRecommendLGSegment adapted_recommend_lane;
      adapted_recommend_lane.start_offset = proto_recommend_lane.start_offset();
      adapted_recommend_lane.end_offset   = proto_recommend_lane.end_offset();
      adapted_recommend_lane.lane_num     = proto_recommend_lane.lane_num();
      for (const auto &raw_lane_seqs : proto_recommend_lane.lane_seqs()) {
        adapted_recommend_lane.lane_seqs.emplace_back(raw_lane_seqs);
      }
      adapted_recommend_lane_list.recommend_lane.emplace_back(std::move(adapted_recommend_lane));
    }

    adapted_recommend_lane_list.available_lane.reserve(raw_recommend_lane_list.available_lane().size());
    for (const auto &proto_available_lane : raw_recommend_lane_list.available_lane()) {
      cem::message::env_model::SDRecommendLGSegment adapted_available_lane;
      adapted_available_lane.start_offset = proto_available_lane.start_offset();
      adapted_available_lane.end_offset   = proto_available_lane.end_offset();
      adapted_available_lane.lane_num     = proto_available_lane.lane_num();
      for (const auto &raw_lane_seqs : proto_available_lane.lane_seqs()) {
        adapted_available_lane.lane_seqs.emplace_back(raw_lane_seqs);
      }
      adapted_recommend_lane_list.available_lane.emplace_back(std::move(adapted_available_lane));
    }
  }
}

// RoutingMap internal
void AdaptInput2Internal(const std::shared_ptr<byd::msg::orin::routing_map::RoutingMap> &raw_routing_map,
                         RoutingMapPtr                                                   adapted_routing_map) {
  if (raw_routing_map->has_header()) {
    adapted_routing_map->header.timestamp     = raw_routing_map->header().measurement_timestamp();
    adapted_routing_map->header.cycle_counter = raw_routing_map->header().sequence_num();
  }
  adapted_routing_map->header.recv_timestamp = GetMwTimeNowSec();

  if (raw_routing_map->has_map_info()) {
    adapted_routing_map->map_info.coord_sys      = static_cast<CoordSys>(raw_routing_map->map_info().coord_sys());
    adapted_routing_map->map_info.map_version    = raw_routing_map->map_info().map_version();
    adapted_routing_map->map_info.engine_version = raw_routing_map->map_info().engine_version();
    adapted_routing_map->map_info.map_provider =
        static_cast<MapProvider>(raw_routing_map->map_info().map_provider());
  }

  if (raw_routing_map->has_map_type()) {
    adapted_routing_map->type = static_cast<cem::message::env_model::MapType>(raw_routing_map->map_type());
  }
  if (raw_routing_map->has_cur_road_class()) {
    adapted_routing_map->cur_road_class = static_cast<cem::message::env_model::RoadClass>(raw_routing_map->cur_road_class());
  }
  if (raw_routing_map->has_is_on_highway()) {
    adapted_routing_map->is_on_highway = raw_routing_map->is_on_highway();
  }
  //LaneInfo
  adapted_routing_map->lanes.reserve(raw_routing_map->lanes().size());
  for (const auto &raw_lane : raw_routing_map->lanes()) {
    adapted_routing_map->lanes.emplace_back(LaneInfo());
    auto &adapted_lane = adapted_routing_map->lanes.back();
    AdaptInput2Internal(raw_lane, &adapted_lane);
  }

  // LaneBoundaryInfo
  adapted_routing_map->lane_boundaries.reserve(raw_routing_map->lane_boundaries().size());
  for (const auto &raw_lane_bd : raw_routing_map->lane_boundaries()) {
    adapted_routing_map->lane_boundaries.emplace_back(LaneBoundaryInfo());
    auto &adapted_lane_bd = adapted_routing_map->lane_boundaries.back();
    AdaptInput2Internal(raw_lane_bd, &adapted_lane_bd);
  }

  // RoadBoundaryInfo
  adapted_routing_map->road_boundaries.reserve(raw_routing_map->road_boundaries().size());
  for (const auto &raw_road_bd : raw_routing_map->road_boundaries()) {
    adapted_routing_map->road_boundaries.emplace_back(RoadBoundaryInfo());
    auto &adapted_road_bd = adapted_routing_map->road_boundaries.back();
    AdaptInput2Internal(raw_road_bd, &adapted_road_bd);
  }

  // StopLineInfo
  adapted_routing_map->stop_lines.reserve(raw_routing_map->stop_lines().size());
  for (const auto &raw_sline : raw_routing_map->stop_lines()) {
    adapted_routing_map->stop_lines.emplace_back(StopLineInfo());
    auto &adapted_sline = adapted_routing_map->stop_lines.back();
    AdaptInput2Internal(raw_sline, &adapted_sline);
  }

  // JunctionInfo
  adapted_routing_map->junctions.reserve(raw_routing_map->junctions().size());
  for (const auto &jc : raw_routing_map->junctions()) {
    adapted_routing_map->junctions.emplace_back(JunctionInfo());
    auto &adapted_junc = adapted_routing_map->junctions.back();
    AdaptInput2Internal(jc, &adapted_junc);
  }

  // CrossWalkInfo
  adapted_routing_map->cross_walks.reserve(raw_routing_map->cross_walks().size());
  for (const auto &cw : raw_routing_map->cross_walks()) {
    adapted_routing_map->cross_walks.emplace_back(CrossWalkInfo());
    auto &adapted_cw = adapted_routing_map->cross_walks.back();
    AdaptInput2Internal(cw, &adapted_cw);
  }

  // RouteInfo
  if (raw_routing_map->has_route()) {
    auto &adapted_route = adapted_routing_map->route;
    AdaptInput2Internal(raw_routing_map->route(), &adapted_route);
  }

  //TrafficLightInfo
  adapted_routing_map->traffic_lights.reserve(raw_routing_map->traffic_lights().size());
  for (const auto &light : raw_routing_map->traffic_lights()) {
    TrafficLightMap traffic_light;
    adapted_routing_map->traffic_lights.emplace_back(traffic_light);
    auto &adapted_light = adapted_routing_map->traffic_lights.back();
    AdaptInput2Internal(light, &adapted_light);
  }

  // 转换 EnvInfo
  if (raw_routing_map->has_env_info()) {
    AdaptInput2Internal(raw_routing_map->env_info(), &adapted_routing_map->env_info);
  }
  //SDRouteInfo
  AdaptInput2Internal(raw_routing_map->sd_route(), &adapted_routing_map->sd_route);

  // sd_lane_groups
  adapted_routing_map->sd_lane_groups.reserve(raw_routing_map->sd_lane_groups().size());
  for (const auto &raw_sd_lane_group : raw_routing_map->sd_lane_groups()) {
    adapted_routing_map->sd_lane_groups.emplace_back(SDLaneGroupInfo());
    auto &adapted_sd_lane_group = adapted_routing_map->sd_lane_groups.back();
    AdaptInput2Internal(raw_sd_lane_group, &adapted_sd_lane_group);
  }
  //exp_trajectories
  adapted_routing_map->exp_trajectories.reserve(raw_routing_map->exp_trajectories().size());
  for (const auto &raw_exp_trajectory : raw_routing_map->exp_trajectories()) {
    adapted_routing_map->exp_trajectories.emplace_back(Trajectory());
    auto &adapted_exp_trajectory = adapted_routing_map->exp_trajectories.back();
    AdaptInput2Internal(raw_exp_trajectory, &adapted_exp_trajectory);
  }
}

void AdaptInput2Internal(const byd::msg::orin::routing_map::SDLaneGroupInfo &proto_sd_lane_group,
                         cem::message::env_model::SDLaneGroupInfo           *internal_sd_lane_group) {
  internal_sd_lane_group->id       = proto_sd_lane_group.id();
  internal_sd_lane_group->length   = proto_sd_lane_group.length();
  internal_sd_lane_group->lane_num = proto_sd_lane_group.lane_num();

  // successor_lane_group_ids
  internal_sd_lane_group->successor_lane_group_ids.clear();
  for (const auto &successor_id : proto_sd_lane_group.successor_lane_group_ids()) {
    internal_sd_lane_group->successor_lane_group_ids.push_back(successor_id);
  }

  // predecessor_lane_group_ids
  internal_sd_lane_group->predecessor_lane_group_ids.clear();
  for (const auto &predecessor_id : proto_sd_lane_group.predecessor_lane_group_ids()) {
    internal_sd_lane_group->predecessor_lane_group_ids.push_back(predecessor_id);
  }

  // lane_info
  internal_sd_lane_group->lane_info.clear();
  for (const auto &raw_lane : proto_sd_lane_group.lane_info()) {
    internal_sd_lane_group->lane_info.emplace_back(LaneInfo());
    auto &adapted_lane = internal_sd_lane_group->lane_info.back();
    AdaptInput2Internal(raw_lane, &adapted_lane);
    //  plan_turn_type
    if (raw_lane.has_plan_turn_type()) {
      adapted_lane.plan_turn_type = static_cast<TurnType>(raw_lane.plan_turn_type());
    }
  }

  // recommend_lane
  internal_sd_lane_group->recommend_lane.clear();
  for (const auto &raw_recommend_lane : proto_sd_lane_group.recommend_lane()) {
    internal_sd_lane_group->recommend_lane.emplace_back(SDRecommendLGSegment());
    auto &adapted_recommend_lane        = internal_sd_lane_group->recommend_lane.back();
    adapted_recommend_lane.start_offset = raw_recommend_lane.start_offset();
    adapted_recommend_lane.end_offset   = raw_recommend_lane.end_offset();
    adapted_recommend_lane.lane_num     = raw_recommend_lane.lane_num();
    for (const auto &raw_lane_seqs : raw_recommend_lane.lane_seqs()) {
      adapted_recommend_lane.lane_seqs.emplace_back(raw_lane_seqs);
    }
  }
}

void AdaptInput2Internal(const byd::msg::orin::routing_map::Trajectory &proto_exp_trajectory,
                         cem::message::env_model::Trajectory           *internal_exp_trajectory) {
  internal_exp_trajectory->id            = proto_exp_trajectory.id();
  internal_exp_trajectory->lane_id       = proto_exp_trajectory.lane_id();
  internal_exp_trajectory->start_lane_id = proto_exp_trajectory.start_lane_id();
  internal_exp_trajectory->end_lane_id   = proto_exp_trajectory.end_lane_id();
  internal_exp_trajectory->relative_lane_id.reserve(proto_exp_trajectory.relative_lane_id().size());
  internal_exp_trajectory->relative_lane_id.assign(proto_exp_trajectory.relative_lane_id().begin(),
                                                   proto_exp_trajectory.relative_lane_id().end());

  internal_exp_trajectory->points.reserve(proto_exp_trajectory.points().size());
  for (const auto &raw_point : proto_exp_trajectory.points()) {
    internal_exp_trajectory->points.emplace_back(Point());
    auto &adapted_point = internal_exp_trajectory->points.back();
    AdaptInput2Internal(raw_point, &adapted_point);
  }
}

void AdaptInput2Internal(const std::shared_ptr<byd::msg::drivers::TrafficInfoNotify> &raw_navi_traf,
                         std::shared_ptr<cem::message::env_model::NaviTrafficInfo> &adapted_navi_traf) {
  adapted_navi_traf = std::make_shared<cem::message::env_model::NaviTrafficInfo>();
  adapted_navi_traf->header.timestamp      = raw_navi_traf->header().measurement_timestamp();
  adapted_navi_traf->header.recv_timestamp = GetMwTimeNowSec();
  adapted_navi_traf->spd_lmt_speed_value   = raw_navi_traf->spd_lmt_speed_value();
  adapted_navi_traf->road_class            = raw_navi_traf->road_class();
}

void AdaptInput2Internal(const std::shared_ptr<byd::msg::drivers::Event> &raw_drivers_event,
                         std::shared_ptr<cem::message::env_model::DriversEvent> &adapted_drivers_event) {
  adapted_drivers_event = std::make_shared<cem::message::env_model::DriversEvent>();
  adapted_drivers_event->header.timestamp = raw_drivers_event->header().measurement_timestamp();
  adapted_drivers_event->header.recv_timestamp = GetMwTimeNowSec();
  adapted_drivers_event->function_support_status = raw_drivers_event->function_support_status();
}

void AdaptInput2Internal(const std::shared_ptr<byd::msg::orin::routing_map::MapEvent> &raw_map_event,
                         std::shared_ptr<cem::message::env_model::MapEvent>           &adapted_map_event) {
  // 确保适配器的 std::shared_ptr 被正确初始化
  adapted_map_event = std::make_shared<cem::message::env_model::MapEvent>();

  // 1. 处理 Header
  adapted_map_event->header.timestamp      = raw_map_event->header().measurement_timestamp();
  adapted_map_event->header.recv_timestamp = GetMwTimeNowSec();

  // 2. 处理 ControlWayPoint 枚举字段
  adapted_map_event->control_waypoint_type = static_cast<cem::message::env_model::ControlWayPoint>(raw_map_event->control_waypoint_type());

  // 3. 处理 ControlWayPoint 距离
  adapted_map_event->control_waypoint_dis = raw_map_event->control_waypoint_dis();

  // 4. 处理 ReminderWayPoint 数组
  adapted_map_event->reminder_way_info.reserve(raw_map_event->reminder_way_info().size());
  for (const auto &raw_reminder_way_info : raw_map_event->reminder_way_info()) {
    adapted_map_event->reminder_way_info.emplace_back(cem::message::env_model::ReminderWayInfo());
    auto &adapted_reminder_way_info = adapted_map_event->reminder_way_info.back();
    adapted_reminder_way_info.reminder_waypoint_type =
        static_cast<cem::message::env_model::ReminderWayPoint>(raw_reminder_way_info.reminder_waypoint_type());
    adapted_reminder_way_info.reminder_waypoint_dis = raw_reminder_way_info.reminder_waypoint_dis();
  }

  // 5. 处理 RemainDistanceType 数组
  adapted_map_event->remain_dis_info.reserve(raw_map_event->remain_dis_info().size());
  for (const auto &raw_remain_distance_info : raw_map_event->remain_dis_info()) {
    adapted_map_event->remain_dis_info.emplace_back(cem::message::env_model::RemainDistanceInfo());
    auto &adapted_remain_distance_info = adapted_map_event->remain_dis_info.back();
    adapted_remain_distance_info.remain_dis_type =
        static_cast<cem::message::env_model::RemainType>(raw_remain_distance_info.remain_dis_type());
    adapted_remain_distance_info.remain_dis = raw_remain_distance_info.remain_dis();
  }

  // 6. 处理 NaviStatus 枚举字段
  adapted_map_event->navi_stat = static_cast<cem::message::env_model::NaviStatus>(raw_map_event->navi_stat());

  // 7. 处理 LocationQuality 枚举字段
  adapted_map_event->loc_qulity = static_cast<cem::message::env_model::LocationQuality>(raw_map_event->loc_qulity());

  // 8. 处理 OddInfo 数组
  adapted_map_event->odd_info.reserve(raw_map_event->odd_info().size());
  for (const auto &raw_odd_info : raw_map_event->odd_info()) {
    adapted_map_event->odd_info.emplace_back(cem::message::env_model::OddInfo());
    auto &adapted_odd_info        = adapted_map_event->odd_info.back();
    adapted_odd_info.section_id   = raw_odd_info.section_id();
    adapted_odd_info.lane_id      = raw_odd_info.lane_id();
    adapted_odd_info.type         = static_cast<cem::message::env_model::OddInfo_Type>(raw_odd_info.type());
    adapted_odd_info.start_offset = raw_odd_info.start_offset();
    adapted_odd_info.end_offset   = raw_odd_info.end_offset();
    adapted_odd_info.value        = static_cast<cem::message::env_model::ValueType>(raw_odd_info.value());
  }

  // 9. 处理 LaneType 和 RoadType 枚举字段
  adapted_map_event->lane_type = static_cast<cem::message::env_model::LaneType_EV>(raw_map_event->lane_type());
  adapted_map_event->road_type = static_cast<cem::message::env_model::RoadType>(raw_map_event->road_type());

  // 10. 处理 SpeedLimitInfo 数组
  adapted_map_event->speed_limit_info.reserve(raw_map_event->speed_limit_info().size());
  for (const auto &raw_speed_limit_info : raw_map_event->speed_limit_info()) {
    adapted_map_event->speed_limit_info.emplace_back(cem::message::env_model::SpeedLimitInfo());
    auto &adapted_speed_limit_info       = adapted_map_event->speed_limit_info.back();
    adapted_speed_limit_info.speed_limit = raw_speed_limit_info.speed_limit();
    adapted_speed_limit_info.offset      = raw_speed_limit_info.offset();
  }

  // 11. 处理 TrafficLightInfo
  adapted_map_event->traffic_light_info.light_state     = raw_map_event->traffic_light_info().light_state();
  adapted_map_event->traffic_light_info.light_direction = raw_map_event->traffic_light_info().light_direction();
  adapted_map_event->traffic_light_info.end_timestamp   = raw_map_event->traffic_light_info().end_timestamp();
  adapted_map_event->traffic_light_dis                  = raw_map_event->traffic_light_dis();

  // 12. 处理 NaviActionInfo 数组
  adapted_map_event->navi_action.reserve(raw_map_event->navi_action().size());
  for (const auto &raw_navi_action : raw_map_event->navi_action()) {
    adapted_map_event->navi_action.emplace_back(cem::message::env_model::NaviActionInfo());
    auto &adapted_navi_action            = adapted_map_event->navi_action.back();
    adapted_navi_action.main_action      = static_cast<cem::message::env_model::NaviMainAction>(raw_navi_action.main_action());
    adapted_navi_action.assistant_action = static_cast<cem::message::env_model::NaviAssistantAction>(raw_navi_action.assistant_action());
    adapted_navi_action.action_dis       = raw_navi_action.action_dis();
  }

  // 13. 处理 CurvatureInfo 数组
  adapted_map_event->curvature_info.reserve(raw_map_event->curvature_info().size());
  for (const auto &raw_curvature_info : raw_map_event->curvature_info()) {
    adapted_map_event->curvature_info.emplace_back(cem::message::env_model::CurvatureInfo());
    auto &adapted_curvature_info      = adapted_map_event->curvature_info.back();
    adapted_curvature_info.curvature  = raw_curvature_info.curvature();
    adapted_curvature_info.remain_dis = raw_curvature_info.remain_dis();
  }

  // 14. 处理 TrafficInfoNotify
  if (raw_map_event->has_traffic_info()) {
    const auto &raw_traffic_info                              = raw_map_event->traffic_info();
    adapted_map_event->traffic_info.header.timestamp          = raw_traffic_info.header().measurement_timestamp();
    adapted_map_event->traffic_info.header.recv_timestamp     = GetMwTimeNowSec();
    adapted_map_event->traffic_info.traffic_jam_dist          = raw_traffic_info.traffic_jam_dist();
    adapted_map_event->traffic_info.dist_to_start_traffic_jam = raw_traffic_info.dist_to_start_traffic_jam();
    adapted_map_event->traffic_info.traffic_jam_status        = raw_traffic_info.traffic_jam_status();
    adapted_map_event->traffic_info.pass_time                 = raw_traffic_info.pass_time();
  }
}

void AdaptInput2Internal(const std::shared_ptr<byd::msg::drivers::Ins> &raw_loc, LocalizationPtr adapted_loc) {
  //! just adapt what we need
  adapted_loc->header.timestamp      = raw_loc->header().measurement_timestamp();
  adapted_loc->header.recv_timestamp = GetMwTimeNowSec();
  adapted_loc->status                = 0;  // 0-done, 1-imu, 2-vcu, 3-re-init

  const auto &quat            = raw_loc->orientation();
  adapted_loc->orientation.qx = float(quat.qx());
  adapted_loc->orientation.qy = float(quat.qy());
  adapted_loc->orientation.qz = float(quat.qz());
  adapted_loc->orientation.qw = float(quat.qw());

  adapted_loc->attitude_dr =
      std::atan2(2.0f * (quat.qw() * quat.qz() + quat.qx() * quat.qy()), 1.0f - 2.0f * (quat.qy() * quat.qy() + quat.qz() * quat.qz())) *
      180.0f / M_PI;

  adapted_loc->posne_dr.reserve(2);
  adapted_loc->posne_dr.emplace_back(raw_loc->position().x());
  adapted_loc->posne_dr.emplace_back(raw_loc->position().y());
}

void AdaptInput2Internal(const std::shared_ptr<byd::msg::localization::LocalizationEstimate> &raw_loc, LocalizationPtr adapted_loc) {
  //! just adapt what we need
  adapted_loc->header.timestamp      = raw_loc->header().measurement_timestamp();
  adapted_loc->header.recv_timestamp = GetMwTimeNowSec();
  adapted_loc->status                = 0;  // 0-done, 1-imu, 2-vcu, 3-re-init

  const auto &quat            = raw_loc->pose().orientation();
  adapted_loc->orientation.qx = float(quat.qx());
  adapted_loc->orientation.qy = float(quat.qy());
  adapted_loc->orientation.qz = float(quat.qz());
  adapted_loc->orientation.qw = float(quat.qw());

  // adapted_loc->attitude_dr =
  //     std::atan2(2.0f * (quat.qw() * quat.qz() + quat.qx() * quat.qy()),
  //                1.0f - 2.0f * (quat.qy() * quat.qy() + quat.qz() * quat.qz())) *
  //     180.0f / M_PI;
  adapted_loc->attitude_dr = raw_loc->pose().heading();
  if (raw_loc->pose().has_linear_velocity()) {
    adapted_loc->velocity.push_back(raw_loc->pose().linear_velocity().x());
    adapted_loc->velocity.push_back(raw_loc->pose().linear_velocity().y());
    adapted_loc->velocity.push_back(raw_loc->pose().linear_velocity().z());
  }

  adapted_loc->posne_dr.reserve(2);
  adapted_loc->posne_dr.emplace_back(raw_loc->pose().position().x());
  adapted_loc->posne_dr.emplace_back(raw_loc->pose().position().y());
}

void AdaptInput2Internal(const std::shared_ptr<byd::msg::drivers::VehInfo> &veh_info_msg_ptr, Bcm50msPdu02Ptr adapted_bcm50ms_pdu02) {
  if (1u == veh_info_msg_ptr->body_system_status().da_in_turnindsts_u8()) {
    adapted_bcm50ms_pdu02->ildircnio = 1;
    adapted_bcm50ms_pdu02->irdircnio = 0;
  } else if (2u == veh_info_msg_ptr->body_system_status().da_in_turnindsts_u8()) {
    adapted_bcm50ms_pdu02->ildircnio = 0;
    adapted_bcm50ms_pdu02->irdircnio = 1;
  } else {
    adapted_bcm50ms_pdu02->ildircnio = 0;
    adapted_bcm50ms_pdu02->irdircnio = 0;
  }

  if (1u == veh_info_msg_ptr->body_system_status().da_in_frtwipersts_u8()) {
    adapted_bcm50ms_pdu02->ifrtwiperswsts = 2;
  } else if (2u == veh_info_msg_ptr->body_system_status().da_in_frtwipersts_u8()) {
    adapted_bcm50ms_pdu02->ifrtwiperswsts = 3;
  } else {
    adapted_bcm50ms_pdu02->ifrtwiperswsts = 0;
  }
  //set default value
  adapted_bcm50ms_pdu02->header.recv_timestamp = GetMwTimeNowSec();
  adapted_bcm50ms_pdu02->idaytimerunninglampon = 0;
  adapted_bcm50ms_pdu02->idipdbeamlghton       = 0;
  adapted_bcm50ms_pdu02->idircnindlampswsts    = 0;
  adapted_bcm50ms_pdu02->ifrtfoglghton         = 0;
  adapted_bcm50ms_pdu02->ifrtwiperparkposa     = 0;
  adapted_bcm50ms_pdu02->ifrtwshrpumpa         = 0;
  adapted_bcm50ms_pdu02->ikeydetindx           = 0;
  adapted_bcm50ms_pdu02->ildircnindlghtf       = 0;
  adapted_bcm50ms_pdu02->imainbeamlghton       = 0;
  adapted_bcm50ms_pdu02->irdircnindlghtf       = 0;
  adapted_bcm50ms_pdu02->irrfoglghton          = 0;
  adapted_bcm50ms_pdu02->ivehsidelghtsts       = 0;
  adapted_bcm50ms_pdu02->e2e_status            = 0;
}

void AdaptInput2Internal(const std::shared_ptr<byd::msg::drivers::VehInfo> &veh_info_msg_ptr, Ibs20msPdu08Ptr adapted_ibs20ms_pdu08) {
  adapted_ibs20ms_pdu08->ivehdynyawrate     = veh_info_msg_ptr->ego_motion_status().da_in_yawrate_sg() * 57.3;
  adapted_ibs20ms_pdu08->ivselongtacc       = veh_info_msg_ptr->ego_motion_status().da_in_lgtacc_sg();
  adapted_ibs20ms_pdu08->ivselatacc         = veh_info_msg_ptr->ego_motion_status().da_in_latacc_sg();
  adapted_ibs20ms_pdu08->soc_recv_timestamp = GetMwTimeNowSec();

  // set default value
  adapted_ibs20ms_pdu08->header.recv_timestamp = GetMwTimeNowSec();
  adapted_ibs20ms_pdu08->ivehdynyawratev       = 0;
  adapted_ibs20ms_pdu08->ivselatacccrc         = 0;
  adapted_ibs20ms_pdu08->ivselataccrc          = 0;
  adapted_ibs20ms_pdu08->ivselataccv           = 0;
  adapted_ibs20ms_pdu08->ivselongtaccv         = 0;
  adapted_ibs20ms_pdu08->e2e_status            = 0;
}

void AdaptInput2Internal(const std::shared_ptr<byd::msg::drivers::VehInfo> &veh_info_msg_ptr, Imcu20msPdu05Ptr adapted_imcu20ms_pdu05) {
  adapted_imcu20ms_pdu05->header.recv_timestamp         = GetMwTimeNowSec();
  adapted_imcu20ms_pdu05->e2e_status                    = 0;
  adapted_imcu20ms_pdu05->ieptrdy                       = 0;
  adapted_imcu20ms_pdu05->iimcu_025ms_group05_crc       = 0;
  adapted_imcu20ms_pdu05->iimcu_025ms_group05_rc        = 0;
  adapted_imcu20ms_pdu05->iimcu_025ms_group05_reserve01 = 0;
  adapted_imcu20ms_pdu05->iimcu_025ms_group05_reserve02 = 0;
  adapted_imcu20ms_pdu05->iimcu_025ms_group05_reserve03 = 0;
  adapted_imcu20ms_pdu05->iimcu_025ms_group05_reserve04 = 0;
  adapted_imcu20ms_pdu05->itrestdgear                   = 0;
  if (0u == veh_info_msg_ptr->propulsion_system_status().da_in_trmgearlvl_u8()) {
    adapted_imcu20ms_pdu05->itrshftlvrpos = 0;
  } else if (1u == veh_info_msg_ptr->propulsion_system_status().da_in_trmgearlvl_u8()) {
    adapted_imcu20ms_pdu05->itrshftlvrpos = 1;
  } else if (2u == veh_info_msg_ptr->propulsion_system_status().da_in_trmgearlvl_u8()) {
    adapted_imcu20ms_pdu05->itrshftlvrpos = 3;
  } else if (3u == veh_info_msg_ptr->propulsion_system_status().da_in_trmgearlvl_u8()) {
    adapted_imcu20ms_pdu05->itrshftlvrpos = 4;
  } else if (4u == veh_info_msg_ptr->propulsion_system_status().da_in_trmgearlvl_u8()) {
    adapted_imcu20ms_pdu05->itrshftlvrpos = 2;
  } else {
    adapted_imcu20ms_pdu05->itrshftlvrpos = 0;
  }
  adapted_imcu20ms_pdu05->itrshftlvrposv = 0;
}

void AdaptInput2Internal(const std::shared_ptr<byd::msg::drivers::VehInfo> &veh_info_msg_ptr, Eps010msPdu00Ptr adapted_eps010ms_pdu00) {
  adapted_eps010ms_pdu00->header.recv_timestamp        = GetMwTimeNowSec();
  adapted_eps010ms_pdu00->iadasstrgdlvrdtoq            = 0.0;
  adapted_eps010ms_pdu00->iadasstrgdlvrdtoqv           = 0;
  adapted_eps010ms_pdu00->iactupnonang                 = 0.0;
  adapted_eps010ms_pdu00->iactupnonangv                = 0;
  adapted_eps010ms_pdu00->iacturoadwhlang              = 0.0;
  adapted_eps010ms_pdu00->iacturoadwhlangv             = 0;
  adapted_eps010ms_pdu00->ichlkahptresp                = 0;
  adapted_eps010ms_pdu00->ichlkaangreqresp             = 0;
  adapted_eps010ms_pdu00->ichlkaangreqresp_hf          = 0;
  adapted_eps010ms_pdu00->ichlkatoqreqresp             = 0;
  adapted_eps010ms_pdu00->ichlkatoqreqresp_hf          = 0;
  adapted_eps010ms_pdu00->idrvrstrgdlvrdtoq            = veh_info_msg_ptr->steering_system_status().da_in_drvrstrtoq_sg();
  adapted_eps010ms_pdu00->idrvrstrgdlvrdtoqv           = 0;
  adapted_eps010ms_pdu00->iepscurntdrvmd               = 0;
  adapted_eps010ms_pdu00->iepsdrvngmdctrlinh           = 0;
  adapted_eps010ms_pdu00->iepsflrsts                   = 0;
  adapted_eps010ms_pdu00->iepshflmtrsts                = 0;
  adapted_eps010ms_pdu00->iepshfmontngresultsts        = 0;
  adapted_eps010ms_pdu00->iepspriychnlflrsts           = 0;
  adapted_eps010ms_pdu00->iepspriystsalvrc             = 0;
  adapted_eps010ms_pdu00->iepspriystschksm             = 0;
  adapted_eps010ms_pdu00->iepstoqvalaldmax             = 0.0;
  adapted_eps010ms_pdu00->iepstoqvalaldmaxv            = 0;
  adapted_eps010ms_pdu00->iepstoqvalaldmin             = 0.0;
  adapted_eps010ms_pdu00->iepstoqvalaldminv            = 0;
  adapted_eps010ms_pdu00->ieps_010ms_group00_reserve01 = 0;
  adapted_eps010ms_pdu00->ieps_010ms_group00_reserve03 = 0;
  adapted_eps010ms_pdu00->ieps_010ms_group00_reserve04 = 0;
  adapted_eps010ms_pdu00->istrgcustsetngdspcmd         = 0;
  adapted_eps010ms_pdu00->e2e_status                   = 0;
}

void AdaptInput2Internal(const std::shared_ptr<byd::msg::drivers::VehInfo> &veh_info_msg_ptr, Ibs20msPdu15Ptr adapted_ibs20ms_pdu15) {
  adapted_ibs20ms_pdu15->header.recv_timestamp       = GetMwTimeNowSec();
  adapted_ibs20ms_pdu15->iibs_20ms_group15_reserve01 = 0;
  adapted_ibs20ms_pdu15->ivehspdavg                  = veh_info_msg_ptr->ego_motion_status().da_in_vehspd_sg() * 3.6;
  adapted_ibs20ms_pdu15->ivehspdavgalvrc             = 0;
  adapted_ibs20ms_pdu15->ivehspdavgcrc               = 0;
  adapted_ibs20ms_pdu15->ivehspdavgdrvn              = 0.0;
  adapted_ibs20ms_pdu15->ivehspdavgdrvnsrc           = 0;
  adapted_ibs20ms_pdu15->ivehspdavgdrvnv             = 0;
  adapted_ibs20ms_pdu15->ivehspdavgnondrvn           = 0.0;
  adapted_ibs20ms_pdu15->ivehspdavgnondrvnv          = 0;
  adapted_ibs20ms_pdu15->ivehspdavgv                 = 0;
  adapted_ibs20ms_pdu15->timestamp                   = veh_info_msg_ptr->header().publish_timestamp();
  adapted_ibs20ms_pdu15->e2e_status                  = 0;
}

void AdaptInput2Internal(const std::shared_ptr<byd::msg::drivers::VehInfo> &veh_info_msg_ptr, Sas10msPdu00Ptr adapted_sas10ms_pdu00) {
  adapted_sas10ms_pdu00->header.recv_timestamp        = GetMwTimeNowSec();
  adapted_sas10ms_pdu00->isas_010ms_group00_reserve01 = 0;
  adapted_sas10ms_pdu00->isas_010ms_group00_reserve02 = 0;
  adapted_sas10ms_pdu00->istrgwhlang                  = veh_info_msg_ptr->steering_system_status().da_in_strgwhlang_sg() * 57.3 / 13.1;
  adapted_sas10ms_pdu00->istrgwhlangalvrc             = 0;
  adapted_sas10ms_pdu00->istrgwhlanggrd               = 0;
  adapted_sas10ms_pdu00->istrgwhlangsnsrcrc           = 0;
  adapted_sas10ms_pdu00->istrgwhlangsnsrcalsts        = 0;
  adapted_sas10ms_pdu00->istrgwhlangsnsrflt           = 0;
  adapted_sas10ms_pdu00->istrgwhlangsnsrinid          = 0;
  adapted_sas10ms_pdu00->istrgwhlangsnsrmultcapb      = 0;
  adapted_sas10ms_pdu00->istrgwhlangv                 = 0;
  adapted_sas10ms_pdu00->timestamp                    = veh_info_msg_ptr->header().publish_timestamp();
  // veh_info_msg_ptr->header().publish_timestamp();
  adapted_sas10ms_pdu00->e2e_status = 0;
}

void AdaptInput2Internal(const std::shared_ptr<byd::msg::drivers::VehInfo> &veh_info_msg_ptr,
                         AdasIpd100msPdu12Ptr                               adapted_adas_ipd100ms_pdu12) {
  adapted_adas_ipd100ms_pdu12->header.timestamp             = veh_info_msg_ptr->header().measurement_timestamp();
  adapted_adas_ipd100ms_pdu12->header.recv_timestamp        = GetMwTimeNowSec();
  adapted_adas_ipd100ms_pdu12->iautomainbeamlghtreq         = 2;
  adapted_adas_ipd100ms_pdu12->idistsincetrgtcamr           = 0;  //todo
  adapted_adas_ipd100ms_pdu12->ifvcmblkd                    = 0;
  adapted_adas_ipd100ms_pdu12->ifvcmcalprgsreq              = 0;
  adapted_adas_ipd100ms_pdu12->ifvcmfltsts                  = 0;
  adapted_adas_ipd100ms_pdu12->ifvcmspdlmtvalsts            = 0;
  adapted_adas_ipd100ms_pdu12->iipd_100ms_group12_crc       = 0;
  adapted_adas_ipd100ms_pdu12->iipd_100ms_group12_rc        = 0;
  adapted_adas_ipd100ms_pdu12->iipd_100ms_group12_reserve01 = 0;
  adapted_adas_ipd100ms_pdu12->iipd_100ms_group12_reserve02 = 0;
  adapted_adas_ipd100ms_pdu12->ispdastcndstscamr            = 0;  //todo
  adapted_adas_ipd100ms_pdu12->ispdastreqstscamr            = 1;
  adapted_adas_ipd100ms_pdu12->ispdlmtmdlsts                = 0;
  adapted_adas_ipd100ms_pdu12->ispdlmtmdresp                = 0;
  adapted_adas_ipd100ms_pdu12->itrgtspdreqcamr              = 1;
  adapted_adas_ipd100ms_pdu12->e2e_status                   = 0;
}

void AdaptInput2Internal(const std::shared_ptr<byd::msg::drivers::VehInfo> &veh_info_msg_ptr, Mpd100msPdu04Ptr adapted_mpd100ms_pdu04) {
  adapted_mpd100ms_pdu04->header.timestamp          = veh_info_msg_ptr->header().measurement_timestamp();
  adapted_mpd100ms_pdu04->header.recv_timestamp     = GetMwTimeNowSec();
  adapted_mpd100ms_pdu04->e2e_status                = 0;
  adapted_mpd100ms_pdu04->iautolanechngmsgreq       = 0;
  adapted_mpd100ms_pdu04->iautolanechngsetresp      = 0;
  adapted_mpd100ms_pdu04->iessswdspcmd              = 0;
  adapted_mpd100ms_pdu04->ihfrmndrswdspcmd          = 0;
  adapted_mpd100ms_pdu04->ijafnspgoswdspcmd         = 0;
  adapted_mpd100ms_pdu04->ijamsgreq                 = 0;
  adapted_mpd100ms_pdu04->ilcrinhmi                 = 0;
  adapted_mpd100ms_pdu04->ilcrmsgreq                = 0;
  adapted_mpd100ms_pdu04->ilcrswdspcmd              = 0;
  adapted_mpd100ms_pdu04->inopcityswdspcmd          = 0;
  adapted_mpd100ms_pdu04->inopswdspcmd              = 2;
  adapted_mpd100ms_pdu04->inopsysmsg                = 0;
  adapted_mpd100ms_pdu04->irpilotecomdstsdspcmd     = 0;
  adapted_mpd100ms_pdu04->irpilotecomdswavlbldspcmd = 0;
  adapted_mpd100ms_pdu04->itrfclghtfnswdspcmd       = 0;
}

void AdaptInput2Internal(const std::shared_ptr<byd::msg::drivers::VehInfo> &veh_info_msg_ptr, Icm100msPdu29Ptr adapted_apicm100mspdu29) {
  adapted_apicm100mspdu29->header.timestamp        = veh_info_msg_ptr->header().measurement_timestamp();
  adapted_apicm100mspdu29->header.recv_timestamp   = GetMwTimeNowSec();
  adapted_apicm100mspdu29->e2e_status              = 0;
  adapted_apicm100mspdu29->ificmposngsysdircn      = 0;
  adapted_apicm100mspdu29->inavcountrycode         = 0;
  adapted_apicm100mspdu29->inavcrntroadtyp         = 0;
  adapted_apicm100mspdu29->inavdistfromfrtelecceye = 0;
  adapted_apicm100mspdu29->inavnewicon             = 0;
  adapted_apicm100mspdu29->inavnewicondist         = 0;
  adapted_apicm100mspdu29->inavspdlmttyp           = 0;
  adapted_apicm100mspdu29->inavspdlmtval           = 0;
  adapted_apicm100mspdu29->inavspdlmtvalsts        = 1;
  adapted_apicm100mspdu29->inavspdlmtvaltyp        = 0;
  adapted_apicm100mspdu29->inavspdlmtvalunit       = 2;
}

void AdaptInput2Internal(const std::shared_ptr<byd::msg::env_model::LocalMapInfo> &raw_local_map_info, BevMapInfoPtr adapted_bev_lane) {
  adapted_bev_lane->header.timestamp      = raw_local_map_info->header().measurement_timestamp();
  adapted_bev_lane->header.recv_timestamp = GetMwTimeNowSec();
  adapted_bev_lane->header.cycle_counter  = raw_local_map_info->header().sequence_num();
  adapted_bev_lane->is_local_pose         = false;
  // AINFO << "*******VisibilityInfoInPut******";
  // AINFO << "timestamp: " << std::fixed << adapted_bev_lane->header.timestamp;

  if (raw_local_map_info->roads().size() > 0) {
    adapted_bev_lane->frame_id = raw_local_map_info->roads().at(0).id();
  } else {
    adapted_bev_lane->frame_id = 0;
  }

  Eigen::Isometry3d T_local_ego = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_ego_local = Eigen::Isometry3d::Identity();
#ifdef BYD_X2B
  LocalizationPtr odom_now_ptr{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(adapted_bev_lane->header.timestamp, 0.05, odom_now_ptr);
  if (odom_now_ptr == nullptr) {
    return;
  }
  T_local_ego.translation() = Eigen::Vector3d(odom_now_ptr->posne_dr.at(0), odom_now_ptr->posne_dr.at(1), 0);
  T_local_ego.linear()      = Eigen::AngleAxisd(odom_now_ptr->attitude_dr * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T_ego_local               = T_local_ego.inverse();
#endif

  adapted_bev_lane->lanemarkers.reserve(raw_local_map_info->lane_markers().size());
  adapted_bev_lane->edges.reserve(raw_local_map_info->road_edges().size());

  auto AdaptInput_LaneMarker = [](const byd::msg::env_model::LocalMapInfo::LaneMarker &raw_lanemarker, BevLaneMarker *adapted_lane_marker,
                                  const Eigen::Isometry3d &T_ego_local) -> void {
    adapted_lane_marker->id               = raw_lanemarker.track_id();
    adapted_lane_marker->number_of_points = raw_lanemarker.polyline().size();

    //pos mapping
    auto it = EnumMap2LaneMarkerPosition.find(raw_lanemarker.location());
    if (it != EnumMap2LaneMarkerPosition.end()) {
      // The key exists in the map
      adapted_lane_marker->position = (int)it->second;
    } else {
      adapted_lane_marker->position = 0;
    }

    if (EnumMap2LaneMarkerType.count(raw_lanemarker.type())) {
      adapted_lane_marker->type = EnumMap2LaneMarkerType.at(raw_lanemarker.type());
    } else {
      adapted_lane_marker->type = BevLaneMarkerType::BEV_LMT__UNDECIDED;
    }

    //color mapping
    auto it2 = EnumMap2LaneMarkerColor.find(raw_lanemarker.color());
    if (it2 != EnumMap2LaneMarkerColor.end()) {
      // The key exists in the map
      adapted_lane_marker->color = (int)it2->second;
    } else {
      adapted_lane_marker->color = 2;
    }

    adapted_lane_marker->conf = raw_lanemarker.quality();
    adapted_lane_marker->line_points.reserve(raw_lanemarker.polyline().size());
    std::vector<uint32_t> idx_list;
    uint32_t              idx = 0;
    for (const auto &point : raw_lanemarker.polyline()) {
      adapted_lane_marker->line_points.emplace_back(Point2DF());
      auto           &adapted_point = adapted_lane_marker->line_points.back();
      Eigen::Vector3d point_3d(point.x(), point.y(), 0);
#ifdef BYD_X2B
      point_3d = T_ego_local * point_3d;
#endif
      adapted_point.x            = point_3d.x();
      adapted_point.y            = point_3d.y();
      adapted_point.point_source = cem::message::common::PointSource::PS_BEV;
      idx_list.push_back(idx);
      idx++;
      adapted_lane_marker->geos->emplace_back(adapted_point.x, adapted_point.y);
    }
    adapted_lane_marker->indexed_geos.BuildIndex(adapted_lane_marker->geos);

    for (const auto &tmp_type_seg : raw_lanemarker.type_seg()) {
      cem::message::sensor::BevLmTypeSeg tmp     = {BevLaneMarkerType::BEV_LMT__UNDECIDED, 0, 0};
      auto                               it_type = EnumMap2LaneMarkerType.find(tmp_type_seg.type());
      if (it_type != EnumMap2LaneMarkerType.end()) {
        // The key exists in the map
        tmp.type = static_cast<cem::message::sensor::BevLaneMarkerType>(it_type->second);
      } else {
        tmp.type = BevLaneMarkerType::BEV_LMT__UNDECIDED;
      }
      tmp.start_index = tmp_type_seg.start_index();
      tmp.end_index   = tmp_type_seg.end_index();
      adapted_lane_marker->type_segs.push_back(tmp);
    }

    for (const auto &tmp_color_seg : raw_lanemarker.color_seg()) {
      cem::message::sensor::BevLmColorSeg tmp      = {BevLaneMarkerColor::BEV_LMC__UNKNOWN, 0, 0};
      auto                                it_color = EnumMap2LaneMarkerColor.find(tmp_color_seg.color());
      if (it_color != EnumMap2LaneMarkerColor.end()) {
        // The key exists in the map
        tmp.color = static_cast<cem::message::sensor::BevLaneMarkerColor>(it_color->second);
      } else {
        tmp.color = BevLaneMarkerColor::BEV_LMC__UNKNOWN;
      }
      tmp.start_index = tmp_color_seg.start_index();
      tmp.end_index   = tmp_color_seg.end_index();
      adapted_lane_marker->color_segs.push_back(tmp);
    }

    MergeSegs(adapted_lane_marker);

    return;
  };

  for (const auto &lane_marker : raw_local_map_info->lane_markers()) {
    if (lane_marker.type() == byd::msg::env_model::LaneMarkerType::RM_TYPE_CROSSWALK) {
      adapted_bev_lane->crosswalks.emplace_back(BevLaneMarker());
      auto &adapted_crosswalk = adapted_bev_lane->crosswalks.back();
      AdaptInput_LaneMarker(lane_marker, &adapted_crosswalk, T_ego_local);
    } else if (lane_marker.type() == byd::msg::env_model::LaneMarkerType::RM_TYPE_STOPLINE) {
      adapted_bev_lane->stop_lines.emplace_back(BevLaneMarker());
      auto &adapted_stopline = adapted_bev_lane->stop_lines.back();
      AdaptInput_LaneMarker(lane_marker, &adapted_stopline, T_ego_local);
    } else if (lane_marker.type() >= byd::msg::env_model::LaneMarkerType::RM_TYPE_STRAIGHT &&
               lane_marker.type() <= byd::msg::env_model::LaneMarkerType::RM_TYPE_LEFT_TURNING) {
      adapted_bev_lane->arrows.emplace_back(BevLaneMarker());
      auto &adapted_arrow = adapted_bev_lane->arrows.back();
      AdaptInput_LaneMarker(lane_marker, &adapted_arrow, T_ego_local);
    } else if (lane_marker.type() == byd::msg::env_model::LaneMarkerType::RM_TYPE_INTERSECTION_ZONE) {
      adapted_bev_lane->junctions.emplace_back(BevLaneMarker());
      auto &adapted_junction = adapted_bev_lane->junctions.back();
      AdaptInput_LaneMarker(lane_marker, &adapted_junction, T_ego_local);
    } else if (lane_marker.type() == byd::msg::env_model::LaneMarkerType::RM_TYPE_DIVERSION_ZONE) {
      adapted_bev_lane->diversion_zone.emplace_back(BevLaneMarker());
      auto &adapted_diversion_zone = adapted_bev_lane->diversion_zone.back();
      AdaptInput_LaneMarker(lane_marker, &adapted_diversion_zone, T_ego_local);
      adapted_diversion_zone.id = adapted_diversion_zone.id + 100;
    } else {
      adapted_bev_lane->lanemarkers.emplace_back(BevLaneMarker());
      auto &adapted_lane_marker = adapted_bev_lane->lanemarkers.back();
      AdaptInput_LaneMarker(lane_marker, &adapted_lane_marker, T_ego_local);
    }
  }

  auto AdaptInput_Edge = [](const byd::msg::env_model::LocalMapInfo::RoadEdge &raw_lane, BevLaneMarker *adapted_lane,
                            const Eigen::Isometry3d &T_ego_local) -> void {
    adapted_lane->id               = raw_lane.track_id();
    adapted_lane->number_of_points = raw_lane.polyline().size();

    // pos mapping

    auto it = EnumMap2RoadEdgePosition.find(raw_lane.location());
    if (it != EnumMap2RoadEdgePosition.end()) {
      // The key exists in the map
      adapted_lane->position = (int)it->second;
    } else {
      adapted_lane->position = 0;
    }

    // bev road edge pos mapping
    auto it_bevedgepos = EnumMap2BevRoadEdgePosition.find(raw_lane.location());
    if (it_bevedgepos != EnumMap2BevRoadEdgePosition.end()) {
      // The key exists in the map
      adapted_lane->road_edeg_pos = it_bevedgepos->second;
    } else {
      adapted_lane->road_edeg_pos = BevRoadEdgePosition::BEV_REP__UNKNOWN;
    }

    // edge_type mapping
    // before cem using bev_lane type struct as the edge type
    auto it3 = EnumMap2RoadEdgeType.find(raw_lane.type());
    if (it3 != EnumMap2RoadEdgeType.end()) {
      // The key exists in the map
      adapted_lane->type = (int)it3->second;
    } else {
      adapted_lane->type = 0;
    }

    // adapted_lane->color = raw_lane.color();
    adapted_lane->conf = raw_lane.quality();
    adapted_lane->line_points.reserve(raw_lane.polyline().size());
    for (int idx = 0; idx < raw_lane.polyline().size(); idx++) {
      adapted_lane->line_points.emplace_back(Point2DF());
      auto           &adapted_point = adapted_lane->line_points.back();
      const auto     &point         = raw_lane.polyline()[idx];
      Eigen::Vector3d point_3d(point.x(), point.y(), 0);
#ifdef BYD_X2B
      point_3d = T_ego_local * point_3d;
#endif
      adapted_point.x   = point_3d.x();
      adapted_point.y   = point_3d.y();
      adapted_point.mse = raw_lane.mse()[idx];
      adapted_lane->geos->emplace_back(adapted_point.x, adapted_point.y);
    }
    adapted_lane->indexed_geos.BuildIndex(adapted_lane->geos);
    return;
  };

  for (const auto &edge : raw_local_map_info->road_edges()) {
    adapted_bev_lane->edges.emplace_back(BevLaneMarker());
    auto &adapted_edge = adapted_bev_lane->edges.back();
    AdaptInput_Edge(edge, &adapted_edge, T_ego_local);
  }

  auto AdaptInput_LaneInfo = [](const byd::msg::env_model::LocalMapInfo::Lane &raw_lane_info, BevLaneInfo *adapted_lane_info,
                                const Eigen::Isometry3d &T_ego_local) {
    adapted_lane_info->id               = raw_lane_info.track_id();
    adapted_lane_info->number_of_points = raw_lane_info.polyline().size();

    int mseIndex = 0;
    if (raw_lane_info.mse().size() != raw_lane_info.polyline().size()) {
      return;
    }

    int forward_count    = 0;
    int horizontal_count = 0;
    for (int i = 0; i < raw_lane_info.polyline().size(); i++) {
      const auto &point = raw_lane_info.polyline()[i];
      adapted_lane_info->line_points.emplace_back(Point2DF());
      auto           &adapted_point = adapted_lane_info->line_points.back();
      Eigen::Vector3d point_3d(point.x(), point.y(), 0);
#ifdef BYD_X2B
      point_3d = T_ego_local * point_3d;
#endif
      adapted_point.x = point_3d.x();
      adapted_point.y = point_3d.y();
      if (!raw_lane_info.scatter_visibility().empty()) {
        adapted_point.scatter_visibility = raw_lane_info.scatter_visibility()[i];
        // if (!adapted_point.scatter_visibility){
        //   AINFO << "input point: (" << adapted_point.x << ", " << adapted_point.y << "); " << adapted_lane_info->id;
        //   AINFO << "************************";
        // }
      }
      adapted_point.point_source = cem::message::common::PointSource::PS_BEV;

      adapted_point.mse = raw_lane_info.mse()[mseIndex];
      mseIndex++;

      if (!adapted_lane_info->geos->empty()) {
        if (fabs(adapted_point.x - adapted_lane_info->geos->back().x()) >= fabs(adapted_point.y - adapted_lane_info->geos->back().y())) {
          forward_count++;
        } else {
          horizontal_count++;
        }
      }
      adapted_lane_info->geos->emplace_back(adapted_point.x, adapted_point.y);
    }
    adapted_lane_info->indexed_geos.BuildIndex(adapted_lane_info->geos);
    if (horizontal_count > forward_count && horizontal_count > 3) {
      adapted_lane_info->is_horizontal_lane = true;
    }

    ///BevLaneInfo所有的枚举表保持和传入msg的枚举表保持一致，直接传递数值即可
    adapted_lane_info->lane_type            = static_cast<BevLaneType>(raw_lane_info.type());
    adapted_lane_info->direction            = static_cast<BevLaneDirection>(raw_lane_info.direction());
    adapted_lane_info->connect_type         = static_cast<BevLaneConnectType>(raw_lane_info.connect_type());
    adapted_lane_info->intersection_type    = static_cast<BevLaneIntersectionType>(raw_lane_info.intersection_type());
    adapted_lane_info->left_lane_marker_id  = raw_lane_info.left_lanemark_id();
    adapted_lane_info->right_lane_marker_id = raw_lane_info.right_lanemark_id();
    adapted_lane_info->position             = raw_lane_info.location();
    adapted_lane_info->width                = raw_lane_info.width();
    adapted_lane_info->connect_score        = raw_lane_info.connect_score();
    adapted_lane_info->is_blocked           = raw_lane_info.is_blocked();

    for (auto &next_lane_id : raw_lane_info.next_lane_id()) {
      adapted_lane_info->next_lane_ids.emplace_back(next_lane_id);
    }
    for (auto &prev_lane_id : raw_lane_info.previous_lane_id()) {
      adapted_lane_info->previous_lane_ids.emplace_back(prev_lane_id);
    }
    adapted_lane_info->road_id            = raw_lane_info.road_id();
    adapted_lane_info->trafficlight_state = static_cast<BevTrafficLightState>(raw_lane_info.trafficlight_state());
  };

  for (auto &raw_lane_info : raw_local_map_info->lanes()) {
    if (raw_lane_info.polyline().size() < 2) {
      continue;
    }
    adapted_bev_lane->lane_infos.emplace_back(BevLaneInfo());
    auto &adapted_lane_info = adapted_bev_lane->lane_infos.back();
    AdaptInput_LaneInfo(raw_lane_info, &adapted_lane_info, T_ego_local);
  }
}

void AdaptInput2Internal(
    const std::shared_ptr<byd::modules::localization::MapMatchResult>
        &raw_map_match_result,
    std::shared_ptr<cem::message::sensor::MapMatchResultBaidu>
        adapted_map_match_result) {
  if (raw_map_match_result->has_header()) {
    adapted_map_match_result->header.sequence_num =
        raw_map_match_result->header().sequence_num();
    adapted_map_match_result->header.timestamp =
        raw_map_match_result->header().measurement_timestamp();
    adapted_map_match_result->header.recv_timestamp = GetMwTimeNowSec();
  }
  if (raw_map_match_result->has_location_level()) {
    adapted_map_match_result->location_level =
        static_cast<cem::message::sensor::LocationLevel>(
            raw_map_match_result->location_level());
  }
}

void MergeSegs(BevLaneMarker *adapted_lane_marker) {
  std::vector<BevLmTypeSeg>    type_segs   = adapted_lane_marker->type_segs;
  std::vector<BevLmColorSeg>   color_segs  = adapted_lane_marker->color_segs;
  std::vector<BevLmMergedSeg> &merged_segs = adapted_lane_marker->merged_segs;
  int                          n = type_segs.size(), m = color_segs.size();

  BevLmMergedSeg tmp;
  tmp.start_index  = 0;
  tmp.end_index    = 0;
  tmp.type         = BevLaneMarkerType::BEV_LMT__UNDECIDED;
  tmp.color        = BevLaneMarkerColor::BEV_LMC__UNKNOWN;
  tmp.start_offset = 0.0;
  tmp.end_offset   = 0.0;
  // int start = 0, end = 0;
  // int type = -1, color = -1;

  // 双指针遍历两个数组
  for (int i = 0, j = 0; i < n || j < m;) {
    if (i < n && (j >= m || type_segs[i].start_index <= color_segs[j].start_index)) {
      // 当前区间来自 type_segs
      if (((tmp.end_index + 1 <= color_segs[j].end_index) || (tmp.end_index + 1 <= type_segs[i].end_index)) && (i > 0 || j > 0)) {
        tmp.start_index = tmp.end_index + 1;
        // AERROR<<"lxy_test if  tmp.start_index:"<<tmp.start_index;
      }
      tmp.end_index = type_segs[i].end_index;
      tmp.type      = type_segs[i].type;

      // 检查与 color_segs 的交叉
      if (j < m && color_segs[j].start_index <= tmp.end_index) {
        tmp.end_index = std::min(tmp.end_index, color_segs[j].end_index);
        if (tmp.end_index < tmp.start_index) {
          tmp.end_index = tmp.start_index;
        } else {
          tmp.color = color_segs[j].color;
        }
      }

      // AINFO<<"lxy_test if tmp.end_index:"<<tmp.end_index<<" ,type_segs["<<i<<"].end_index:"<<type_segs[i].end_index<<" tmp.type:"<<tmp.type<<" tmp.color:"<<tmp.color;
      if (tmp.end_index == type_segs[i].end_index)
        i++;
      // AINFO<<"lxy_test if tmp.end_index:"<<tmp.end_index<<" ,color_segs["<<j<<"].end_index:"<<color_segs[j].end_index<<" tmp.type:"<<tmp.type<<" tmp.color:"<<tmp.color;
      if (j < m && tmp.end_index >= color_segs[j].end_index)
        j++;
    } else {
      // 当前区间来自 color_segs
      // tmp.start_index = color_segs[j].start_index;
      if (((tmp.end_index + 1 <= color_segs[j].end_index) || (tmp.end_index + 1 <= type_segs[i].end_index)) && (i > 0 || j > 0)) {
        tmp.start_index = tmp.end_index + 1;
        // AERROR<<"lxy_test else tmp.start_index:"<<tmp.start_index;
      }
      tmp.end_index = color_segs[j].end_index;
      tmp.color     = color_segs[j].color;

      // 检查与 type_segs 的交叉
      if (i < n && type_segs[i].start_index <= tmp.end_index) {
        tmp.end_index = std::min(tmp.end_index, type_segs[i].end_index);
        tmp.type      = type_segs[i].type;
      }

      // AINFO<<"lxy_test else tmp.end_index:"<<tmp.end_index<<" ,color_segs["<<j<<"].end_index:"<<color_segs[j].end_index<<" tmp.type:"<<tmp.type<<" tmp.color:"<<tmp.color;
      if (tmp.end_index == color_segs[j].end_index)
        j++;
      // AINFO<<"lxy_test else tmp.end_index:"<<tmp.end_index<<" ,type_segs["<<i<<"].end_index:"<<type_segs[i].end_index<<" tmp.type:"<<tmp.type<<" tmp.color:"<<tmp.color;
      if (i < n && tmp.end_index == type_segs[i].end_index)
        i++;
    }

    // 检查是否需要调整上一段的 end_index
    if (!merged_segs.empty() && merged_segs.back().end_index == tmp.start_index) {
      tmp.start_index = merged_segs.back().end_index + 1;  // 前一段的 end_index + 1
    }
    // AWARN<<"lxy_test tmp.start_index:"<<tmp.start_index<<" tmp.end_index:"<<tmp.end_index<<" tmp.type:"<<tmp.type<<" tmp.color:"<<tmp.color;
    merged_segs.push_back(tmp);
  }
}

void AdaptInput2Internal(const std::shared_ptr<byd::msg::perception::PerceptionTrafficInfo> &raw_perception_traffic_info,
                         PercepTrfInfoPtr                                                    adapted_percep_trf_info) {
  adapted_percep_trf_info->header.recv_timestamp = GetMwTimeNowSec();
  adapted_percep_trf_info->header.timestamp      = raw_perception_traffic_info->header().measurement_timestamp();
  adapted_percep_trf_info->header.cycle_counter  = raw_perception_traffic_info->header().sequence_num();
  adapted_percep_trf_info->num_objects           = raw_perception_traffic_info->objects().size();
  adapted_percep_trf_info->objects.reserve(raw_perception_traffic_info->objects().size());

  for (const auto &raw_percep_trf_object : raw_perception_traffic_info->objects()) {
    adapted_percep_trf_info->objects.emplace_back(TrfObjectInfo());
    auto &adapted_percep_trf_obj          = adapted_percep_trf_info->objects.back();
    adapted_percep_trf_obj.id             = raw_percep_trf_object.id();
    adapted_percep_trf_obj.type           = static_cast<cem::message::sensor::ObjectType>(raw_percep_trf_object.type());
    adapted_percep_trf_obj.source         = static_cast<cem::message::sensor::CameraSource>(raw_percep_trf_object.source());
    adapted_percep_trf_obj.position.x     = raw_percep_trf_object.position().x();
    adapted_percep_trf_obj.position.y     = raw_percep_trf_object.position().y();
    adapted_percep_trf_obj.position.z     = raw_percep_trf_object.position().z();
    adapted_percep_trf_obj.position_std.x = raw_percep_trf_object.position_std().x();
    adapted_percep_trf_obj.position_std.y = raw_percep_trf_object.position_std().y();
    adapted_percep_trf_obj.position_std.z = raw_percep_trf_object.position_std().z();
    adapted_percep_trf_obj.length         = raw_percep_trf_object.length();
    adapted_percep_trf_obj.width          = raw_percep_trf_object.width();
    adapted_percep_trf_obj.height         = raw_percep_trf_object.height();
    adapted_percep_trf_obj.attributes.speed_restriction_type =
        static_cast<cem::message::sensor::SpeedRestrictionType>(raw_percep_trf_object.attributes().speed_restriction_type());
    adapted_percep_trf_obj.attributes.speed_restriction_num = raw_percep_trf_object.attributes().speed_restriction_num();
    adapted_percep_trf_obj.attributes.traffic_light_direction =
        static_cast<cem::message::sensor::TrafficLightDirectionType>(raw_percep_trf_object.attributes().traffic_light_direction());
    adapted_percep_trf_obj.attributes.traffic_light_color =
        static_cast<cem::message::sensor::TrafficLightColorType>(raw_percep_trf_object.attributes().traffic_light_color());
    adapted_percep_trf_obj.attributes.traffic_light_shape =
        static_cast<cem::message::sensor::TrafficLightShapeType>(raw_percep_trf_object.attributes().traffic_light_shape());
    adapted_percep_trf_obj.attributes.traffic_light_num      = raw_percep_trf_object.attributes().traffic_light_num();
    adapted_percep_trf_obj.attributes.traffic_light_flashing = raw_percep_trf_object.attributes().traffic_light_flashing();
    adapted_percep_trf_obj.exist_score                       = raw_percep_trf_object.exist_score();
    adapted_percep_trf_obj.track_age                         = raw_percep_trf_object.track_age();
    adapted_percep_trf_obj.track_status = static_cast<cem::message::sensor::TrackStatus>(raw_percep_trf_object.track_status());
    adapted_percep_trf_obj.direction_conf = raw_percep_trf_object.attributes().traffic_light_direction_conf();
    adapted_percep_trf_obj.shape_conf = raw_percep_trf_object.attributes().traffic_light_shape_conf();
    adapted_percep_trf_obj.traffic_light_box_id = raw_percep_trf_object.traffic_light_box_id();
    adapted_percep_trf_obj.is_occluded          = raw_percep_trf_object.attributes().is_occluded();
    adapted_percep_trf_obj.yellow_flashing_start_time = raw_percep_trf_object.attributes().yellow_flashing_start_time();
    adapted_percep_trf_obj.det_source = static_cast<cem::message::sensor::DetectionSource>(raw_percep_trf_object.det_source());
    // AINFO<<"tsr perception_traffic_info id: "<<adapted_percep_trf_obj.id<<" type: "<<adapted_percep_trf_obj.type;

    // AINFO<<"tsr perception_traffic_info attr spd_restrict_type:"<<adapted_percep_trf_obj.attributes.speed_restriction_type
    // <<", speed_restric_num:"<<adapted_percep_trf_obj.attributes.speed_restriction_num;
  }
}

void AdaptInput2Internal(const std::shared_ptr<byd::msg::perception::PerceptionTrafficInfo> &raw_perception_traffic_info,
                         VisionTrfInfoPtr                                                    adapted_vision_trf_info) {
  adapted_vision_trf_info->header.recv_timestamp = GetMwTimeNowSec();
  adapted_vision_trf_info->header.timestamp      = raw_perception_traffic_info->header().measurement_timestamp();
  adapted_vision_trf_info->header.cycle_counter  = raw_perception_traffic_info->header().sequence_num();
  adapted_vision_trf_info->num_objects           = raw_perception_traffic_info->objects().size();
  adapted_vision_trf_info->objects.reserve(raw_perception_traffic_info->objects().size());

  for (const auto &raw_percep_trf_object : raw_perception_traffic_info->objects()) {
    adapted_vision_trf_info->objects.emplace_back(TrfObjectInfo());
    auto &adapted_percep_trf_obj          = adapted_vision_trf_info->objects.back();
    adapted_percep_trf_obj.id             = raw_percep_trf_object.id();
    adapted_percep_trf_obj.type           = static_cast<cem::message::sensor::ObjectType>(raw_percep_trf_object.type());
    adapted_percep_trf_obj.source         = static_cast<cem::message::sensor::CameraSource>(raw_percep_trf_object.source());
    adapted_percep_trf_obj.position.x     = raw_percep_trf_object.position().x();
    adapted_percep_trf_obj.position.y     = raw_percep_trf_object.position().y();
    adapted_percep_trf_obj.position.z     = raw_percep_trf_object.position().z();
    adapted_percep_trf_obj.position_std.x = raw_percep_trf_object.position_std().x();
    adapted_percep_trf_obj.position_std.y = raw_percep_trf_object.position_std().y();
    adapted_percep_trf_obj.position_std.z = raw_percep_trf_object.position_std().z();
    adapted_percep_trf_obj.length         = raw_percep_trf_object.length();
    adapted_percep_trf_obj.width          = raw_percep_trf_object.width();
    adapted_percep_trf_obj.height         = raw_percep_trf_object.height();
    adapted_percep_trf_obj.attributes.speed_restriction_type =
        static_cast<cem::message::sensor::SpeedRestrictionType>(raw_percep_trf_object.attributes().speed_restriction_type());
    adapted_percep_trf_obj.attributes.speed_restriction_num = raw_percep_trf_object.attributes().speed_restriction_num();
    adapted_percep_trf_obj.attributes.traffic_light_direction =
        static_cast<cem::message::sensor::TrafficLightDirectionType>(raw_percep_trf_object.attributes().traffic_light_direction());
    adapted_percep_trf_obj.attributes.traffic_light_color =
        static_cast<cem::message::sensor::TrafficLightColorType>(raw_percep_trf_object.attributes().traffic_light_color());
    adapted_percep_trf_obj.attributes.traffic_light_shape =
        static_cast<cem::message::sensor::TrafficLightShapeType>(raw_percep_trf_object.attributes().traffic_light_shape());
    adapted_percep_trf_obj.attributes.traffic_light_num      = raw_percep_trf_object.attributes().traffic_light_num();
    adapted_percep_trf_obj.attributes.traffic_light_flashing = raw_percep_trf_object.attributes().traffic_light_flashing();
    adapted_percep_trf_obj.exist_score                       = raw_percep_trf_object.exist_score();
    adapted_percep_trf_obj.track_age                         = raw_percep_trf_object.track_age();
    adapted_percep_trf_obj.track_status = static_cast<cem::message::sensor::TrackStatus>(raw_percep_trf_object.track_status());
    
    // AINFO<<"tsr perception_traffic_info id: "<<adapted_percep_trf_obj.id<<" type: "<<adapted_percep_trf_obj.type;
    // AINFO<<"tsr perception_traffic_info attr spd_restrict_type:"<<adapted_percep_trf_obj.attributes.speed_restriction_type
    // <<", speed_restric_num:"<<adapted_percep_trf_obj.attributes.speed_restriction_num;
  }
}

void AdaptInput2Internal(const std::shared_ptr<byd::msg::perception::PerceptionObstacles> &raw_perception_obstacles_info,
                         FusObjInfoPtr                                                     adapted_fusion_obj_info) {
  if (raw_perception_obstacles_info == nullptr || adapted_fusion_obj_info == nullptr) {
    return;
  }

  auto *header                        = &adapted_fusion_obj_info->header;
  header->sequence_num                = raw_perception_obstacles_info->header().sequence_num();
  header->timestamp                   = raw_perception_obstacles_info->header().measurement_timestamp();
  header->recv_timestamp              = GetMwTimeNowSec();
  adapted_fusion_obj_info->num_tracks = raw_perception_obstacles_info->num_objects();
  for (const auto &obj : raw_perception_obstacles_info->obstacles()) {
    if ((static_cast<uint32_t>(obj.type()) > 3 && static_cast<uint32_t>(obj.type()) < 12) ||
        (static_cast<uint32_t>(obj.type()) > 13 && static_cast<uint32_t>(obj.type()) < 20)) {
      continue;
    }
    cem::message::sensor::FusionObj fusion_obj;
    fusion_obj.timestamp             = header->timestamp;
    fusion_obj.id                    = obj.id();
    fusion_obj.obstacle_valid        = true;
    fusion_obj.fusion_source         = static_cast<uint16_t>(obj.fusion_source());
    fusion_obj.type                  = static_cast<uint32_t>(obj.type());
    fusion_obj.position.x()          = obj.position().x();
    fusion_obj.position.y()          = obj.position().y();
    fusion_obj.velocity.x()          = obj.velocity().x();
    fusion_obj.velocity.y()          = obj.velocity().y();
    fusion_obj.acceleration.x()      = obj.acceleration().x();
    fusion_obj.acceleration.y()      = obj.acceleration().y();
    fusion_obj.heading_angle         = obj.heading_angle();
    fusion_obj.heading_rate          = obj.heading_rate();
    fusion_obj.length                = obj.length();
    fusion_obj.width                 = obj.width();
    fusion_obj.height                = obj.height();
    fusion_obj.speed                 = obj.speed();
    fusion_obj.normal_acceleration   = obj.normal_acceleration();
    fusion_obj.curvature             = obj.curvature();
    fusion_obj.is_velocity_valid     = obj.is_velocity_valid();
    fusion_obj.is_acceleration_valid = obj.is_acceleration_valid();
    fusion_obj.exist_score           = obj.exist_score();
    fusion_obj.track_age             = obj.track_age();
    fusion_obj.aeb_flag              = obj.aeb_flag();
    fusion_obj.object_lane_id        = obj.object_lane_id();
    fusion_obj.track_status          = static_cast<uint8_t>(obj.track_status());
    fusion_obj.cipv                  = static_cast<uint8_t>(obj.cipv());
    adapted_fusion_obj_info->fused_obstacles.emplace_back(fusion_obj);
  }
}

void AdaptInput2Internal(const std::shared_ptr<byd::msg::env_model::LocalMapInfo> &raw_lidar_roadedge_info,
                         LidarRoadEdgeInfoPtr                                      adapted_lidar_roadedge_info) {
  adapted_lidar_roadedge_info->header.timestamp      = raw_lidar_roadedge_info->header().measurement_timestamp();
  adapted_lidar_roadedge_info->header.recv_timestamp = GetMwTimeNowSec();
  adapted_lidar_roadedge_info->header.cycle_counter  = raw_lidar_roadedge_info->header().sequence_num();
  adapted_lidar_roadedge_info->edges.reserve(raw_lidar_roadedge_info->road_edges().size());
  auto AdaptInput_Edge = [](const byd::msg::env_model::LocalMapInfo::RoadEdge &raw_lane, BevLaneMarker *adapted_lane) -> void {
    adapted_lane->id               = raw_lane.track_id();
    adapted_lane->number_of_points = raw_lane.polyline().size();

    // pos mapping

    auto it = EnumMap2RoadEdgePosition.find(raw_lane.location());
    if (it != EnumMap2RoadEdgePosition.end()) {
      // The key exists in the map
      adapted_lane->position = (int)it->second;
    } else {
      adapted_lane->position = 0;
    }

    // bev road edge pos mapping
    auto it_bevedgepos = EnumMap2BevRoadEdgePosition.find(raw_lane.location());
    if (it_bevedgepos != EnumMap2BevRoadEdgePosition.end()) {
      // The key exists in the map
      adapted_lane->road_edeg_pos = it_bevedgepos->second;
    } else {
      adapted_lane->road_edeg_pos = BevRoadEdgePosition::BEV_REP__UNKNOWN;
    }

    // edge_type mapping
    // before cem using bev_lane type struct as the edge type
    auto it3 = EnumMap2RoadEdgeType.find(raw_lane.type());
    if (it3 != EnumMap2RoadEdgeType.end()) {
      // The key exists in the map
      adapted_lane->type = (int)it3->second;
    } else {
      adapted_lane->type = 0;
    }

    // adapted_lane->color = raw_lane.color();
    adapted_lane->conf = raw_lane.quality();
    adapted_lane->line_points.reserve(raw_lane.polyline().size());
    for (int idx = 0; idx < raw_lane.polyline().size(); idx++) {
      adapted_lane->line_points.emplace_back(Point2DF());
      auto       &adapted_point = adapted_lane->line_points.back();
      const auto &point         = raw_lane.polyline()[idx];
      adapted_point.x           = point.x();
      adapted_point.y           = point.y();
      adapted_point.mse         = raw_lane.mse()[idx];
      adapted_lane->geos->emplace_back(adapted_point.x, adapted_point.y);
    }
    return;
  };

  for (const auto &edge : raw_lidar_roadedge_info->road_edges()) {
    adapted_lidar_roadedge_info->edges.emplace_back(BevLaneMarker());
    auto &adapted_edge = adapted_lidar_roadedge_info->edges.back();
    AdaptInput_Edge(edge, &adapted_edge);
  }
}

void AdaptInput2Internal(const std::shared_ptr<byd::msg::planning::PLanningResultProto> &raw_planning_result,
                         PLanningResultPtr                                               adapted_planning_result_info) {
  auto *header                           = &adapted_planning_result_info->header;
  header->sequence_num                   = raw_planning_result->header().sequence_num();
  header->timestamp                      = raw_planning_result->header().measurement_timestamp();
  header->recv_timestamp                 = GetMwTimeNowSec();
  adapted_planning_result_info->lc_state = static_cast<PLanningResultData::LaneChangeState>(raw_planning_result->state().lc_state());
}

void AdaptInput2Internal(const std::shared_ptr<byd::msg::pnc::PlanFuncState> &raw_plan_func_state,
                         PlanFuncStatePtr                                     adapted_plan_func_state_info) {
  auto *header                                             = &adapted_plan_func_state_info->header;
  header->sequence_num                                     = raw_plan_func_state->header().sequence_num();
  header->timestamp                                        = raw_plan_func_state->header().measurement_timestamp();
  header->recv_timestamp                                   = GetMwTimeNowSec();
  adapted_plan_func_state_info->func_state.is_noa_actv_flg = raw_plan_func_state->func_state().is_noa_actv_flg();
}
void AdaptInput2Internal(const std::shared_ptr<byd::msg::orin::sm_msgs::MsgSM_BYD_CAN_Output> &raw_can1, CAN1Ptr adapted_can) {
  adapted_can->header.recv_timestamp = GetMwTimeNowSec();
  adapted_can->DNP_Stats_S           = raw_can1->dnp_stats_s();
}
void AdaptInput2Internal(const std::shared_ptr<byd::msg::perception::OCCInfo> &msg, OCCInfoPtr occInfo) {
  occInfo->header.timestamp       = msg->header().measurement_timestamp();
  occInfo->header.recv_timestamp  = GetMwTimeNowSec();
  auto ConvertPolygon2EigenVector = [](const byd::msg::perception::Polygon &polygon, std::vector<Eigen::Vector2d> &output) -> void {
    for (auto &ele : polygon.points()) {
      output.emplace_back();
      auto &ret = output.back();
      ret << ele.x(), ele.y();
    }
    return;
  };
  const auto &objsGod = msg->objects();
  for (const auto &god : objsGod) {
    if (god.category() == GeneralObjectType::GOD_STATIC) {
      occInfo->objects.emplace_back();
      auto &occObj = occInfo->objects.back();
      ConvertPolygon2EigenVector(god.shape(), occObj.shape);
    }
  }
  occInfo->objects.emplace_back();  //
  auto       &occObj = occInfo->objects.back();
  const auto &god    = msg->driving_corridor();
  ConvertPolygon2EigenVector(god, occObj.shape);  //合并
  occInfo->occ_mask.width                   = msg->occ_mask().width();
  occInfo->occ_mask.height                  = msg->occ_mask().height();
  occInfo->occ_mask.pixel_width_resolution  = msg->occ_mask().pixel_width_resolution();
  occInfo->occ_mask.pixel_height_resolution = msg->occ_mask().pixel_height_resolution();
  occInfo->occ_mask.veh_pos_x               = msg->occ_mask().veh_pos_x();
  occInfo->occ_mask.veh_pos_y               = msg->occ_mask().veh_pos_y();

  occInfo->occ_mask.rle_units.resize(msg->occ_mask().rle_units().size());
  for (int idx = 0; idx < msg->occ_mask().rle_units().size(); idx++) {
    auto &adapted_unit = occInfo->occ_mask.rle_units[idx];
    adapted_unit.value = msg->occ_mask().rle_units()[idx].value();
    adapted_unit.count = msg->occ_mask().rle_units()[idx].count();
  }
}
/******************************************************************************/
void AdaptInternal2Output(const MapInfo &raw_map_info, byd::msg::orin::routing_map::MapInfo *adapted_map_info) {
  adapted_map_info->set_map_version(raw_map_info.map_version);
  adapted_map_info->set_engine_version(raw_map_info.engine_version);
  byd::msg::orin::routing_map::MapInfo_CoordSys coord_sys =
      static_cast<byd::msg::orin::routing_map::MapInfo_CoordSys>(raw_map_info.coord_sys);
  adapted_map_info->set_coord_sys(coord_sys);
}
void AdaptInternal2Output(const SensorStatusInfo                        &rawSensorStatusInfo,
                          byd::msg::orin::routing_map::SensorStatusInfo *apapted_sensorstatus_info) {
  apapted_sensorstatus_info->set_sensor_status(rawSensorStatusInfo.sensor_status);
  apapted_sensorstatus_info->set_special_type(rawSensorStatusInfo.special_type);
  if (SensorInfoWarningType.count(rawSensorStatusInfo.warning_type) > 0) {
    apapted_sensorstatus_info->set_warning_type(
        SensorInfoWarningType.at(rawSensorStatusInfo.warning_type));
  } else {
    apapted_sensorstatus_info->set_warning_type(
        byd::msg::orin::routing_map::SensorStatusInfo_SensorWarningType::
            SensorStatusInfo_SensorWarningType_NO_WARNING);
  }
}
void AdaptInternal2Output(const LaneInfo &lane, byd::msg::orin::routing_map::LaneInfo *adapted_lane, cem::fusion::navigation::AccLaneInfo sd_acc_lane_info) {
  adapted_lane->set_id(lane.id);
  adapted_lane->set_section_id(lane.section_id);
  adapted_lane->set_junction_id(lane.junction_id);
  adapted_lane->set_left_lane_id(lane.left_lane_id);
  adapted_lane->set_right_lane_id(lane.right_lane_id);
  adapted_lane->clear_next_lane_ids();
  adapted_lane->mutable_next_lane_ids()->Reserve(lane.next_lane_ids.size());
  adapted_lane->set_stopline_angle_flag(lane.stopline_angle_flag);
  adapted_lane->set_distance_to_stopline(lane.distance_to_stopline);
  adapted_lane->set_cloud_junction_type(lane.cloud_junction_type);
  adapted_lane->set_traffic_match_cloud_file(lane.traffic_match_cloud_file);
  adapted_lane->mutable_light_info()->set_prev_has_light_now_no(lane.light_info.prev_has_light_now_no);
  adapted_lane->mutable_light_info()->set_maybe_exist_light(lane.light_info.maybe_exist_light);
  adapted_lane->mutable_light_info()->set_yellow_flashing_start_time(lane.yellow_flashing_start_time);
  for (const auto &nextId : lane.next_lane_ids) {
    adapted_lane->add_next_lane_ids(nextId);
  }

  // 处理 previous_lane_ids（新增字段）
  adapted_lane->clear_previous_lane_ids();
  adapted_lane->mutable_previous_lane_ids()->Reserve(lane.previous_lane_ids.size());
  for (const auto &prevId : lane.previous_lane_ids) {
    adapted_lane->add_previous_lane_ids(prevId);
  }

  adapted_lane->clear_left_lane_boundary_ids();
  adapted_lane->mutable_left_lane_boundary_ids()->Reserve(lane.left_lane_boundary_ids.size());
  for (const auto &leftId : lane.left_lane_boundary_ids) {
    adapted_lane->add_left_lane_boundary_ids(leftId);
  }
  adapted_lane->clear_right_lane_boundary_ids();
  adapted_lane->mutable_right_lane_boundary_ids()->Reserve(lane.right_lane_boundary_ids.size());
  for (const auto &rightId : lane.right_lane_boundary_ids) {
    adapted_lane->add_right_lane_boundary_ids(rightId);
  }
  adapted_lane->clear_left_road_boundary_ids();
  adapted_lane->mutable_left_road_boundary_ids()->Reserve(lane.left_road_boundary_ids.size());
  for (const auto &leftRoadId : lane.left_road_boundary_ids) {
    adapted_lane->add_left_road_boundary_ids(leftRoadId);
  }

  adapted_lane->clear_right_road_boundary_ids();
  adapted_lane->mutable_right_road_boundary_ids()->Reserve(lane.right_road_boundary_ids.size());
  for (const auto &rightRadId : lane.right_road_boundary_ids) {
    adapted_lane->add_right_road_boundary_ids(rightRadId);
  }
  //is_acc_adj_lane
  //adapted_lane->set_is_acc_adj_lane(lane.is_acc_adj_lane);
    bool is_target_lane = false;
    double merge_start = std::numeric_limits<double>::max();
    double merge_end   = std::numeric_limits<double>::max();

    // 判断是否是目标邻车道
    if (sd_acc_lane_info.exists &&
        sd_acc_lane_info.ego_normal_lane_id != 0 &&
        lane.id == sd_acc_lane_info.ego_normal_lane_id) {
        
        is_target_lane = true;
        merge_start = sd_acc_lane_info.merge_start_dis;
        merge_end   = sd_acc_lane_info.merge_end_dis;
      //  AINFO << "  [AdaptInternal2Output] Setting  LD lane ID: " << lane.id << " to LANE_ACC_ADJ"
      //                      << " merge_start_dis: " << merge_start << " mergeEnd: " << merge_end;
    }

    adapted_lane->set_is_acc_adj_lane(is_target_lane);
    adapted_lane->set_merge_start_dis(merge_start);
    adapted_lane->set_merge_end_dis(merge_end);

  byd::msg::orin::routing_map::LaneInfo_LaneType laneType = static_cast<byd::msg::orin::routing_map::LaneInfo_LaneType>(lane.type);
  adapted_lane->set_type(laneType);

  byd::msg::orin::routing_map::NoneOddType none_odd = static_cast<byd::msg::orin::routing_map::NoneOddType>(lane.none_odd_type);
  adapted_lane->set_none_odd_type(none_odd);

  byd::msg::orin::routing_map::LaneInfo_TurnType turnType = static_cast<byd::msg::orin::routing_map::LaneInfo_TurnType>(lane.turn_type);
  adapted_lane->set_turn_type(turnType);

  byd::msg::orin::routing_map::LightStatus light_status = static_cast<byd::msg::orin::routing_map::LightStatus>(lane.light_status);
  adapted_lane->set_light_status(light_status);

  byd::msg::orin::routing_map::LaneInfo_SplitTopology split_topology =
      static_cast<byd::msg::orin::routing_map::LaneInfo_SplitTopology>(lane.split_topology);
  adapted_lane->set_split_topology(split_topology);

  byd::msg::orin::routing_map::LaneInfo_MergeTopology merge_topology =
      static_cast<byd::msg::orin::routing_map::LaneInfo_MergeTopology>(lane.merge_topology);
  adapted_lane->set_merge_topology(merge_topology);

  adapted_lane->set_length(lane.length);
  adapted_lane->set_speed_limit(lane.speed_limit);
  adapted_lane->set_cnoa_is_virtual(lane.cnoa_is_virtual);
  adapted_lane->set_light_countdown(lane.light_countdown);
  adapted_lane->set_traffic_light_obj_id(lane.traffic_light_obj_id);
  adapted_lane->set_traffic_light_seq_num(lane.traffic_light_seq_num);
  adapted_lane->set_traffic_set_reason(lane.traffic_set_reason);
  adapted_lane->set_stay_prev_counter(lane.stay_prev_counter);

  adapted_lane->clear_points();
  adapted_lane->mutable_points()->Reserve(lane.points.size());
  for (const auto &point : lane.points) {
    AdaptInternal2Output(point, adapted_lane->add_points());
  }

  adapted_lane->clear_cross_walks();
  adapted_lane->mutable_cross_walks()->Reserve(lane.cross_walks.size());
  for (const auto &crosssWalk : lane.cross_walks) {
    adapted_lane->add_cross_walks(crosssWalk);
  }

  adapted_lane->clear_traffic_stop_lines();
  adapted_lane->mutable_traffic_stop_lines()->Reserve(lane.traffic_stop_lines.size());
  for (const auto &stopLine : lane.traffic_stop_lines) {
    adapted_lane->add_traffic_stop_lines(stopLine);
  }

  adapted_lane->clear_exp_trajectory_ids();
  adapted_lane->mutable_exp_trajectory_ids()->Reserve(lane.exp_trajectory_ids.size());
  for (const auto &exp_trajectory_id : lane.exp_trajectory_ids) {
    adapted_lane->add_exp_trajectory_ids(exp_trajectory_id);
  }

  auto *out_ri = adapted_lane->mutable_restricted_info();
  out_ri->set_restricted_type(static_cast<byd::msg::orin::routing_map::RestrictedType>(lane.restricted_info.restricted_type));
  // out_ri->set_is_passable(lane.restricted_info.is_passable);
  ///透传地图特殊车道的限行属性
  out_ri->set_passable_env_state(lane.restricted_info.passable_env_state);

}

void AdaptInternal2Output(const LaneBoundaryInfo &raw_lane_bound, byd::msg::orin::routing_map::LaneBoundaryInfo *adapted_lane_boundaries) {
  adapted_lane_boundaries->set_id(raw_lane_bound.id);
  adapted_lane_boundaries->clear_points();
  adapted_lane_boundaries->mutable_points()->Reserve(raw_lane_bound.points.size());
  for (const auto &point : raw_lane_bound.points) {
    AdaptInternal2Output(point, adapted_lane_boundaries->add_points());
    AdaptInternal2Output(point, adapted_lane_boundaries->add_line_points());
  }
  byd::msg::orin::routing_map::LaneBoundaryInfo_LineType lineType =
      static_cast<byd::msg::orin::routing_map::LaneBoundaryInfo_LineType>(raw_lane_bound.line_type);
  byd::msg::orin::routing_map::LaneBoundaryInfo_LaneMarkerColor lineColor =
      static_cast<byd::msg::orin::routing_map::LaneBoundaryInfo_LaneMarkerColor>(raw_lane_bound.line_color);
  byd::msg::orin::routing_map::LaneBoundaryInfo_ColorSegment adapted_color_segment;
  adapted_color_segment.set_color(lineColor);
  adapted_color_segment.set_start_index(0);
  adapted_color_segment.set_end_index(raw_lane_bound.points.size() - 1);
  adapted_lane_boundaries->clear_color_seg();
  adapted_lane_boundaries->add_color_seg()->CopyFrom(adapted_color_segment);
  adapted_lane_boundaries->set_line_type(lineType);
}

void AdaptInternal2Output(const RoadBoundaryInfo &raw_road_boundary, byd::msg::orin::routing_map::RoadBoundaryInfo *adapted_roadBoundary) {
  adapted_roadBoundary->set_id(raw_road_boundary.id);
  adapted_roadBoundary->clear_points();
  adapted_roadBoundary->mutable_points()->Reserve(raw_road_boundary.points.size());
  for (const auto &point : raw_road_boundary.points) {
    AdaptInternal2Output(point, adapted_roadBoundary->add_points());
    AdaptInternal2Output(point, adapted_roadBoundary->add_line_points());
  }
  byd::msg::orin::routing_map::RoadBoundaryInfo_BoundaryType boundaryType =
      static_cast<byd::msg::orin::routing_map::RoadBoundaryInfo_BoundaryType>(raw_road_boundary.boundary_type);
  adapted_roadBoundary->set_boundary_type(boundaryType);
}

void AdaptInternal2Output(const StopLineInfo &raw_stop_line, byd::msg::orin::routing_map::StopLineInfo *adapted_stop_line) {
  adapted_stop_line->set_id(raw_stop_line.id);
  adapted_stop_line->clear_points();
  adapted_stop_line->mutable_points()->Reserve(raw_stop_line.points.size());
  for (const auto &point : raw_stop_line.points) {
    AdaptInternal2Output(point, adapted_stop_line->add_points());
  }
  adapted_stop_line->set_type(raw_stop_line.type);
}

void AdaptInternal2Output(const JunctionInfo &raw_junction, byd::msg::orin::routing_map::JunctionInfo *adapted_junction) {
  adapted_junction->set_id(raw_junction.id);
  adapted_junction->clear_points();
  adapted_junction->mutable_points()->Reserve(raw_junction.points.size());
  for (const auto &point : raw_junction.points) {
    AdaptInternal2Output(point, adapted_junction->add_points());
  }
}

void AdaptInternal2Output(const CrossWalkInfo &raw_cross, byd::msg::orin::routing_map::CrossWalkInfo *adapted_cross) {
  adapted_cross->set_id(raw_cross.id);
  adapted_cross->clear_points();
  adapted_cross->mutable_points()->Reserve(raw_cross.points.size());
  for (const auto &point : raw_cross.points) {
    AdaptInternal2Output(point, adapted_cross->add_points());
  }
}

void AdaptInternal2Output(const Arrows &raw_arrow, byd::msg::orin::routing_map::ArrowInfo *adapted_arrows) {

  byd::msg::orin::routing_map::ArrowInfo adapted_arrow;
  adapted_arrow.set_id(static_cast<uint64_t>(raw_arrow.id));
  switch (raw_arrow.type) {
    case 11:
      adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::RM_TYPE_STRAIGHT);
      break;
    case 12:
      adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::RM_TYPE_LEFT);
      break;
    case 13:
      adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::RM_TYPE_RIGHT);
      break;
    case 14:
      adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::RM_TYPE_TURNING);
      break;
    case 15:
      adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::RM_TYPE_STRAIGHT_LEFT);
      break;
    case 16:
      adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::RM_TYPE_STRAIGHT_RIGHT);
      break;
    case 17:
      adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::RM_TYPE_STRAIGHT_LEFT_RIGHT);
      break;
    case 18:
      adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::RM_TYPE_LEFT_RIGHT);
      break;
    case 19:
      adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::RM_TYPE_STRAIGHT_TURNING);
      break;
    case 20:
      adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::RM_TYPE_LEFT_TURNING);
      break;
    case 21:
      adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::ArrowType_MIN);
      break;
    case 22:
      adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::ArrowType_MAX);
      break;
    default:
      break;
  }

  for (const auto &point : raw_arrow.points) {
    byd::msg::basic::Point3D adapted_point;
    adapted_point.set_x(static_cast<double>(point.x));
    adapted_point.set_y(static_cast<double>(point.y));
    adapted_point.set_z(0.0);
    adapted_arrow.add_bounding_box_geometry()->CopyFrom(adapted_point);
  }
  adapted_arrows->CopyFrom(adapted_arrow);
}

void AdaptInternal2Output(const SectionInfo &raw_section, byd::msg::orin::routing_map::SectionInfo *adapted_section) {
  adapted_section->set_id(raw_section.id);
  adapted_section->set_length(raw_section.length);
  adapted_section->clear_lane_ids();
  adapted_section->mutable_lane_ids()->Reserve(raw_section.lane_ids.size());
  adapted_section->set_link_type(raw_section.link_type);
  for (const auto &id : raw_section.lane_ids) {
    adapted_section->add_lane_ids(id);
  }
  adapted_section->clear_points();
  adapted_section->mutable_points()->Reserve(raw_section.points.size());
  for (const auto &point : raw_section.points) {
    AdaptInternal2Output(point, adapted_section->add_points());
  }

  byd::msg::orin::routing_map::RoadClass roadClass = static_cast<byd::msg::orin::routing_map::RoadClass>(raw_section.road_class);
  adapted_section->set_road_class(roadClass);

  // 新增处理 predecessor_section_id_list
  adapted_section->clear_predecessor_section_id_list();
  adapted_section->mutable_predecessor_section_id_list()->Reserve(raw_section.predecessor_section_id_list.size());
  for (const auto &predecessor_id : raw_section.predecessor_section_id_list) {
    adapted_section->add_predecessor_section_id_list(predecessor_id);
  }

  // 新增处理 successor_section_id_list
  adapted_section->clear_successor_section_id_list();
  adapted_section->mutable_successor_section_id_list()->Reserve(raw_section.successor_section_id_list.size());
  for (const auto &successor_id : raw_section.successor_section_id_list) {
    adapted_section->add_successor_section_id_list(successor_id);
  }
}

void AdaptInternal2Output(const NaviPosition &raw_naviPos, byd::msg::orin::routing_map::NaviPosition *adapted_navi_pos) {
  adapted_navi_pos->set_section_id(raw_naviPos.section_id);
  adapted_navi_pos->set_s_offset(raw_naviPos.s_offset);
}

void AdaptInternal2Output(const RouteInfo &raw_route, byd::msg::orin::routing_map::RouteInfo *adapted_route) {
  adapted_route->set_id(raw_route.id);
  adapted_route->clear_navi_start();
  AdaptInternal2Output(raw_route.navi_start, adapted_route->mutable_navi_start());

  adapted_route->clear_sections();
  adapted_route->mutable_sections()->Reserve(raw_route.sections.size());
  for (const auto &section : raw_route.sections) {
    AdaptInternal2Output(section, adapted_route->add_sections());
  }

  // 新增字段：适配 subpaths
  adapted_route->clear_subpaths();
  adapted_route->mutable_subpaths()->Reserve(raw_route.subpaths.size());
  for (const auto &subpath : raw_route.subpaths) {
    auto *adapted_subpath = adapted_route->add_subpaths();
    adapted_subpath->set_enter_section_id(subpath.enter_section_id);
    adapted_subpath->mutable_sections()->Reserve(subpath.sections.size());
    for (const auto &section : subpath.sections) {
      AdaptInternal2Output(section, adapted_subpath->add_sections());
    }
  }

  adapted_route->clear_route_id();
  adapted_route->set_route_id(raw_route.route_id);
  adapted_route->clear_extend_sections();
  adapted_route->mutable_extend_sections()->Reserve(raw_route.extend_sections.size());
  for (const auto &extend_section : raw_route.extend_sections) {
    auto *adapted_extend_section = adapted_route->add_extend_sections();
    adapted_extend_section->set_from_section_idx(extend_section.from_section_idx);
    adapted_extend_section->set_from_section_id(extend_section.from_section_id);
    adapted_extend_section->set_to_section_idx(extend_section.to_section_idx);
    adapted_extend_section->mutable_sections()->Reserve(extend_section.sections.size());
    for (const auto &section : extend_section.sections) {
      AdaptInternal2Output(section, adapted_extend_section->add_sections());
    }
  }
}

void AdaptInternal2Output(const cem::message::env_model::TrafficLightMap &raw_traffic_light,
                          byd::msg::orin::routing_map::TrafficLightInfo  *adapted_traffic_light) {
  adapted_traffic_light->set_id(raw_traffic_light.id);
  AdaptInternal2Output(raw_traffic_light.center_position, adapted_traffic_light->mutable_center_position());
  for (const auto &pt : raw_traffic_light.bounding_box_geometry) {
    AdaptInternal2Output(pt, adapted_traffic_light->add_bounding_box_geometry());
  }
  adapted_traffic_light->clear_laneids();
  adapted_traffic_light->mutable_laneids()->Reserve(raw_traffic_light.laneids.size());
  for (const auto &id : raw_traffic_light.laneids) {
    adapted_traffic_light->add_laneids(id);
  }
  adapted_traffic_light->set_light_countdown(raw_traffic_light.light_countdown);
  adapted_traffic_light->set_shape(raw_traffic_light.shape);
  byd::msg::orin::routing_map::LightStatus lightStatus =
      static_cast<byd::msg::orin::routing_map::LightStatus>(raw_traffic_light.light_status);
  adapted_traffic_light->set_light_status(lightStatus);
}

void AdaptInternal2Output(const cem::message::env_model::EnvInfo &internal_env_info, byd::msg::orin::routing_map::EnvInfo *proto_env_info) {
  proto_env_info->Clear();
  proto_env_info->set_v2_valid(internal_env_info.v2_valid);
  proto_env_info->set_v2_has_navigation(internal_env_info.v2_has_navigation);
  proto_env_info->set_v2_dist_to_ramp(internal_env_info.v2_dist_to_ramp);
  proto_env_info->set_v2_dist_to_toll(internal_env_info.v2_dist_to_toll);
  proto_env_info->set_v2_dist_to_tunnel(internal_env_info.v2_dist_to_tunnel);
  proto_env_info->set_dist_to_subpath(internal_env_info.v2_dist_to_subpath);
  proto_env_info->set_dist_to_split_routelanenum_dec(internal_env_info.v2_dist_to_split_routelanenum_dec);

  // 转换 v2_road_classes
  proto_env_info->mutable_v2_road_classes()->Reserve(internal_env_info.v2_road_classes.size());
  for (const auto &road_class : internal_env_info.v2_road_classes) {
    byd::msg::orin::routing_map::V2RoadClass *proto_road_class = proto_env_info->add_v2_road_classes();
    proto_road_class->set_start(road_class.start);
    proto_road_class->set_end(road_class.end);
    proto_road_class->set_road_class(static_cast<byd::msg::orin::routing_map::V2RoadClassType>(static_cast<int>(road_class.road_class)));
  }

  // 转换 v2_turn_info
  proto_env_info->mutable_v2_turn_info()->Reserve(internal_env_info.v2_turn_info.size());
  for (const auto &turn_info : internal_env_info.v2_turn_info) {
    byd::msg::orin::routing_map::V2TurnInfo *proto_turn_info = proto_env_info->add_v2_turn_info();
    proto_turn_info->set_id(turn_info.id);
    proto_turn_info->set_is_valid(turn_info.is_valid);

    proto_turn_info->set_turn_type(static_cast<byd::msg::orin::routing_map::V2TurnInfo_V2TurnType>(static_cast<int>(turn_info.turn_type)));
    proto_turn_info->set_detail_turn_type(
        static_cast<byd::msg::orin::routing_map::V2TurnInfo_V2DetailTurnType>(static_cast<int>(turn_info.detail_turn_type)));

    proto_turn_info->set_dist(turn_info.dist);

    proto_turn_info->set_v2_dist(turn_info.v2_dist);

    // 转换 before_turn 和 after_turn
    const auto &before_turn = turn_info.before_turn;
    const auto &after_turn  = turn_info.after_turn;

    byd::msg::orin::routing_map::V2RoadInfo *proto_before_turn = proto_turn_info->mutable_before_turn();
    proto_before_turn->set_road_class(static_cast<byd::msg::orin::routing_map::V2RoadClassType>(static_cast<int>(before_turn.road_class)));
    proto_before_turn->set_lane_num(before_turn.lane_num);

    byd::msg::orin::routing_map::V2RoadInfo *proto_after_turn = proto_turn_info->mutable_after_turn();
    proto_after_turn->set_road_class(static_cast<byd::msg::orin::routing_map::V2RoadClassType>(static_cast<int>(after_turn.road_class)));
    proto_after_turn->set_lane_num(after_turn.lane_num);
  }

  // 转换 traffic_flows
  proto_env_info->mutable_traffic_flows()->Reserve(internal_env_info.traffic_flows.size());
  for (const auto &traffic_flow : internal_env_info.traffic_flows) {
    byd::msg::orin::routing_map::V2TrafficFlow *proto_traffic_flow = proto_env_info->add_traffic_flows();
    proto_traffic_flow->set_start_s(traffic_flow.start_s);
    proto_traffic_flow->set_end_s(traffic_flow.end_s);
    proto_traffic_flow->set_type(
        static_cast<byd::msg::orin::routing_map::V2TrafficFlow_V2TrafficFlowType>(static_cast<int>(traffic_flow.type)));
  }
}

void AdaptInternal2Output(RoutingMapPtr routingMap, std::shared_ptr<byd::msg::orin::routing_map::RoutingMap> adapted_routing_map,
                          bool is_on_highway_flag, cem::fusion::navigation::AccLaneInfo sd_acc_lane_info) {
  adapted_routing_map->mutable_header()->set_measurement_timestamp(routingMap->header.timestamp);
  adapted_routing_map->clear_map_info();
  AdaptInternal2Output(routingMap->map_info, adapted_routing_map->mutable_map_info());
  byd::msg::orin::routing_map::RoutingMap_MapType type = static_cast<byd::msg::orin::routing_map::RoutingMap_MapType>(routingMap->type);
  adapted_routing_map->set_map_type(type);

  byd::msg::orin::routing_map::RoadClass roadClass = static_cast<byd::msg::orin::routing_map::RoadClass>(routingMap->cur_road_class);
  adapted_routing_map->set_cur_road_class(roadClass);
  adapted_routing_map->set_is_on_highway(is_on_highway_flag);
  adapted_routing_map->mutable_fusionspeedinfo()->set_speed_value(routingMap->speed_info.speed_value);
  adapted_routing_map->mutable_fusionspeedinfo()->set_normal_speed_status(routingMap->speed_info.normal_speed_status);
  adapted_routing_map->mutable_fusionspeedinfo()->set_speed_limit_mode(routingMap->speed_info.speed_limit_mode);
  adapted_routing_map->mutable_fusionspeedinfo()->clear_condition_limit();
  for (const auto &condition_limit : routingMap->speed_info.condition_limits) {
    auto *adapted_condition_limit = adapted_routing_map->mutable_fusionspeedinfo()->add_condition_limit();
    if (adapted_condition_limit != nullptr) {
      adapted_condition_limit->set_condition_limit_speed_value(condition_limit.condition_limit_speed_value);
      adapted_condition_limit->set_condition_distance(condition_limit.condition_distance);
    }
  }

  // lane
  adapted_routing_map->clear_lanes();
  adapted_routing_map->mutable_lanes()->Reserve(routingMap->lanes.size());
  for (const auto &raw_line : routingMap->lanes) {
    AdaptInternal2Output(raw_line, adapted_routing_map->add_lanes(), sd_acc_lane_info);
  }

  // lane bound
  adapted_routing_map->clear_lane_boundaries();
  adapted_routing_map->mutable_lane_boundaries()->Reserve(routingMap->lane_boundaries.size());
  for (const auto &raw_lane_bound : routingMap->lane_boundaries) {
    AdaptInternal2Output(raw_lane_bound, adapted_routing_map->add_lane_boundaries());
  }

  //road bound
  adapted_routing_map->clear_road_boundaries();
  adapted_routing_map->mutable_road_boundaries()->Reserve(routingMap->road_boundaries.size());
  for (const auto &raw_road_bound : routingMap->road_boundaries) {
    AdaptInternal2Output(raw_road_bound, adapted_routing_map->add_road_boundaries());
  }

  // stop line
  adapted_routing_map->clear_stop_lines();
  adapted_routing_map->mutable_stop_lines()->Reserve(routingMap->stop_lines.size());
  for (const auto &raw_stopLine : routingMap->stop_lines) {
    AdaptInternal2Output(raw_stopLine, adapted_routing_map->add_stop_lines());
  }

  // junctions
  adapted_routing_map->clear_junctions();
  adapted_routing_map->mutable_junctions()->Reserve(routingMap->junctions.size());
  for (const auto &raw_junction : routingMap->junctions) {
    AdaptInternal2Output(raw_junction, adapted_routing_map->add_junctions());
  }

  //cross_walks
  adapted_routing_map->clear_cross_walks();
  adapted_routing_map->mutable_cross_walks()->Reserve(routingMap->cross_walks.size());
  for (const auto &raw_cross : routingMap->cross_walks) {
    AdaptInternal2Output(raw_cross, adapted_routing_map->add_cross_walks());
  }

  //arrows
  adapted_routing_map->clear_arrows();
  adapted_routing_map->mutable_arrows()->Reserve(routingMap->arrows.size());
  for (const auto &raw_arrow : routingMap->arrows) {
    AdaptInternal2Output(raw_arrow, adapted_routing_map->add_arrows());
  }

  //RouteInfo
  adapted_routing_map->clear_route();
  AdaptInternal2Output(routingMap->route, adapted_routing_map->mutable_route());

  // TrafficLightInfo
  adapted_routing_map->clear_traffic_lights();
  adapted_routing_map->mutable_traffic_lights()->Reserve(routingMap->traffic_lights.size());
  for (const auto &light : routingMap->traffic_lights) {
    AdaptInternal2Output(light, adapted_routing_map->add_traffic_lights());
  }

  // 转换 EnvInfo
  AdaptInternal2Output(routingMap->env_info, adapted_routing_map->mutable_env_info());

  //exp_trajectory
  adapted_routing_map->clear_exp_trajectories();
  adapted_routing_map->mutable_exp_trajectories()->Reserve(routingMap->exp_trajectories.size());
  for (const auto &exp_trajectory : routingMap->exp_trajectories) {
    AdaptInternal2Output(exp_trajectory, adapted_routing_map->add_exp_trajectories());
  }
  //SensorStatusInfo
  adapted_routing_map->clear_sensor_status_info();
  AdaptInternal2Output(routingMap->sensor_status_info, adapted_routing_map->mutable_sensor_status_info());

  //Arrows
  adapted_routing_map->clear_arrows();

  //DebugInfo
  adapted_routing_map->set_debug_info(routingMap->debug_infos);
}
// 转换发送到端到端的红绿灯信息
void AdaptInternal2Output(const TrafficLightsE2EInfoPtr                                      traffic_lights_e2e,
                          std::shared_ptr<byd::msg::orin::routing_map::TrafficLightsE2EInfo> adapted_traffic_lights_e2e) {
  adapted_traffic_lights_e2e->mutable_header()->set_measurement_timestamp(GetMwTimeNowSec());
  adapted_traffic_lights_e2e->mutable_header()->set_frame_id("dr");
  adapted_traffic_lights_e2e->clear_traffic_status();
  for (const auto &traffic_light : traffic_lights_e2e->traffic_status) {
    auto *adapted_traffic_light = adapted_traffic_lights_e2e->add_traffic_status();
    adapted_traffic_light->set_light_status(static_cast<byd::msg::orin::routing_map::LightStatus>(traffic_light.light_status));
    adapted_traffic_light->set_turn_type(static_cast<byd::msg::orin::routing_map::LaneInfo_TurnType>(traffic_light.turn_type));
    for (const auto &point : traffic_light.stop_line_pts) {
      AdaptInternal2Output(point, adapted_traffic_light->add_stop_line_pts());
    }
    for (const auto &point : traffic_light.stop_line_pts_wait) {
      AdaptInternal2Output(point, adapted_traffic_light->add_stop_line_pts_wait());
    }
    adapted_traffic_light->set_traffic_light_num(traffic_light.traffic_light_num);
    adapted_traffic_light->set_is_navi_light(traffic_light.is_navi_light);
    adapted_traffic_light->set_stopline_is_virtual(traffic_light.stopline_is_virtual);
    adapted_traffic_light->set_distance_to_stopline(traffic_light.distance_to_stopline);
    adapted_traffic_light->set_traffic_match_cloud_file(traffic_light.traffic_match_cloud_file);
  }
  adapted_traffic_lights_e2e->clear_traffic_lights();
  for (const auto &light : traffic_lights_e2e->traffic_lights) {
    AdaptInternal2Output(light, adapted_traffic_lights_e2e->add_traffic_lights());
  }
  auto *pose = adapted_traffic_lights_e2e->mutable_pose();
  pose->set_x(traffic_lights_e2e->pose.x);
  pose->set_y(traffic_lights_e2e->pose.y);
  pose->set_z(traffic_lights_e2e->pose.z);
}

/***************************************************************/

void AdaptInternal2Output(std::shared_ptr<byd::msg::basic::ModuleStatus> adpted_diagnostic_status) {}

void AdaptInternal2Output(cem::message::env_model::TrafficLight &raw_tla_obj, byd::msg::env_model::TrafficLight *adapted_traffic_light) {
  adapted_traffic_light->set_tla_id(raw_tla_obj.tla_id);
  adapted_traffic_light->set_tla_distance_x(raw_tla_obj.tla_distance_x);
  adapted_traffic_light->set_tla_distance_y(raw_tla_obj.tla_distance_y);
  adapted_traffic_light->set_tla_distance_z(raw_tla_obj.tla_distance_z);
  adapted_traffic_light->set_tla_position_confidence(raw_tla_obj.tla_position_confidence);
  adapted_traffic_light->set_left_tla_color(raw_tla_obj.left_tla_color);
  adapted_traffic_light->set_left_tla_type(raw_tla_obj.left_tla_type);
  adapted_traffic_light->set_straight_tla_color(raw_tla_obj.straight_tla_color);
  adapted_traffic_light->set_straight_tla_type(raw_tla_obj.straight_tla_type);
  adapted_traffic_light->set_right_tla_color(raw_tla_obj.right_tla_color);
  adapted_traffic_light->set_right_tla_type(raw_tla_obj.right_tla_type);
  adapted_traffic_light->set_new_left_tla_second(0);
  adapted_traffic_light->set_new_straight_tla_second(0);
  adapted_traffic_light->set_new_right_tla_second(0);
  adapted_traffic_light->set_tla_reserved_1(0.0);
  adapted_traffic_light->set_tla_reserved_2(0.0);
  adapted_traffic_light->set_tla_reserved_3(0.0);
  adapted_traffic_light->set_tla_reserved_4(0.0);
  adapted_traffic_light->set_tla_reserved_5(0.0);
}

/* FusionMap */
void AdaptInternal2Output(const BevMapInfoPtr bev_map_ptr, const std::vector<cem::message::env_model::StopLine> stop_lines_ptr,
                          const SensorStatusInfo                                  &sensorStatus_info,
                          std::shared_ptr<byd::msg::orin::routing_map::RoutingMap> adapted_fusion_map, bool is_on_highway_flag) {
  if (!bev_map_ptr) {
    return;
  }
  adapted_fusion_map->mutable_header()->set_measurement_timestamp(bev_map_ptr->header.timestamp);
  adapted_fusion_map->set_map_type(static_cast<byd::msg::orin::routing_map::RoutingMap::MapType>(bev_map_ptr->map_type));
  adapted_fusion_map->mutable_map_info()->set_coord_sys(byd::msg::orin::routing_map::MapInfo_CoordSys::MapInfo_CoordSys_FLU);
  adapted_fusion_map->clear_junctions();
  adapted_fusion_map->clear_traffic_lights();

  // AINFO << "*******VisibilityInfoOutput******";
  // AINFO << "timestamp: " << std::fixed << adapted_fusion_map->mutable_header()->measurement_timestamp();

  adapted_fusion_map->clear_lanes();
  adapted_fusion_map->mutable_lanes()->Reserve(bev_map_ptr->lane_infos.size());
  adapted_fusion_map->set_is_on_highway(is_on_highway_flag);
  adapted_fusion_map->set_debug_info(bev_map_ptr->debug_infos);
  adapted_fusion_map->mutable_fusionspeedinfo()->set_speed_value(bev_map_ptr->speed_info.speed_value);
  adapted_fusion_map->mutable_fusionspeedinfo()->set_normal_speed_status(bev_map_ptr->speed_info.normal_speed_status);
  adapted_fusion_map->mutable_fusionspeedinfo()->set_speed_limit_mode(bev_map_ptr->speed_info.speed_limit_mode);
  adapted_fusion_map->mutable_fusionspeedinfo()->clear_condition_limit();
  for (const auto &condition_limit : bev_map_ptr->speed_info.condition_limits) {
    auto *adapted_condition_limit = adapted_fusion_map->mutable_fusionspeedinfo()->add_condition_limit();
    if (adapted_condition_limit != nullptr) {
      adapted_condition_limit->set_condition_limit_speed_value(condition_limit.condition_limit_speed_value);
      adapted_condition_limit->set_condition_distance(condition_limit.condition_distance);
    }
  }

  std::vector<cem::message::sensor::BevMapSectionInfo> sections;
  size_t                                               num_sections = bev_map_ptr->route.sections.size();
  for (const auto &subpath : bev_map_ptr->route.subpaths) {
    num_sections += subpath.sections.size();
  }
  sections.reserve(num_sections);
  sections.insert(sections.end(), bev_map_ptr->route.sections.begin(), bev_map_ptr->route.sections.end());
  for (const auto &subpath : bev_map_ptr->route.subpaths) {
    sections.insert(sections.end(), subpath.sections.begin(), subpath.sections.end());
  }
  std::vector<uint64_t> route_lanes_id;
  for (const auto &section : sections) {
    for (const auto &lane : section.lane_infos) {
      route_lanes_id.push_back(lane.id);
      auto *adapted_lane_info = adapted_fusion_map->add_lanes();
      adapted_lane_info->set_id(lane.id);
      adapted_lane_info->set_section_id(lane.section_id);
      if (lane.is_virtual && lane.junction_id != 0) {
        adapted_lane_info->set_junction_id(lane.junction_id);
      }
      adapted_lane_info->set_left_lane_id(lane.left_lane_id);
      adapted_lane_info->set_right_lane_id(lane.right_lane_id);
      adapted_lane_info->set_stopline_angle_flag(lane.stopline_angle_flag);
      for (const auto left_lane_marker_id : lane.left_lane_boundary_ids) {
        adapted_lane_info->add_left_lane_boundary_ids(left_lane_marker_id);
      }
      for (const auto right_lane_marker_id : lane.right_lane_boundary_ids) {
        adapted_lane_info->add_right_lane_boundary_ids(right_lane_marker_id);
      }

      adapted_lane_info->set_split_topology(static_cast<byd::msg::orin::routing_map::LaneInfo_SplitTopology>((int)lane.split_topo_extend));
      adapted_lane_info->set_merge_topology(static_cast<byd::msg::orin::routing_map::LaneInfo_MergeTopology>((int)lane.merge_topo_extend));

      auto *merge_info = adapted_lane_info->mutable_merge_info();
      merge_info->set_merge_valid(lane.merge_info_extend.merge_valid);
      merge_info->set_dis_to_merge(lane.merge_info_extend.dis_to_merge);

      switch (lane.merge_info_extend.merge_source) {
        case MergeSourceExtend::MERGE_UNKNOWN:
          merge_info->set_merge_source(byd::msg::orin::routing_map::LaneInfo::MERGE_UNKNOWN);
          break;
        case MergeSourceExtend::MERGE_BEV:
          merge_info->set_merge_source(byd::msg::orin::routing_map::LaneInfo::MERGE_BEV);
          break;
        case MergeSourceExtend::MERGE_LD:
          merge_info->set_merge_source(byd::msg::orin::routing_map::LaneInfo::MERGE_LD);
          break;
        case MergeSourceExtend::MERGE_SD:
          merge_info->set_merge_source(byd::msg::orin::routing_map::LaneInfo::MERGE_SD);
          break;
        default:
          merge_info->set_merge_source(byd::msg::orin::routing_map::LaneInfo::MERGE_UNKNOWN);
          break;
      }

      adapted_lane_info->mutable_left_road_boundary_ids()->Reserve(lane.left_road_boundary_ids.size());
      for (const auto left_road_boundary_id : lane.left_road_boundary_ids) {
        adapted_lane_info->add_left_road_boundary_ids(left_road_boundary_id);
      }
      adapted_lane_info->mutable_right_road_boundary_ids()->Reserve(lane.right_road_boundary_ids.size());
      for (const auto right_road_boundary_id : lane.right_road_boundary_ids) {
        adapted_lane_info->add_right_road_boundary_ids(right_road_boundary_id);
      }

      adapted_lane_info->set_is_acc_adj_lane(lane.is_acc_adj_lane);

      if (lane.is_acc_adj_lane) {
        adapted_lane_info->set_merge_start_dis(lane.merge_start_dis);
        adapted_lane_info->set_merge_end_dis(lane.merge_end_dis);
      } else {
        adapted_lane_info->set_merge_start_dis(std::numeric_limits<double>::max());
        adapted_lane_info->set_merge_end_dis(std::numeric_limits<double>::max());
      }

      switch (lane.lane_type) {
        case BevLaneType::LANE_TYPE_EMERGENCY: {
          adapted_lane_info->set_type(byd::msg::orin::routing_map::LaneInfo_LaneType::LaneInfo_LaneType_LANE_EMERGENCY);
          break;
        }
        case BevLaneType::LANE_TYPE_HARBOR_STOP: {
          adapted_lane_info->set_type(byd::msg::orin::routing_map::LaneInfo_LaneType::LaneInfo_LaneType_LANE_HARBOR_STOP);
          break;
        }
        case BevLaneType::LANE_TYPE_VIRTUAL_JUNCTION: {
          adapted_lane_info->set_type(byd::msg::orin::routing_map::LaneInfo_LaneType::LaneInfo_LaneType_LANE_VIRTUAL_JUNCTION);
          break;
        }
        case BevLaneType::LANE_BRT: {
          adapted_lane_info->set_type(byd::msg::orin::routing_map::LaneInfo_LaneType::LaneInfo_LaneType_LANE_BRT);
          break;
        }
        default: {
          adapted_lane_info->set_type(byd::msg::orin::routing_map::LaneInfo_LaneType::LaneInfo_LaneType_LANE_NORMAL);
          break;
        }
      }

      switch (lane.bev_turn_type) {
        case BevAction::LEFT_TURN: {
          adapted_lane_info->set_turn_type(byd::msg::orin::routing_map::LaneInfo_TurnType::LaneInfo_TurnType_LEFT_TURN);
          break;
        }
        case BevAction::STRAIGHT: {
          adapted_lane_info->set_turn_type(byd::msg::orin::routing_map::LaneInfo_TurnType::LaneInfo_TurnType_NO_TURN);
          break;
        }
        case BevAction::RIGHT_TURN: {
          adapted_lane_info->set_turn_type(byd::msg::orin::routing_map::LaneInfo_TurnType::LaneInfo_TurnType_RIGHT_TURN);
          break;
        }
        case BevAction::U_TURN: {
          adapted_lane_info->set_turn_type(byd::msg::orin::routing_map::LaneInfo_TurnType::LaneInfo_TurnType_U_TURN);
          break;
        }
        default: {
          adapted_lane_info->set_turn_type(byd::msg::orin::routing_map::LaneInfo_TurnType::LaneInfo_TurnType_OTHER_UNKNOWN);
          break;
        }
      }

      switch (lane.trafficlight_state) {
        case BevTrafficLightState::TL_COLOR_RED: {
          adapted_lane_info->set_light_status(byd::msg::orin::routing_map::LightStatus::RED_LIGHT);
          break;
        }
        case BevTrafficLightState::TL_COLOR_YELLOW: {
          adapted_lane_info->set_light_status(byd::msg::orin::routing_map::LightStatus::YELLOW_LIGHT);
          break;
        }
        case BevTrafficLightState::TL_COLOR_GREEN: {
          adapted_lane_info->set_light_status(byd::msg::orin::routing_map::LightStatus::GREEN_LIGHT);
          break;
        }
        case BevTrafficLightState::TL_COLOR_GREEN_FLASH: {
          adapted_lane_info->set_light_status(byd::msg::orin::routing_map::LightStatus::GREEN_LIGHT);
          break;
        }
        case BevTrafficLightState::TL_COLOR_RED_FLASH: {
          adapted_lane_info->set_light_status(byd::msg::orin::routing_map::LightStatus::RED_LIGHT);
          break;
        }
        case BevTrafficLightState::TL_COLOR_YELLOW_FLASH: {
          adapted_lane_info->set_light_status(byd::msg::orin::routing_map::LightStatus::YELLOW_LIGHT);
          break;
        }
        case BevTrafficLightState::TL_COLOR_BLURRING_MODE: {
          adapted_lane_info->set_light_status(byd::msg::orin::routing_map::LightStatus::BLURRING_MODE);
          break;
        }
        case BevTrafficLightState::TL_COLOR_BLOCK_FAILED: {
          adapted_lane_info->set_light_status(byd::msg::orin::routing_map::LightStatus::BLOCK_FAIL);
          break;
        }
        case BevTrafficLightState::TL_COLOR_UNKNOWN: {
          adapted_lane_info->set_light_status(byd::msg::orin::routing_map::LightStatus::UNKNOWN_LIGHT);
          break;
        }
        default: {
          adapted_lane_info->set_light_status(byd::msg::orin::routing_map::LightStatus::NONE_LIGHT);
          break;
        }
      }

      for (const auto &point : lane.line_points) {
        int size_before = adapted_lane_info->points_size();
        AdaptInternal2Output(point, adapted_lane_info->add_points());
        int size_after = adapted_lane_info->points_size();
        if (size_after > size_before) {
          auto *add_point = adapted_lane_info->mutable_points(size_after - 1);
          if (!point.scatter_visibility) {
            add_point->set_point_source(byd::msg::orin::routing_map::PointSource::PS_INVISIBLE);
            // AINFO << "input point: (" << add_point->x() << ", " << add_point->y() << "); " << adapted_lane_info->id();
            // AINFO << "************************";
          }
        }
      }

      adapted_lane_info->set_length(lane.length);
      adapted_lane_info->set_speed_limit(lane.speed_limit);

      adapted_lane_info->set_is_virtual(lane.is_mounted);
      adapted_lane_info->set_light_countdown(lane.traffic_light_num);
      adapted_lane_info->set_traffic_light_obj_id(lane.traffic_light_obj_id);
      adapted_lane_info->set_traffic_light_seq_num(lane.traffic_light_seq_num);
      adapted_lane_info->set_traffic_set_reason(lane.traffic_set_reason);
      for (auto traffic_stop_line : lane.traffic_stop_lines) {
        adapted_lane_info->add_traffic_stop_lines(traffic_stop_line);
      }
      for (auto traffic_crosswalk : lane.traffic_crosswalks) {
        adapted_lane_info->add_cross_walks(traffic_crosswalk);
      }
      adapted_lane_info->set_stay_prev_counter(lane.stay_prev_counter);

      for (auto next_lane_id : lane.next_lane_ids) {
        adapted_lane_info->add_next_lane_ids(next_lane_id);
      }
      for (auto previous_lane_id : lane.previous_lane_ids) {
        adapted_lane_info->add_previous_lane_ids(previous_lane_id);
      }
      adapted_lane_info->set_connect_score(lane.connect_score);
      adapted_lane_info->set_is_blocked(lane.is_blocked);
      // adapted_lane_info->set_direction(static_cast<byd::msg::env_model::Direction>((int)lane.direction));
      auto *restricted_info = adapted_lane_info->mutable_restricted_info();
      restricted_info->set_restricted_type(static_cast<byd::msg::orin::routing_map::RestrictedType>((int)lane.restricted_info.restricted_type));
      restricted_info->set_is_passable(lane.restricted_info.is_passable);
      restricted_info->set_passable_env_state(lane.restricted_info.passable_env_state);
    }
  }

  for (const auto &lane : bev_map_ptr->lane_infos) {
    if (std::find(route_lanes_id.begin(), route_lanes_id.end(), lane.id) != route_lanes_id.end()) {
      continue;
    }
    auto *adapted_lane_info = adapted_fusion_map->add_lanes();
    adapted_lane_info->set_id(lane.id);
    adapted_lane_info->set_section_id(lane.section_id);
    if (lane.is_virtual && lane.junction_id != 0) {
      adapted_lane_info->set_junction_id(lane.junction_id);
    }
    adapted_lane_info->set_left_lane_id(lane.left_lane_id);
    adapted_lane_info->set_right_lane_id(lane.right_lane_id);

    adapted_lane_info->clear_left_lane_boundary_ids();
    adapted_lane_info->mutable_left_lane_boundary_ids()->Reserve(lane.left_lane_boundary_ids.size());
    for (auto left_lane_boundary_id : lane.left_lane_boundary_ids) {
      adapted_lane_info->add_left_lane_boundary_ids(left_lane_boundary_id);
    }

    adapted_lane_info->clear_right_lane_boundary_ids();
    adapted_lane_info->mutable_right_lane_boundary_ids()->Reserve(lane.right_lane_boundary_ids.size());
    for (auto right_lane_boundary_id : lane.right_lane_boundary_ids) {
      adapted_lane_info->add_right_lane_boundary_ids(right_lane_boundary_id);
    }

    adapted_lane_info->set_split_topology(static_cast<byd::msg::orin::routing_map::LaneInfo_SplitTopology>((int)lane.split_topo_extend));
    adapted_lane_info->set_merge_topology(static_cast<byd::msg::orin::routing_map::LaneInfo_MergeTopology>((int)lane.merge_topo_extend));

    auto *merge_info = adapted_lane_info->mutable_merge_info();
    merge_info->set_merge_valid(lane.merge_info_extend.merge_valid);
    merge_info->set_dis_to_merge(lane.merge_info_extend.dis_to_merge);

    switch (lane.merge_info_extend.merge_source) {
      case MergeSourceExtend::MERGE_UNKNOWN:
        merge_info->set_merge_source(byd::msg::orin::routing_map::LaneInfo::MERGE_UNKNOWN);
        break;
      case MergeSourceExtend::MERGE_BEV:
        merge_info->set_merge_source(byd::msg::orin::routing_map::LaneInfo::MERGE_BEV);
        break;
      case MergeSourceExtend::MERGE_LD:
        merge_info->set_merge_source(byd::msg::orin::routing_map::LaneInfo::MERGE_LD);
        break;
      case MergeSourceExtend::MERGE_SD:
        merge_info->set_merge_source(byd::msg::orin::routing_map::LaneInfo::MERGE_SD);
        break;
      default:
        merge_info->set_merge_source(byd::msg::orin::routing_map::LaneInfo::MERGE_UNKNOWN);
        break;
    }

    adapted_lane_info->clear_left_road_boundary_ids();
    adapted_lane_info->mutable_left_road_boundary_ids()->Reserve(lane.left_road_boundary_ids.size());
    for (const auto left_road_boundary_id : lane.left_road_boundary_ids) {
      adapted_lane_info->add_left_road_boundary_ids(left_road_boundary_id);
    }

    adapted_lane_info->clear_right_road_boundary_ids();
    adapted_lane_info->mutable_right_road_boundary_ids()->Reserve(lane.right_road_boundary_ids.size());
    for (const auto right_road_boundary_id : lane.right_road_boundary_ids) {
      adapted_lane_info->add_right_road_boundary_ids(right_road_boundary_id);
    }

    adapted_lane_info->set_is_acc_adj_lane(lane.is_acc_adj_lane);

    if (lane.is_acc_adj_lane) {
      adapted_lane_info->set_merge_start_dis(lane.merge_start_dis);
      adapted_lane_info->set_merge_end_dis(lane.merge_end_dis);
    } else {
      adapted_lane_info->set_merge_start_dis(std::numeric_limits<double>::max());
      adapted_lane_info->set_merge_end_dis(std::numeric_limits<double>::max());
    }

    switch (lane.lane_type) {
      case BevLaneType::LANE_TYPE_EMERGENCY: {
        adapted_lane_info->set_type(byd::msg::orin::routing_map::LaneInfo_LaneType::LaneInfo_LaneType_LANE_EMERGENCY);
        break;
      }
      case BevLaneType::LANE_TYPE_HARBOR_STOP: {
        adapted_lane_info->set_type(byd::msg::orin::routing_map::LaneInfo_LaneType::LaneInfo_LaneType_LANE_HARBOR_STOP);
        break;
      }
      case BevLaneType::LANE_TYPE_VIRTUAL_JUNCTION: {
        adapted_lane_info->set_type(byd::msg::orin::routing_map::LaneInfo_LaneType::LaneInfo_LaneType_LANE_VIRTUAL_JUNCTION);
        break;
      }
      case BevLaneType::LANE_BRT: {
        adapted_lane_info->set_type(byd::msg::orin::routing_map::LaneInfo_LaneType::LaneInfo_LaneType_LANE_BRT);
        break;
      }
      default: {
        adapted_lane_info->set_type(byd::msg::orin::routing_map::LaneInfo_LaneType::LaneInfo_LaneType_LANE_NORMAL);
        break;
      }
    }

    switch (lane.bev_turn_type) {
      case BevAction::LEFT_TURN: {
        adapted_lane_info->set_turn_type(byd::msg::orin::routing_map::LaneInfo_TurnType::LaneInfo_TurnType_LEFT_TURN);
        break;
      }
      case BevAction::STRAIGHT: {
        adapted_lane_info->set_turn_type(byd::msg::orin::routing_map::LaneInfo_TurnType::LaneInfo_TurnType_NO_TURN);
        break;
      }
      case BevAction::RIGHT_TURN: {
        adapted_lane_info->set_turn_type(byd::msg::orin::routing_map::LaneInfo_TurnType::LaneInfo_TurnType_RIGHT_TURN);
        break;
      }
      default: {
        adapted_lane_info->set_turn_type(byd::msg::orin::routing_map::LaneInfo_TurnType::LaneInfo_TurnType_OTHER_UNKNOWN);
        break;
      }
    }

    switch (lane.trafficlight_state) {
      case BevTrafficLightState::TL_COLOR_RED: {
        adapted_lane_info->set_light_status(byd::msg::orin::routing_map::LightStatus::RED_LIGHT);
        break;
      }
      case BevTrafficLightState::TL_COLOR_YELLOW: {
        adapted_lane_info->set_light_status(byd::msg::orin::routing_map::LightStatus::YELLOW_LIGHT);
        break;
      }
      case BevTrafficLightState::TL_COLOR_GREEN: {
        adapted_lane_info->set_light_status(byd::msg::orin::routing_map::LightStatus::GREEN_LIGHT);
        break;
      }
      case BevTrafficLightState::TL_COLOR_GREEN_FLASH: {
        adapted_lane_info->set_light_status(byd::msg::orin::routing_map::LightStatus::GREEN_BLINKING);
        break;
      }
      case BevTrafficLightState::TL_COLOR_RED_FLASH: {
        adapted_lane_info->set_light_status(byd::msg::orin::routing_map::LightStatus::RED_LIGHT);
        break;
      }
      case BevTrafficLightState::TL_COLOR_YELLOW_FLASH: {
        adapted_lane_info->set_light_status(byd::msg::orin::routing_map::LightStatus::YELLOW_BLINKING);
        break;
      }
      case BevTrafficLightState::TL_COLOR_BLURRING_MODE: {
        adapted_lane_info->set_light_status(byd::msg::orin::routing_map::LightStatus::BLURRING_MODE);
        break;
      }
      case BevTrafficLightState::TL_COLOR_BLOCK_FAILED: {
        adapted_lane_info->set_light_status(byd::msg::orin::routing_map::LightStatus::BLOCK_FAIL);
        break;
      }
      default: {
        adapted_lane_info->set_light_status(byd::msg::orin::routing_map::LightStatus::UNKNOWN_LIGHT);
        break;
      }
    }

    for (const auto &point : lane.line_points) {
      int size_before = adapted_lane_info->points_size();
      AdaptInternal2Output(point, adapted_lane_info->add_points());
      int size_after = adapted_lane_info->points_size();
      if (size_after > size_before) {
        auto *add_point = adapted_lane_info->mutable_points(size_after - 1);
        if (!point.scatter_visibility) {
          add_point->set_point_source(byd::msg::orin::routing_map::PointSource::PS_INVISIBLE);
          // AINFO << "input point: (" << add_point->x() << ", " << add_point->y() << "); " << adapted_lane_info->id();
          // AINFO << "************************";
        }
      }
    }

    adapted_lane_info->set_length(lane.length);
    adapted_lane_info->set_speed_limit(lane.speed_limit);
    adapted_lane_info->set_is_virtual(lane.is_mounted);
    adapted_lane_info->set_light_countdown(lane.traffic_light_num);

    for (auto next_lane_id : lane.next_lane_ids) {
      adapted_lane_info->add_next_lane_ids(next_lane_id);
    }
    for (auto previous_lane_id : lane.previous_lane_ids) {
      adapted_lane_info->add_previous_lane_ids(previous_lane_id);
    }
    adapted_lane_info->set_connect_score(lane.connect_score);
    adapted_lane_info->set_is_blocked(lane.is_blocked);
    // adapted_lane_info->set_direction(static_cast<byd::msg::env_model::Direction>((int)lane.direction));
    auto *restricted_info = adapted_lane_info->mutable_restricted_info();
    restricted_info->set_restricted_type(static_cast<byd::msg::orin::routing_map::RestrictedType>((int)lane.restricted_info.restricted_type));
    restricted_info->set_is_passable(lane.restricted_info.is_passable);
    restricted_info->set_passable_env_state(lane.restricted_info.passable_env_state);
  }

  adapted_fusion_map->clear_lane_boundaries();
  adapted_fusion_map->mutable_lane_boundaries()->Reserve(bev_map_ptr->lanemarkers.size() + bev_map_ptr->diversion_zone.size());
  std::vector<uint32_t> route_lanemarkers_id;
  for (const auto &section : sections) {
    for (const auto &lanemarker : section.lanemarkers) {
      if (std::find(route_lanemarkers_id.begin(), route_lanemarkers_id.end(), lanemarker.id) != route_lanemarkers_id.end()) {
        continue;
      }
      route_lanemarkers_id.push_back(lanemarker.id);
      auto *adapted_lane_boundary = adapted_fusion_map->add_lane_boundaries();
      adapted_lane_boundary->set_id(lanemarker.id);
      for (const auto &point : lanemarker.line_points) {
        AdaptInternal2Output(point, adapted_lane_boundary->add_points());
        AdaptInternal2Output(point, adapted_lane_boundary->add_line_points());
      }
      for (const auto &color_seg : lanemarker.color_segs) {
        byd::msg::orin::routing_map::LaneBoundaryInfo_ColorSegment adapted_color_segment;
        switch (color_seg.color) {
          case BevLaneMarkerColor::BEV_LMC__WHITE: {
            adapted_color_segment.set_color(
                byd::msg::orin::routing_map::LaneBoundaryInfo_LaneMarkerColor::LaneBoundaryInfo_LaneMarkerColor_LM_COLOR_WHITE);
            break;
          }
          case BevLaneMarkerColor::BEV_LMC__YELLOW: {
            adapted_color_segment.set_color(
                byd::msg::orin::routing_map::LaneBoundaryInfo_LaneMarkerColor::LaneBoundaryInfo_LaneMarkerColor_LM_COLOR_YELLOW);
            break;
          }
          case BevLaneMarkerColor::BEV_LMC__UNKNOWN: {
            adapted_color_segment.set_color(
                byd::msg::orin::routing_map::LaneBoundaryInfo_LaneMarkerColor::LaneBoundaryInfo_LaneMarkerColor_LM_COLOR_UNKNOWN);
            break;
          }
          case BevLaneMarkerColor::BEV_LMC__BLUE: {
            adapted_color_segment.set_color(
                byd::msg::orin::routing_map::LaneBoundaryInfo_LaneMarkerColor::LaneBoundaryInfo_LaneMarkerColor_LM_COLOR_BLUE);
            break;
          }
          case BevLaneMarkerColor::BEV_LMC__GREEN: {
            adapted_color_segment.set_color(
                byd::msg::orin::routing_map::LaneBoundaryInfo_LaneMarkerColor::LaneBoundaryInfo_LaneMarkerColor_LM_COLOR_GREEN);
            break;
          }
          case BevLaneMarkerColor::BEV_LMC__RED: {
            adapted_color_segment.set_color(
                byd::msg::orin::routing_map::LaneBoundaryInfo_LaneMarkerColor::LaneBoundaryInfo_LaneMarkerColor_LM_COLOR_RED);
            break;
          }
          default: {
            break;
          }
        }
        adapted_color_segment.set_start_index(color_seg.start_index);
        adapted_color_segment.set_end_index(color_seg.end_index);
        adapted_lane_boundary->add_color_seg()->CopyFrom(adapted_color_segment);
      }
      switch (lanemarker.type) {
        case 0: {
          adapted_lane_boundary->set_line_type(byd::msg::orin::routing_map::LaneBoundaryInfo::LineType::LaneBoundaryInfo_LineType_UNKNOWN);
          break;
        }
        case 1: {
          adapted_lane_boundary->set_line_type(byd::msg::orin::routing_map::LaneBoundaryInfo::LineType::LaneBoundaryInfo_LineType_DASHED);
          break;
        }
        case 2: {
          adapted_lane_boundary->set_line_type(byd::msg::orin::routing_map::LaneBoundaryInfo::LineType::LaneBoundaryInfo_LineType_SOLID);
          break;
        }
        case 3: {
          adapted_lane_boundary->set_line_type(
              byd::msg::orin::routing_map::LaneBoundaryInfo::LineType::LaneBoundaryInfo_LineType_DASHED_DASHED);
          break;
        }
        case 4: {
          adapted_lane_boundary->set_line_type(
              byd::msg::orin::routing_map::LaneBoundaryInfo::LineType::LaneBoundaryInfo_LineType_SOLID_SOLID);
          break;
        }
        case 5: {
          adapted_lane_boundary->set_line_type(
              byd::msg::orin::routing_map::LaneBoundaryInfo::LineType::LaneBoundaryInfo_LineType_DASHED_SOLID);
          break;
        }
        case 6: {
          adapted_lane_boundary->set_line_type(
              byd::msg::orin::routing_map::LaneBoundaryInfo::LineType::LaneBoundaryInfo_LineType_SOLID_DASHED);
          break;
        }
        case 9: {
          adapted_lane_boundary->set_line_type(
              byd::msg::orin::routing_map::LaneBoundaryInfo::LineType::LaneBoundaryInfo_LineType_VIRTUAL_LANE);
          break;
        }
        default: {
          break;
        }
      }
    }
  }

  for (const auto &lanemarker : bev_map_ptr->lanemarkers) {
    if (std::find(route_lanemarkers_id.begin(), route_lanemarkers_id.end(), lanemarker.id) != route_lanemarkers_id.end()) {
      continue;
    }
    auto *adapted_lane_boundary = adapted_fusion_map->add_lane_boundaries();
    adapted_lane_boundary->set_id(lanemarker.id);
    for (const auto &point : lanemarker.line_points) {
      AdaptInternal2Output(point, adapted_lane_boundary->add_points());
      AdaptInternal2Output(point, adapted_lane_boundary->add_line_points());
    }
    for (const auto &color_seg : lanemarker.color_segs) {
      byd::msg::orin::routing_map::LaneBoundaryInfo_ColorSegment adapted_color_segment;
      switch (color_seg.color) {
        case BevLaneMarkerColor::BEV_LMC__WHITE:
          adapted_color_segment.set_color(
              byd::msg::orin::routing_map::LaneBoundaryInfo_LaneMarkerColor::LaneBoundaryInfo_LaneMarkerColor_LM_COLOR_WHITE);
          break;
        case BevLaneMarkerColor::BEV_LMC__YELLOW:
          adapted_color_segment.set_color(
              byd::msg::orin::routing_map::LaneBoundaryInfo_LaneMarkerColor::LaneBoundaryInfo_LaneMarkerColor_LM_COLOR_YELLOW);
          break;
        case BevLaneMarkerColor::BEV_LMC__UNKNOWN:
          adapted_color_segment.set_color(
              byd::msg::orin::routing_map::LaneBoundaryInfo_LaneMarkerColor::LaneBoundaryInfo_LaneMarkerColor_LM_COLOR_UNKNOWN);
          break;
        case BevLaneMarkerColor::BEV_LMC__BLUE:
          adapted_color_segment.set_color(
              byd::msg::orin::routing_map::LaneBoundaryInfo_LaneMarkerColor::LaneBoundaryInfo_LaneMarkerColor_LM_COLOR_BLUE);
          break;
        case BevLaneMarkerColor::BEV_LMC__GREEN:
          adapted_color_segment.set_color(
              byd::msg::orin::routing_map::LaneBoundaryInfo_LaneMarkerColor::LaneBoundaryInfo_LaneMarkerColor_LM_COLOR_GREEN);
          break;
        case BevLaneMarkerColor::BEV_LMC__RED:
          adapted_color_segment.set_color(
              byd::msg::orin::routing_map::LaneBoundaryInfo_LaneMarkerColor::LaneBoundaryInfo_LaneMarkerColor_LM_COLOR_RED);
          break;
        default:
          break;
      }
      adapted_color_segment.set_start_index(color_seg.start_index);
      adapted_color_segment.set_end_index(color_seg.end_index);
      adapted_lane_boundary->add_color_seg()->CopyFrom(adapted_color_segment);
    }
    switch (lanemarker.type) {
      case 1: {
        adapted_lane_boundary->set_line_type(byd::msg::orin::routing_map::LaneBoundaryInfo::LineType::LaneBoundaryInfo_LineType_DASHED);
        break;
      }
      case 2: {
        adapted_lane_boundary->set_line_type(byd::msg::orin::routing_map::LaneBoundaryInfo::LineType::LaneBoundaryInfo_LineType_SOLID);
        break;
      }
      case 3: {
        adapted_lane_boundary->set_line_type(
            byd::msg::orin::routing_map::LaneBoundaryInfo::LineType::LaneBoundaryInfo_LineType_DASHED_DASHED);
        break;
      }
      case 4: {
        adapted_lane_boundary->set_line_type(
            byd::msg::orin::routing_map::LaneBoundaryInfo::LineType::LaneBoundaryInfo_LineType_SOLID_SOLID);
        break;
      }
      case 5: {
        adapted_lane_boundary->set_line_type(
            byd::msg::orin::routing_map::LaneBoundaryInfo::LineType::LaneBoundaryInfo_LineType_DASHED_SOLID);
        break;
      }
      case 6: {
        adapted_lane_boundary->set_line_type(
            byd::msg::orin::routing_map::LaneBoundaryInfo::LineType::LaneBoundaryInfo_LineType_SOLID_DASHED);
        break;
      }
      case 9: {
        adapted_lane_boundary->set_line_type(
            byd::msg::orin::routing_map::LaneBoundaryInfo::LineType::LaneBoundaryInfo_LineType_VIRTUAL_LANE);
        break;
      }
      default: {
        adapted_lane_boundary->set_line_type(byd::msg::orin::routing_map::LaneBoundaryInfo::LineType::LaneBoundaryInfo_LineType_UNKNOWN);
        break;
      }
    }
    if (lanemarker.is_virtual) {
      adapted_lane_boundary->set_line_type(byd::msg::orin::routing_map::LaneBoundaryInfo::LineType::LaneBoundaryInfo_LineType_VIRTUAL_LANE);
    }
  }

  for (const auto &diversion : bev_map_ptr->diversion_zone) {
    auto *adapted_lane_boundary = adapted_fusion_map->add_lane_boundaries();
    adapted_lane_boundary->set_id(diversion.id);
    for (const auto &point : diversion.line_points) {
      AdaptInternal2Output(point, adapted_lane_boundary->add_points());
    }
    byd::msg::orin::routing_map::LaneBoundaryInfo_ColorSegment adapted_color_segment;
    adapted_color_segment.set_color(
        byd::msg::orin::routing_map::LaneBoundaryInfo_LaneMarkerColor::LaneBoundaryInfo_LaneMarkerColor_LM_COLOR_WHITE);
    adapted_color_segment.set_start_index(0);
    adapted_color_segment.set_end_index(diversion.line_points.size() - 1);
    adapted_lane_boundary->add_color_seg()->CopyFrom(adapted_color_segment);
    adapted_lane_boundary->set_line_type(byd::msg::orin::routing_map::LaneBoundaryInfo::LineType::LaneBoundaryInfo_LineType_OTHERS);
  }

  adapted_fusion_map->clear_road_boundaries();
  adapted_fusion_map->mutable_road_boundaries()->Reserve(bev_map_ptr->edges.size());
  for (const auto &edge : bev_map_ptr->edges) {
    auto *adapted_road_boundary = adapted_fusion_map->add_road_boundaries();
    adapted_road_boundary->set_id(edge.id);
    adapted_road_boundary->mutable_points()->Reserve(edge.line_points.size());
    for (const auto &point : edge.line_points) {
      AdaptInternal2Output(point, adapted_road_boundary->add_points());
    }
    switch (edge.type) {
      case 1:
        adapted_road_boundary->set_boundary_type(
            byd::msg::orin::routing_map::RoadBoundaryInfo::BoundaryType::RoadBoundaryInfo_BoundaryType_FLAT_BOUNDARY);
        break;
      case 2:
        adapted_road_boundary->set_boundary_type(
            byd::msg::orin::routing_map::RoadBoundaryInfo::BoundaryType::RoadBoundaryInfo_BoundaryType_LOW_BOUNDARY);
        break;
      case 3:
        adapted_road_boundary->set_boundary_type(
            byd::msg::orin::routing_map::RoadBoundaryInfo::BoundaryType::RoadBoundaryInfo_BoundaryType_HIGH_BOUNDARY);
        break;
      case 4:
        adapted_road_boundary->set_boundary_type(
            byd::msg::orin::routing_map::RoadBoundaryInfo::BoundaryType::RoadBoundaryInfo_BoundaryType_FENCE_BOUNDARY);
        break;
      case 12:
        adapted_road_boundary->set_boundary_type(
            byd::msg::orin::routing_map::RoadBoundaryInfo::BoundaryType::RoadBoundaryInfo_BoundaryType_VIRTUAL);
        break;
      default:
        adapted_road_boundary->set_boundary_type(
            byd::msg::orin::routing_map::RoadBoundaryInfo::BoundaryType::RoadBoundaryInfo_BoundaryType_UNKNOWN_BOUNDARY);
        break;
    }
  }

  adapted_fusion_map->clear_stop_lines();
  if (!stop_lines_ptr.empty()) {
    adapted_fusion_map->mutable_stop_lines()->Reserve(stop_lines_ptr.size());
    for (size_t i = 0; i < stop_lines_ptr.size(); i++) {
      auto *adapted_stop_line_info = adapted_fusion_map->add_stop_lines();
      adapted_stop_line_info->set_id(stop_lines_ptr[i].id);
      adapted_stop_line_info->mutable_points()->Reserve(stop_lines_ptr[i].dr_line_points.size());
      for (const auto &point : stop_lines_ptr[i].dr_line_points) {
        AdaptInternal2Output(point, adapted_stop_line_info->add_points());
      }
    }
  }

  adapted_fusion_map->clear_junctions();
  for (const auto &junction : bev_map_ptr->junctions) {
    byd::msg::orin::routing_map::JunctionInfo adapted_junction;
    adapted_junction.set_id(static_cast<uint64_t>(junction.id));
    for (const auto &pt : junction.line_points) {
      byd::msg::basic::Point2D adapted_point;
      adapted_point.set_x(static_cast<double>(pt.x));
      adapted_point.set_y(static_cast<double>(pt.y));
      adapted_junction.add_points()->CopyFrom(adapted_point);
    }
    adapted_fusion_map->add_junctions()->CopyFrom(adapted_junction);
  }

  adapted_fusion_map->clear_cross_walks();
  for (const auto &crosswalk : bev_map_ptr->crosswalks) {
    byd::msg::orin::routing_map::CrossWalkInfo adapted_crosswalk;
    adapted_crosswalk.set_id(static_cast<uint64_t>(crosswalk.id));
    for (const auto &pt : crosswalk.line_points) {
      byd::msg::basic::Point2D adapted_point;
      adapted_point.set_x(static_cast<double>(pt.x));
      adapted_point.set_y(static_cast<double>(pt.y));
      adapted_crosswalk.add_points()->CopyFrom(adapted_point);
    }
    adapted_fusion_map->add_cross_walks()->CopyFrom(adapted_crosswalk);
  }

  adapted_fusion_map->clear_route();
  adapted_fusion_map->mutable_route()->set_id(bev_map_ptr->route.id);
  adapted_fusion_map->mutable_route()->mutable_navi_start()->set_section_id(bev_map_ptr->route.navi_start.section_id);
  adapted_fusion_map->mutable_route()->mutable_navi_start()->set_s_offset(bev_map_ptr->route.navi_start.s_offset);

  for (const auto &section : bev_map_ptr->route.sections) {
    auto *adapted_section = adapted_fusion_map->mutable_route()->add_sections();
    adapted_section->set_id(section.id);
    adapted_section->set_length(section.length);
    adapted_section->mutable_lane_ids()->Reserve(section.lane_infos.size());
    for (size_t i = 0; i < section.lane_infos.size(); i++) {
      adapted_section->mutable_lane_ids()->Add(section.lane_infos.at(i).id);
    }
  }

  for (const auto &subpath : bev_map_ptr->route.subpaths) {
    auto *adapted_subpath = adapted_fusion_map->mutable_route()->add_subpaths();
    adapted_subpath->set_enter_section_id(subpath.enter_section_id);
    for (const auto &section : subpath.sections) {
      auto *adapted_section = adapted_subpath->add_sections();
      adapted_section->set_id(section.id);
      adapted_section->set_length(section.length);
      adapted_section->mutable_lane_ids()->Reserve(section.lane_infos.size());
      for (size_t i = 0; i < section.lane_infos.size(); i++) {
        adapted_section->mutable_lane_ids()->Add(section.lane_infos.at(i).id);
      }
    }
  }

  adapted_fusion_map->set_cur_road_class(static_cast<byd::msg::orin::routing_map::RoadClass>(bev_map_ptr->road_class));

  // 转换车道的指示箭头arrows
  adapted_fusion_map->clear_arrows();
  adapted_fusion_map->mutable_arrows()->Reserve(bev_map_ptr->arrows.size());
  for (const auto &lane_marker : bev_map_ptr->arrows) {
    byd::msg::orin::routing_map::ArrowInfo adapted_arrow;
    adapted_arrow.set_id(static_cast<uint64_t>(lane_marker.id));
    switch (lane_marker.type) {
      case 11:
        adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::RM_TYPE_STRAIGHT);
        break;
      case 12:
        adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::RM_TYPE_LEFT);
        break;
      case 13:
        adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::RM_TYPE_RIGHT);
        break;
      case 14:
        adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::RM_TYPE_TURNING);
        break;
      case 15:
        adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::RM_TYPE_STRAIGHT_LEFT);
        break;
      case 16:
        adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::RM_TYPE_STRAIGHT_RIGHT);
        break;
      case 17:
        adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::RM_TYPE_STRAIGHT_LEFT_RIGHT);
        break;
      case 18:
        adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::RM_TYPE_LEFT_RIGHT);
        break;
      case 19:
        adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::RM_TYPE_STRAIGHT_TURNING);
        break;
      case 20:
        adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::RM_TYPE_LEFT_TURNING);
        break;
      case 21:
        adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::ArrowType_MIN);
        break;
      case 22:
        adapted_arrow.set_type(byd::msg::orin::routing_map::ArrowInfo::ArrowType_MAX);
        break;
      default:
        break;
    }
    for (const auto pt : lane_marker.line_points) {
      byd::msg::basic::Point3D adapted_point;
      adapted_point.set_x(static_cast<double>(pt.x));
      adapted_point.set_y(static_cast<double>(pt.y));
      adapted_point.set_z(0.0);
      adapted_arrow.add_bounding_box_geometry()->CopyFrom(adapted_point);
    }
    adapted_fusion_map->add_arrows()->CopyFrom(adapted_arrow);
  }

  ///将导流区赋值到arrows字段
  // adapted_fusion_map->mutable_arrows()->Reserve(bev_map_ptr->diversion_zone.size());
  // for(auto& diversion_tmp : bev_map_ptr->diversion_zone) {
  //   byd::msg::orin::routing_map::ArrowInfo adapted_arrow;
  //   adapted_arrow.set_id(static_cast<uint64_t>(diversion_tmp.id * 1e3));
  //   for (const auto pt : diversion_tmp.line_points) {
  //     byd::msg::basic::Point3D adapted_point;
  //     adapted_point.set_x(static_cast<double>(pt.x));
  //     adapted_point.set_y(static_cast<double>(pt.y));
  //     adapted_point.set_z(0.0);
  //     adapted_arrow.add_bounding_box_geometry()->CopyFrom(adapted_point);
  //   }
  //   adapted_fusion_map->add_arrows()->CopyFrom(adapted_arrow);
  // }

  // 转换 EnvInfo
  adapted_fusion_map->clear_env_info();
  auto *proto_env_info = adapted_fusion_map->mutable_env_info();
  AdaptInternal2Output(bev_map_ptr->env_info, proto_env_info);  // 调用适配 EnvInfo 的函数

  adapted_fusion_map->clear_sensor_status_info();
  AdaptInternal2Output(sensorStatus_info, adapted_fusion_map->mutable_sensor_status_info());
  return;
}
void AdaptInternal2Output(const cem::message::env_model::Trajectory &raw_exp_trajectory,
                          byd::msg::orin::routing_map::Trajectory   *adapted_exp_trajectory) {
  adapted_exp_trajectory->set_id(raw_exp_trajectory.id);
  adapted_exp_trajectory->set_lane_id(raw_exp_trajectory.lane_id);
  adapted_exp_trajectory->set_start_lane_id(raw_exp_trajectory.start_lane_id);
  adapted_exp_trajectory->set_end_lane_id(raw_exp_trajectory.end_lane_id);
  adapted_exp_trajectory->clear_relative_lane_id();
  adapted_exp_trajectory->mutable_relative_lane_id()->Reserve(raw_exp_trajectory.relative_lane_id.size());
  for (const auto &relative_lane_id : raw_exp_trajectory.relative_lane_id) {
    adapted_exp_trajectory->add_relative_lane_id(relative_lane_id);
  }
  adapted_exp_trajectory->clear_points();
  adapted_exp_trajectory->mutable_points()->Reserve(raw_exp_trajectory.points.size());
  for (const auto &point : raw_exp_trajectory.points) {
    AdaptInternal2Output(point, adapted_exp_trajectory->add_points());
  }
}
#endif

}  // namespace fusion
}  // namespace cem
