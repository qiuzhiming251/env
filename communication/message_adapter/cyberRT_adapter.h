#ifndef DDS_ADAPTER_H_
#define DDS_ADAPTER_H_
#include "common/utility.h"
#include "base/params_manager/params_manager.h"
#include "modules/msg/drivers_msgs/veh_info.pb.h"
#include "modules/msg/orin_msgs/lane_fusion_msgs.pb.h"
#include "modules/msg/drivers_msgs/ins.pb.h"
#include "modules/msg/basic_msgs/header.pb.h"
#include "modules/msg/environment_model_msgs/local_map_info.pb.h"
#include "modules/msg/localization_msgs/localization_info.pb.h"
#include "modules/msg/basic_msgs/module_status.pb.h"
#include "modules/msg/perception_msgs/perception_traffic_info.pb.h"
#include "modules/msg/environment_model_msgs/sr_element.pb.h"
#include "modules/msg/orin_msgs/routing_map.pb.h"
#include "modules/msg/orin_msgs/map_event.pb.h"
#include "message/internal_message.h"
#include "lib/message/env_model/stop_line/stop_line.h"
#include "lib/message/env_model/occ/occ.h"
#include "modules/msg/basic_msgs/geometry.pb.h"
#include "modules/msg/orin_msgs/sm_msgs.pb.h"
#include "modules/msg/orin_msgs/lane_fusion_msgs.pb.h"
#include "modules/msg/st_msgs/planning_result.pb.h"
#include "modules/msg/planning_msgs/plan_func_state.pb.h"
#include "modules/msg/drivers_msgs/navi_traffic_info.pb.h"
#include "modules/msg/drivers_msgs/event.pb.h"
#include "lib/message/sensor/localization/map_match.h"

namespace cem {
namespace fusion {

#if defined(MW_TYPE_CyberRT)

void MergeSegs(BevLaneMarker *adapted_lane_marker);

void AdaptInput2Internal(
    const std::shared_ptr<byd::msg::drivers::VehInfo> &raw_veh_signal,
    VehicleSignalPtr adapted_veh_signal);

void AdaptInput2Internal(
    const std::shared_ptr<byd::msg::env_model::LocalMapInfo> &raw_local_map_info,
    BevMapInfoPtr adapted_bev_lane);

void AdaptInput2Internal(
    const std::shared_ptr<byd::msg::orin::routing_map::RoutingMap> &raw_map_info,
    RoutingMapPtr adapted_map_info);

// Point2D internal(map_info)
void AdaptInput2Internal(const byd::msg::basic::Point2D &raw_point2d, byd::msg::basic::Point2D *adapted_point);

// Point internal(map_info)
void AdaptInput2Internal(const byd::msg::orin::routing_map::Point &raw_point, Point *adapted_point);

// SectionInfo(map_info)
void AdaptInput2Internal(const byd::msg::orin::routing_map::SectionInfo &raw_section_info, SectionInfo *adapted_section_info);

// NaviPosition internal(map_info)
void AdaptInput2Internal(const byd::msg::orin::routing_map::NaviPosition &raw_position, NaviPosition *adapted_position);

// RouteInfo internal(map_info)
void AdaptInput2Internal(const byd::msg::orin::routing_map::RouteInfo &raw_route, RouteInfo *adapted_route);

// StopLineInfo internal
void AdaptInput2Internal(const byd::msg::orin::routing_map::StopLineInfo &raw_stop_line, StopLineInfo *adapted_stop_line);

// JunctionInfo internal
void AdaptInput2Internal(const byd::msg::orin::routing_map::JunctionInfo &raw_junction_info, JunctionInfo *adapted_junction_info);

// CrossWalkInfo internal
void AdaptInput2Internal(const byd::msg::orin::routing_map::CrossWalkInfo &raw_cross_walk, CrossWalkInfo *adapted_cross_walk);

// RoadBoundaryInfo internal(map_info)
void AdaptInput2Internal(const byd::msg::orin::routing_map::RoadBoundaryInfo &raw_road_bd, RoadBoundaryInfo *adapted_road_bd);

// LaneBoundaryInfo internal(map_info)
void AdaptInput2Internal(const byd::msg::orin::routing_map::LaneBoundaryInfo &raw_lane_bd, LaneBoundaryInfo *adapted_lane_bd);

// LaneInfo internal(map_info)
void AdaptInput2Internal(const byd::msg::orin::routing_map::LaneInfo &raw_lane_info, LaneInfo *adapted_lane_info);

// RoutingMap internal
void AdaptInput2Internal(const std::shared_ptr<byd::msg::orin::routing_map::RoutingMap> &raw_routing_map, RoutingMapPtr adapted_routing_map);

void AdaptInput2Internal(const byd::msg::orin::routing_map::SDLaneGroupInfo& proto_sd_lane_group,
    cem::message::env_model::SDLaneGroupInfo* internal_sd_lane_group);

// exp_trajectory internal
void AdaptInput2Internal(const byd::msg::orin::routing_map::Trajectory &proto_exp_trajectory,
                         cem::message::env_model::Trajectory           *internal_exp_trajectory);

void AdaptInput2Internal(const std::shared_ptr<byd::msg::drivers::TrafficInfoNotify> &raw_navi_traf,
                         std::shared_ptr<cem::message::env_model::NaviTrafficInfo> &adapted_navi_traf);

void AdaptInput2Internal(const std::shared_ptr<byd::msg::drivers::Event> &raw_drivers_event,
                         std::shared_ptr<cem::message::env_model::DriversEvent> &adapted_drivers_event);

// 声明 MapEvent 的适配函数
void AdaptInput2Internal(
    const std::shared_ptr<byd::msg::orin::routing_map::MapEvent>& raw_map_event,
    std::shared_ptr<cem::message::env_model::MapEvent>& adapted_map_event);

void AdaptInput2Internal(
    const std::shared_ptr<byd::msg::drivers::VehInfo> &veh_info_msg_ptr,
    Bcm50msPdu02Ptr adapted_bcm50ms_pdu02);

void AdaptInput2Internal(
    const std::shared_ptr<byd::msg::drivers::VehInfo>
        &veh_info_msg_ptr,
    Ibs20msPdu08Ptr adapted_ibs20ms_pdu08);

void AdaptInput2Internal(
    const std::shared_ptr<byd::msg::drivers::VehInfo>
        &veh_info_msg_ptr,
    Imcu20msPdu05Ptr adapted_imcu20ms_pdu05);

void AdaptInput2Internal(
    const std::shared_ptr<byd::msg::drivers::VehInfo>
        &veh_info_msg_ptr,
    Eps010msPdu00Ptr adapted_eps010ms_pdu00);

void AdaptInput2Internal(
    const std::shared_ptr<byd::msg::drivers::VehInfo> &veh_info_msg_ptr,
    Ibs20msPdu15Ptr adapted_ibs20ms_pdu15);

void AdaptInput2Internal(
    const std::shared_ptr<byd::msg::drivers::VehInfo> &veh_info_msg_ptr,
    Sas10msPdu00Ptr adapted_sas10ms_pdu00);

void AdaptInput2Internal(const std::shared_ptr<byd::msg::drivers::VehInfo> &veh_info_msg_ptr,
                         AdasIpd100msPdu12Ptr adapted_adas_ipd100ms_pdu12);

void AdaptInput2Internal(
    const std::shared_ptr<byd::msg::drivers::VehInfo> &veh_info_msg_ptr,
    Mpd100msPdu04Ptr adapted_mpd100ms_pdu04);

void AdaptInput2Internal(
    const std::shared_ptr<byd::msg::drivers::VehInfo>
        &veh_info_msg_ptr,
    Icm100msPdu29Ptr adapted_apicm100mspdu29);

void AdaptInput2Internal(const std::shared_ptr<byd::msg::drivers::Ins> &raw_loc,
                         LocalizationPtr adapted_loc);

void AdaptInput2Internal(const std::shared_ptr<byd::msg::localization::LocalizationEstimate> &raw_loc,
                         LocalizationPtr adapted_loc);

void AdaptInput2Internal(
    const std::shared_ptr<byd::msg::perception::PerceptionTrafficInfo>
        &raw_perception_traffic_info,
    PercepTrfInfoPtr adapted_percep_trf_info);

void AdaptInput2Internal(
    const std::shared_ptr<byd::msg::perception::PerceptionTrafficInfo>
        &raw_perception_traffic_info,
    VisionTrfInfoPtr adapted_vision_trf_info);

inline void MergeSegs(BevLaneMarker *adapted_lane_marker);

void AdaptInput2Internal(
    const byd::msg::basic::Point3D &raw_point,
    cem::message::common::Point3DD *adapt_point);

void AdaptInput2Internal(
    const byd::msg::orin::routing_map::TrafficLightInfo &raw_traffic_light,
    TrafficLightMap *adaped_traffic_light);

void AdaptInput2Internal(const byd::msg::orin::routing_map::EnvInfo& proto_env_info,
                          cem::message::env_model::EnvInfo* internal_env_info);

void AdaptInput2Internal(const byd::msg::orin::routing_map::SDRouteInfo& proto_sd_route_info,
                              cem::message::env_model::SDRouteInfo* internal_env_sd_route_info);

void AdaptInput2Internal(
    const std::shared_ptr<byd::msg::perception::PerceptionObstacles> &raw_perception_obstacles_info,
    FusObjInfoPtr adapted_fusion_obj_info);

void AdaptInput2Internal(
    const std::shared_ptr<byd::msg::env_model::LocalMapInfo> &raw_lidar_roadedge_info,
    LidarRoadEdgeInfoPtr adapted_lidar_roadedge_info);

void AdaptInput2Internal(const std::shared_ptr<byd::msg::planning::PLanningResultProto> &raw_planning_result,
                         PLanningResultPtr                                      adapted_planning_result_info);

void AdaptInput2Internal(const std::shared_ptr<byd::msg::pnc::PlanFuncState> &raw_plan_func_state,
                         PlanFuncStatePtr                                      adapted_plan_func_state_info);

void AdaptInput2Internal(
    const std::shared_ptr<byd::msg::orin::sm_msgs::MsgSM_BYD_CAN_Output> &raw_can1,
    CAN1Ptr adapted_can);
void AdaptInput2Internal(
    const std::shared_ptr<byd::msg::perception::OCCInfo> &msg,
    std::shared_ptr<cem::env_model::occ::OCCInfo> occInfo );

void AdaptInput2Internal(
    const std::shared_ptr<byd::modules::localization::MapMatchResult>
        &raw_map_match_result,
    std::shared_ptr<cem::message::sensor::MapMatchResultBaidu>
        adapted_map_match_result);

/******************************************************************************/

/*diagnotics*/
void AdaptInternal2Output(std::shared_ptr<byd::msg::basic::ModuleStatus> adpted_diagnostic_status);

/* FusionMap */
void AdaptInternal2Output(const cem::message::env_model::EnvInfo& internal_env_info,
                          byd::msg::orin::routing_map::EnvInfo* proto_env_info);

void AdaptInternal2Output(const BevMapInfoPtr bev_map_ptr,
    const std::vector<cem::message::env_model::StopLine> stop_lines_ptr,
    const SensorStatusInfo &sensorStatus_info,
    std::shared_ptr<byd::msg::orin::routing_map::RoutingMap> adapted_fusion_map,
    bool is_on_highway_flag);
// routing map output
void AdaptInternal2Output(
    RoutingMapPtr routingMap,
    std::shared_ptr<byd::msg::orin::routing_map::RoutingMap> adapted_routing_map,
    bool is_on_highway_flag,
    cem::fusion::navigation::AccLaneInfo  sd_acc_lane_info);
// TrafficLightsE2EInfo outputstd::shared_ptr<byd::msg::orin::routing_map::TrafficLightsE2EInfo>
void AdaptInternal2Output(const TrafficLightsE2EInfoPtr traffic_lights_e2e,
        std::shared_ptr<byd::msg::orin::routing_map::TrafficLightsE2EInfo> adapted_traffic_lights_e2e);
// map info output
void AdaptInternal2Output(const MapInfo& rawMapInfo,
    byd::msg::orin::routing_map::MapInfo *apapted_map_info);

// SensorStatusInfo output
void AdaptInternal2Output(const SensorStatusInfo& rawSensorStatusInfo,
    byd::msg::orin::routing_map::SensorStatusInfo *apapted_sensorstatus_info);

void AdaptInternal2Output(
    const LaneInfo &lane,
    byd::msg::orin::routing_map::LaneInfo *adapted_lane, cem::fusion::navigation::AccLaneInfo sd_acc_lane_info);

void AdaptInternal2Output(const LaneBoundaryInfo &raw_lane_bound,
                          byd::msg::orin::routing_map::LaneBoundaryInfo *adapted_lane_boundaries);

inline void AdaptInternal2Output(const Point &point, byd::msg::orin::routing_map::Point *adapted_point) {
  if (!adapted_point) {
    return;
  }
  adapted_point->set_x(point.x);
  adapted_point->set_y(point.y);
  adapted_point->set_z(point.z);
  adapted_point->set_curvature(point.curvature);
  adapted_point->set_mse(1.0);
  adapted_point->set_point_source(static_cast<byd::msg::orin::routing_map::PointSource>(cem::message::common::PointSource::PS_HDMAP));
}

inline void AdaptInternal2Output(const Point2D &raw_point2D, byd::msg::basic::Point2D *adapted_point2D) {
  adapted_point2D->set_x(raw_point2D.x);
  adapted_point2D->set_y(raw_point2D.y);
}

inline void AdaptInternal2Output(const cem::message::common::Point2DF &raw_point2DF,
                                 byd::msg::basic::Point2D *adapted_point2D) {
  if (!adapted_point2D) {
    return;
  }
  adapted_point2D->set_x(raw_point2DF.x);
  adapted_point2D->set_y(raw_point2DF.y);
}

inline void AdaptInternal2Output(const Point2D &raw_point2D,
                                byd::msg::orin::routing_map::Point *adapted_point){
    if (!adapted_point) {
        return;
    }
    adapted_point->set_x(raw_point2D.x);
    adapted_point->set_y(raw_point2D.y);
    adapted_point->set_mse(1.0);
    adapted_point->set_point_source(static_cast<byd::msg::orin::routing_map::PointSource>(cem::message::common::PointSource::PS_HDMAP));
}

inline void AdaptInternal2Output(const cem::message::common::Point2DF &raw_point2DF,
                                 byd::msg::orin::routing_map::Point *adapted_point) {
  adapted_point->set_x(raw_point2DF.x);
  adapted_point->set_y(raw_point2DF.y);
  adapted_point->set_mse(raw_point2DF.mse);
  adapted_point->set_point_source(static_cast<byd::msg::orin::routing_map::PointSource>(raw_point2DF.point_source));
}

void AdaptInternal2Output(
    const RoadBoundaryInfo& raw_road_boundary,
    byd::msg::orin::routing_map::RoadBoundaryInfo *adapted_roadBoundary);

void AdaptInternal2Output(
    const StopLineInfo& raw_stop_line,
    byd::msg::orin::routing_map::StopLineInfo *adapted_stop_line);

void AdaptInternal2Output(
    const JunctionInfo& junction,
    byd::msg::orin::routing_map::JunctionInfo *adapted_junction);

void AdaptInternal2Output(
    const CrossWalkInfo& cross,
    byd::msg::orin::routing_map::CrossWalkInfo *adapted_cross);

void AdaptInternal2Output(
    const RouteInfo& route,
    byd::msg::orin::routing_map::RouteInfo *adapted_route);

void AdaptInternal2Output(
    const SectionInfo& raw_route,
    byd::msg::orin::routing_map::SectionInfo *adapted_section);

void AdaptInternal2Output(
    const NaviPosition& naviPos,
    byd::msg::orin::routing_map::NaviPosition *adapted_navi_pos);

void AdaptInternal2Output(
    const cem::message::env_model::TrafficLightMap& raw_traffic_light,
    byd::msg::orin::routing_map::TrafficLightInfo *adapted_traffic_light);

void AdaptInternal2Output(
    const cem::message::env_model::Trajectory& raw_exp_trajectory,
    byd::msg::orin::routing_map::Trajectory* adapted_exp_trajectory);

inline void AdaptInternal2Output(const cem::message::common::Point3DD &raw_point3D,
                                 byd::msg::basic::Point3D *adapted_point3D) {
  adapted_point3D->set_x(raw_point3D.x);
  adapted_point3D->set_y(raw_point3D.y);
  adapted_point3D->set_z(raw_point3D.z);
}
#endif

} // namespace fusion
} // namespace cem

#endif
