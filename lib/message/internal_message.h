#ifndef INTERNAL_MESSAGE_H_
#define INTERNAL_MESSAGE_H_

#include <memory>

// common message
#include "message/common/cem_header.h"
#include "message/common/geometry.h"
#include "message/common/header.h"

// env_model message
#include "message/env_model/traffic_light/traffic_light_info.h"
#include "message/env_model/traffic_light/traffic_light.h"
#include "message/env_model/traffic_road/geometry_line.h"
#include "message/env_model/traffic_road/lane_marker.h"
#include "message/env_model/traffic_road/road_edge.h"

// sensor message
#include "message/sensor/localization/localization_data.h"
#include "message/sensor/vehicle/vehicle_signal.h"
#include "message/sensor/vehicle/bcm50ms_pdu02.h"
#include "message/sensor/vehicle/ibs20ms_pdu08.h"
#include "message/sensor/vehicle/ibs20ms_pdu15.h"
#include "message/sensor/vehicle/sas10ms_pdu00.h"
#include "message/sensor/vehicle/adas_ipd100ms_pdu12.h"
#include "message/sensor/vehicle/imcu20ms_pdu05.h"
#include "message/sensor/vehicle/icm100ms_pdu29.h"
#include "message/sensor/vehicle/ap_ipd100ms_pud12.h"
#include "message/sensor/vehicle/eps010ms_pdu00.h"
#include "message/sensor/vehicle/mpd100ms_pdu04.h"
#include "message/sensor/vision/tsrmobject.h"
#include "message/sensor/camera/bev_lane/bev_lane.h"
#include "message/sensor/vision/radar_vision_obstacles.h"
#include "message/sensor/localization/map_match.h"
#include "message/env_model/routing_map/routing_map.h"
#include "message/env_model/routing_map/map_event.h"
#include "message/env_model/navigation/navigation.h"
#include "message/env_model/pnc/planning_result.h"
#include "message/env_model/pnc/plan_func_state.h"
#include "message/env_model/pnc/canout.h"
#include "message/env_model/occ/occ.h"
#include "modules/msg/drivers_msgs/sdmap_inform_service.pb.h"
#include "modules/msg/localization_msgs/map_match.pb.h"
#include "message/env_model/speed_limit/navi_traf.h"
#include "message/env_model/speed_limit/drivers_event.h"

namespace cem {
namespace fusion {

using namespace cem::message::common;
using namespace cem::message::env_model;
using namespace cem::message::sensor;
using namespace cem::env_model::occ;

// internal message type alias
typedef std::shared_ptr<LocalizationData> LocalizationPtr;
typedef std::shared_ptr<const LocalizationData> LocalizationConstPtr;
typedef std::shared_ptr<VehicleSignal> VehicleSignalPtr;
typedef std::shared_ptr<const VehicleSignal> VehicleSignalConstPtr;
typedef std::shared_ptr<Bcm50msPdu02> Bcm50msPdu02Ptr;
typedef std::shared_ptr<const Bcm50msPdu02> BcmPdu02ConstPtr;
typedef std::shared_ptr<Ibs20msPdu08> Ibs20msPdu08Ptr;
typedef std::shared_ptr<const Ibs20msPdu08> Ibs20msPdu08ConstPtr;
typedef std::shared_ptr<Ibs20msPdu15> Ibs20msPdu15Ptr;
typedef std::shared_ptr<const Ibs20msPdu15> Ibs20msPdu15ConstPtr;
typedef std::shared_ptr<Sas10msPdu00> Sas10msPdu00Ptr;
typedef std::shared_ptr<const Sas10msPdu00> Sas10msPdu00ConstPtr;
typedef std::shared_ptr<AdasIpd100msPdu12> AdasIpd100msPdu12Ptr;
typedef std::shared_ptr<const AdasIpd100msPdu12> AdasIpd100msPdu12ConstPtr;
typedef std::shared_ptr<Mpd100msPdu04> Mpd100msPdu04Ptr;
typedef std::shared_ptr<const Mpd100msPdu04> Mpd100msPdu04ConstPtr;
typedef std::shared_ptr<Imcu20msPdu05> Imcu20msPdu05Ptr;
typedef std::shared_ptr<const Imcu20msPdu05> Imcu20msPdu05ConstPtr;
typedef std::shared_ptr<Eps010msPdu00> Eps010msPdu00Ptr;
typedef std::shared_ptr<const Eps010msPdu00> Eps010msPdu00ConstPtr;
typedef std::shared_ptr<Icm100msPdu29> Icm100msPdu29Ptr;
typedef std::shared_ptr<const Icm100msPdu29> Icm100msPdu29ConstPtr;
typedef std::shared_ptr<Tsrmobject> TsrmobjectPtr;
typedef std::shared_ptr<const Tsrmobject> TsrmobjectConstPtr;
typedef std::shared_ptr<BevMapInfo> BevMapInfoPtr;
typedef std::shared_ptr<const BevMapInfo> BevMapInfoConstPtr;
typedef std::shared_ptr<ApIpd100msPdu12> ApIpd100msPdu12Ptr;
typedef std::shared_ptr<const ApIpd100msPdu12> ApIpd100msPdu12ConstPtr;
typedef std::shared_ptr<PercepTrfInfo> PercepTrfInfoPtr;
typedef std::shared_ptr<const PercepTrfInfo> PercepTrfInfoConstPtr;
typedef std::shared_ptr<RoutingMap> RoutingMapPtr;
typedef std::shared_ptr<const RoutingMap> RoutingMapConstPtr;
typedef std::shared_ptr<MapEvent> MapEventPtr;
typedef std::shared_ptr<const MapEvent> MapEventConstPtr;
typedef std::shared_ptr<SDRouteInfo> SDRouteInfoPtr;
typedef std::shared_ptr<const SDRouteInfo> SDRouteInfoConstPtr;
typedef std::shared_ptr<FusObjInfo> FusObjInfoPtr;
typedef std::shared_ptr<const FusObjInfo> FusObjInfoConstPtr;
typedef std::shared_ptr<TrafficLightsE2EInfo> TrafficLightsE2EInfoPtr;
typedef std::shared_ptr<const TrafficLightsE2EInfo> TrafficLightsE2EInfoConstPtr;
typedef std::shared_ptr<LidarRoadEdgeInfo> LidarRoadEdgeInfoPtr;
typedef std::shared_ptr<const LidarRoadEdgeInfo> LidarRoadEdgeInfoConstPtr;
typedef std::shared_ptr<PLanningResultData> PLanningResultPtr;
typedef std::shared_ptr<PLanFuncState> PlanFuncStatePtr;
typedef std::shared_ptr<CAN1> CAN1Ptr;
typedef std::shared_ptr<const CAN1> CAN1ConstPtr;
typedef std::shared_ptr<OCCInfo> OCCInfoPtr;
typedef std::shared_ptr<const OCCInfo> OCCInfoConstPtr;
typedef std::shared_ptr<byd::msg::drivers::sdTrafficLight> SdTrafficLightPtr;
typedef std::shared_ptr<byd::modules::localization::MapMatchResult>  MapMatchResultPtr;
typedef std::shared_ptr<cem::message::sensor::MapMatchResultBaidu> MapMatchResultBaiduPtr;
typedef std::shared_ptr<VisionTrfInfo> VisionTrfInfoPtr;
typedef std::shared_ptr<const VisionTrfInfo> VisionTrfInfoConstPtr;
typedef std::shared_ptr<NaviTrafficInfo> NaviTrafficInfoPtr;
typedef std::shared_ptr<const NaviTrafficInfo> NaviTrafficInfoConstPtr;
typedef std::shared_ptr<DriversEvent> DriversEventPtr;
typedef std::shared_ptr<const DriversEvent> DriversEventConstPtr;
} // namespace fusion
} // namespace cem

#endif
