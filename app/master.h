#ifndef MASTER_H_
#define MASTER_H_

#include "base/params_manager/params_manager.h"
#include "communication/messenger.h"
#include "lib/base/landmark_buffer.h"
#include "lib/common/debug_infos/debug_infos.h"
#include "lib/localmap_construction/crossroad_construction/crossroad_construction.h"
#include "lib/localmap_construction/routing_map_stopline_processor.h"
#include "lib/localmap_construction/stopline_mapping.h"
#include "lib/localmap_construction/traffic_flow_mapping.h"
#include "lib/localmap_construction/traffic_light_mapping.h"
#include "lib/map_fusion/extend_lane.h"
#include "lib/map_fusion/lane_guidance.h"
#include "lib/map_fusion/speed_limit_fusion.h"
#include "lib/perception_and_ld_map_fusion/data_fusion/geometry_match_info.h"
#include "lib/perception_and_ld_map_fusion/fusion_manager.h"
#include "lib/perception_and_ld_map_fusion/map_source_switch/map_source_switch.h"
#include "lib/pre_processor/pre_processor.h"
#include "lib/sd_navigation/EnvInfoProcessor.h"
#include "lib/sd_navigation/SdNavigationBase.h"
#include "lib/sd_navigation/SdNavigationCity.h"
#include "lib/sd_navigation/SdNavigationHighway.h"
#include "master_interface.h"
#include "modules/msg/localization_msgs/map_match.pb.h"

namespace cem {
namespace fusion {
using cem::message::sensor::MapType;

class Master : public MasterInterface {
 public:
  std::unique_ptr<Messenger> messenger_;

 private:
  std::unique_ptr<PreProcessor>                        pre_processor_;
  std::unique_ptr<StoplineMapping>                     stopline_mapping_;
  std::unique_ptr<CrossWalkTracker>                    crosswalk_mapping_;
  std::unique_ptr<TrafficLightMapping>                 traffic_light_mapping_;
  std::unique_ptr<RoutingMapStopLineProcessor>         routing_map_stopline_processor_;
  std::unique_ptr<FusionManager>                       fusion_manager_;
  std::shared_ptr<LaneGuidance>                        lane_guidance_;
  std::unique_ptr<navigation::EnvInfoProcessor>        env_info_processor_;
  std::unique_ptr<CrossRoadConstruction>               cross_road_construction_;
  std::unique_ptr<cem::fusion::extendLane::ExtendLane> extend_lane_;
  std::unique_ptr<DebugInfos>                          debug_infos_;
  std::shared_ptr<LandmarkBuffer>                      landmark_buffer_;
  std::unique_ptr<SpeedLimitFusion>                    speed_limit_fusion_;
  std::unique_ptr<CnoaSpeedLimit>                      cnoa_speed_limit_;
  std::unique_ptr<MapSourceSwitch>                     map_source_switch_;

  GeometryMatchInfo   bev_ld_map_geometry_match_info_;  // geometry match info(bev match ld)

  std::string conf_abs_path_;

  std::shared_ptr<navigation::SdNavigationBase>    sd_navigation_base_;
  std::shared_ptr<navigation::SdNavigationHighway> sd_navigation_highway_;
  std::shared_ptr<navigation::SdNavigationCity>    sd_navigation_city_;
  std::shared_ptr<byd::msg::orin::routing_map::EnvStatus> env_status_{};

 public:
  Master(const std::string &confdir, const std::string &name);
  ~Master() = default;

  void Init();

  virtual void Proc();

  virtual std::shared_ptr<byd::msg::basic::ModuleStatus> GetModuleStatusPtr();

  virtual std::shared_ptr<byd::msg::orin::routing_map::RoutingMap> GetENVRoutingMapPtr();

  virtual std::shared_ptr<byd::msg::orin::routing_map::TrafficLightsE2EInfo> GetTrafficLightsE2EPtr();

  /********************接收消息callback在此处定义****************************/
  virtual void OnVehInfoCallback(const std::shared_ptr<byd::msg::drivers::VehInfo> &msg);

  virtual void OnEnvModelLocalMapInfoCallback(const std::shared_ptr<byd::msg::env_model::LocalMapInfo> &msg);

  virtual void OnMsgDriverInsCallback(const std::shared_ptr<byd::msg::drivers::Ins> &msg);

  virtual void OnMsgLocationDrCallback(const std::shared_ptr<byd::msg::localization::LocalizationEstimate> &msg);

  virtual void OnPercepTrfInfoCallback(const std::shared_ptr<byd::msg::perception::PerceptionTrafficInfo> &msg);

  virtual void OnVisionTrfInfoCallback(const std::shared_ptr<byd::msg::perception::PerceptionTrafficInfo> &msg);

  virtual void OnMsgMapEventCallback(const std::shared_ptr<byd::msg::orin::routing_map::MapEvent> &msg);

  virtual void OnRoutingMapCallBack(const std::shared_ptr<byd::msg::orin::routing_map::RoutingMap> &msg);

  virtual void OnPerceptionObstaclesInfoCallback(const std::shared_ptr<byd::msg::perception::PerceptionObstacles> &msg);

  virtual void OnEnvModelLidaRoadedgeCallback(const std::shared_ptr<byd::msg::env_model::LocalMapInfo> &msg);

  virtual void OnPlanResultCallback(const std::shared_ptr<byd::msg::planning::PLanningResultProto> &msg);

  virtual void OnPlanFuncStateCallback(const std::shared_ptr<byd::msg::pnc::PlanFuncState> &msg);

  virtual void OncanoutCallback(const std::shared_ptr<byd::msg::orin::sm_msgs::MsgSM_BYD_CAN_Output>& msg);

  virtual void OnOccCallback(const std::shared_ptr<byd::msg::perception::OCCInfo>& msg);

  virtual void OnNaviTrafCallback(const std::shared_ptr<byd::msg::drivers::TrafficInfoNotify> &msg);

  virtual void OnDriversEventCallback(const std::shared_ptr<byd::msg::drivers::Event> &msg);

  void OnSDTrafficLightCallback(const std::shared_ptr<byd::msg::drivers::sdTrafficLight>& msg) final;

  void OnMsgObstaclesCallback(const std::shared_ptr<byd::msg::pnc::ObjInfoFusn> &msg) final;

  void OnMapLocationResultCallback(const std::shared_ptr<byd::modules::localization::MapMatchResult> &msg) final;

  void OnMapLocationResultBaiduCallback(
      const std::shared_ptr<byd::modules::localization::MapMatchResult> &msg)
      final;

 private:
  void LoadConfig();
  void InitMember();

  /**
  * @brief MapType toggle when car transport junction.
  * @return MapConvertType
  * 
  * @author lingpeng (ling.peng3@byd.com)
  * @date 2025-03-12
  */
  MapConvertType MapToggleMode(GeometryMatchResult &match_detail);

 bool GetIsOnHighwayFlag(const RoutingMapPtr& routing_map_raw);

};

}  // namespace fusion
}  // namespace cem

#endif
