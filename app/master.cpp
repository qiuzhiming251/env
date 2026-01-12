#include "master.h"
#include <log_custom.h>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <string>
#include <base/params_manager/params_manager.h>
#include <localmap_construction/traffic_light_map_topo.h>
#include <perception_and_ld_map_fusion/data_fusion/geometry_match_info.h>
#include <time/time.h>

#include "fmt/core.h"
#include "lib/sd_navigation/SDMapElementExtract.h"
#include "lib/common/function_timer.h"
#include "message/env_model/routing_map/routing_map.h"
#include "message/env_model/occ/occ.h"
#include "message/env_model/navigation/navigation.h"
#include "message/sensor/camera/bev_lane/bev_lane.h"
#include "cyber/event/trace.h"
#include "cyber/time/time.h"
#include "modules/perception/env/src/lib/base/localmap_conf.h"

#include "modules/simulator/deterministic_scheduler/deterministic_scheduler.h"

using byd::simulator::deterministic_scheduler::SchedulerUnitManager;
using byd::simulator::deterministic_scheduler::TaskType;
using namespace cem::fusion;
using namespace cem::fusion::extendLane;
namespace cem {
namespace fusion {

std::shared_ptr<cem::fusion::Master> MasterPtr = nullptr;

std::shared_ptr<MasterInterface> GetMasterInstance() {
  if (!MasterPtr) {
    MasterPtr = std::shared_ptr<Master>(new Master("modules/perception/env", "local_map"));
  }
  return MasterPtr;
}

Master::Master(const std::string &confdir, const std::string &name) {
  conf_abs_path_ = confdir + std::string("/");
  VLOG(1) << "conf_abs_path_ = " << conf_abs_path_;
  Init();
}

void Master::Init() {
  LoadConfig();
  InitMember();
}

void Master::LoadConfig() {
  ParamsManager::Instance()->ReadJsonParams(conf_abs_path_ + "conf/TrafficRoadFusion/E2_Conf_TrafficRoadFusion.json",
                                            conf_abs_path_ + "conf/TrafficRoadFusion/E2_Conf_MeasurementEvaluation.json");
}

void Master::InitMember() {
  landmark_buffer_.reset(new LandmarkBuffer());
  pre_processor_.reset(new PreProcessor(landmark_buffer_));
  messenger_.reset(new Messenger());
  stopline_mapping_.reset(new StoplineMapping);
  routing_map_stopline_processor_.reset(new RoutingMapStopLineProcessor);
  crosswalk_mapping_.reset(new CrossWalkTracker);
  traffic_light_mapping_.reset(new TrafficLightMapping);
  fusion_manager_.reset(new FusionManager());
  fusion_manager_->Init();
  lane_guidance_.reset(new LaneGuidance);
  env_info_processor_.reset(new navigation::EnvInfoProcessor());
  cross_road_construction_.reset(new CrossRoadConstruction);
  cross_road_construction_->Init();
  extend_lane_.reset(new ExtendLane);
  debug_infos_.reset(new DebugInfos);
  speed_limit_fusion_.reset(new SpeedLimitFusion());
  cnoa_speed_limit_.reset(new CnoaSpeedLimit());

  sd_navigation_base_    = std::make_shared<navigation::SdNavigationBase>();
  sd_navigation_highway_ = std::make_shared<navigation::SdNavigationHighway>();
  sd_navigation_city_    = std::make_shared<navigation::SdNavigationCity>();

  map_source_switch_ = std::make_unique<MapSourceSwitch>();
  map_source_switch_->Init(fusion_manager_.get());
}

// #define MapLessFlag
void Master::Proc() {
  FunctionTimer::measureTime("fusion_manager_", [&]() {fusion_manager_->Process();});

  fusion_manager_->SetCrossRoadInfo(cross_road_construction_->IsCrossRoad(), cross_road_construction_->CrossRoadDis(),
                                    cross_road_construction_->IsTurnRight());
  auto bev_map_aligned = fusion_manager_->GetBevMapInfo();
  auto ld_map_aligned = fusion_manager_->GetLdMapInfo();

  pre_processor_->SetTraverseCrossWalkLaneInfo(
      fusion_manager_->GetTraverseCrossWalkLaneInfo());
  FunctionTimer::measureTime("pre_processor_", [&]() {pre_processor_->Process(bev_map_aligned, ld_map_aligned);});

  RoutingMapPtr routing_map_raw{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(routing_map_raw);
  auto raw_bev_map_ptr = INTERNAL_PARAMS.raw_bev_data.GetRawBevMapPtr();
  if(raw_bev_map_ptr) {
     AINFO<<"sequence_num:"<<raw_bev_map_ptr->header.cycle_counter;
  }


  ///高速和城区的切换
  bool IS_ON_HIGHWAY_FLAG = false;
  {
    IS_ON_HIGHWAY_FLAG = GetIsOnHighwayFlag(routing_map_raw);
    if (IS_ON_HIGHWAY_FLAG) {
      sd_navigation_base_ = sd_navigation_highway_;
    } else {
      sd_navigation_base_ = sd_navigation_city_;
    }
  }
  DetectBevMap bev_map = pre_processor_->GetGlobalBevMapPtr();

  // stopline processor
  FunctionTimer::measureTime("stopline_mapping_", [&]() {stopline_mapping_->Process();});
  const auto &bev_stopline_objs = stopline_mapping_->GetStopLines();

  // crosswalk processor
  FunctionTimer::measureTime("crosswalk_mapping_", [&]() {crosswalk_mapping_->Process(bev_map);});

  ///导航推荐
  std::vector<std::pair<uint64_t, std::pair<int, int>>> guide_lane_result     = {};
  std::vector<uint64_t>                                 sd_guide_lane_ids_ref = {};
  cem::fusion::navigation::AccLaneInfo  sd_acc_lane_info;
  FunctionTimer::measureTime("sd_navigation_base_", [&]()
    {sd_navigation_base_->Proc(routing_map_raw, raw_bev_map_ptr, bev_map.bev_map_ptr, guide_lane_result, sd_guide_lane_ids_ref, sd_acc_lane_info);});
  SD_COARSE_MATCH_LOG << "[sd_navigation_base_->Proc] Normal lane id: "<< sd_acc_lane_info.ego_normal_lane_id;
  // guide_lane_result.clear();
  // guide_lane_result = fusion_manager_->GetRecommendPath();
  auto junctions_ptr = INTERNAL_PARAMS.navigation_info_data.GetJunctionInfoCityPtr();

  if ((!IS_ON_HIGHWAY_FLAG) && (routing_map_raw != nullptr)) {
    auto trajectory = sd_navigation_city_->GetNextRightPoints(routing_map_raw);
    pre_processor_->SetRightTurnTrajectory(routing_map_raw->header.timestamp, trajectory);
  }

  ///env_info
  env_info_processor_->SetIsOnHighway(IS_ON_HIGHWAY_FLAG);
  FunctionTimer::measureTime("env_info_processor_", [&]() {env_info_processor_->Process(bev_stopline_objs);});
  const EnvInfo *env_info = env_info_processor_->GetEnvInfo();
  if (bev_map.bev_map_ptr != nullptr) {
    bev_map.bev_map_ptr->env_info = *env_info;
  }

  auto routing_map = pre_processor_->routing_map_pre_processor_->GetRoutingMap();

  if (routing_map != nullptr) {
    routing_map->env_info = *env_info;
  }

  FunctionTimer::measureTime("extend_lane_", [&]() {extend_lane_->Process(bev_map);});
  bev_ld_map_geometry_match_info_.CalculateMapGeometryMatchInfo(fusion_manager_->GetBevMapInfo(), fusion_manager_->GetLdMapInfo());

  // cross road part
  cross_road_construction_->SetGeometryMatchInfo(&bev_ld_map_geometry_match_info_);
  cross_road_construction_->SetEnvStatus(env_status_);
  FunctionTimer::measureTime("cross_road_construction_", [&]()
    {cross_road_construction_->Process(bev_map.bev_map_ptr, bev_map.Twb, raw_bev_map_ptr, junctions_ptr, routing_map, IS_ON_HIGHWAY_FLAG);});

  // speed limit fusion
  FunctionTimer::measureTime("speed_limit_fusion_", [&]() {speed_limit_fusion_->Process(bev_map, routing_map);});
  if (!IS_ON_HIGHWAY_FLAG) {
    FunctionTimer::measureTime("cnoa_speed_limit_", [&]() {cnoa_speed_limit_->Process(bev_map, routing_map);});
  }

  // Post-processing of the bev map and intergration the guide lane, virtual
  // lane, traffic light status, etc.
  FunctionTimer::measureTime("lane_guidance_", [&]()
    {lane_guidance_->Process(routing_map_raw, junctions_ptr, bev_map, bev_stopline_objs, guide_lane_result);});

  traffic_light_mapping_->SetEgoLdLaneId(bev_ld_map_geometry_match_info_.GetLdBevLane().id);
  FunctionTimer::measureTime("traffic_light_mapping_", [&]()
    {traffic_light_mapping_->Process(routing_map_raw, routing_map, lane_guidance_->GetMapInfo(), sd_navigation_city_, raw_bev_map_ptr,
                                  lane_guidance_->GetTwb().inverse());});

  messenger_->PulishDiagno();
  messenger_->PulishTrafficLightE2E(traffic_light_mapping_->GetTrafficLightsE2E());
  if(routing_map) {
    BevMapInfoPtr sync_tarck{nullptr};
    SensorDataManager::Instance()->GetLatestSensorFrame(routing_map->header.timestamp, 0.05, sync_tarck);
    // 通过bev停止线调整地图的路口虚拟线起点
    if(sync_tarck) {
      FunctionTimer::measureTime("routing_map_stopline_processor_", [&]()
        {routing_map_stopline_processor_->Process(sync_tarck, bev_map.Twb, routing_map, bev_ld_map_geometry_match_info_);});
    }
  }

  GeometryMatchResult match_detail;
  bev_ld_map_geometry_match_info_.IsLaneBoundaryMatch(match_detail);
  map_source_switch_->SetRampFlag(env_info->is_switched_to_LD_);
  map_source_switch_->SetMultipleConsecutiveRoadIntersectionsFlag(
      sd_navigation_city_->IsCutMapActive());
  map_source_switch_->SetJunctionInfo(
      traffic_light_mapping_->GetPreviousJunctionEgoPassed(),
      traffic_light_mapping_->DistanceToJunction(),
      traffic_light_mapping_->DistanceToSectionOverFirstJunction(),
      traffic_light_mapping_->IsNextSectionHasJunction());
  map_source_switch_->SetEgoLaneMatchingInfo(match_detail);
  FunctionTimer::measureTime("map_source_switch_", [&]() {map_source_switch_->Process();});

  auto env_status = map_source_switch_->GetEnvStatus();
  env_status_ = map_source_switch_->GetEnvStatus();

#ifndef MapLessFlag
  auto used_ld_map = !map_source_switch_->IsUsePerceptionMap();
  map_source_switch_->CopySensorStatusInfoTo(routing_map);
#else
  auto used_ld_map = false;
#endif
  //TODO 实现一个lambda 函数判断
  bool current_bev_has_top{false};
  auto bevLdMatchers = fusion_manager_->GetMatchers();
  for (const  auto &mather : bevLdMatchers){
      if(!fusion_manager_)continue;
      current_bev_has_top = (!(fusion_manager_ ->GetBevLaneSplitInfo(mather.bev_id).empty()))
                          || (!(fusion_manager_ ->GetBevLaneMergeInfo(mather.bev_id).empty()));
      if(current_bev_has_top){
        break;
      }
  }
  messenger_->SetGeometryMatchInfo(bevLdMatchers,current_bev_has_top);
  env_status->set_debug_info(fusion_manager_->DebugInfo());

  debug_infos_->SetDebugInfo(used_ld_map, lane_guidance_->GetMapInfo(), lane_guidance_->GetTwb(), routing_map);
  messenger_->PulishFusionMap(!used_ld_map, lane_guidance_->GetMapInfo(), routing_map, bev_stopline_objs, IS_ON_HIGHWAY_FLAG,
                              sd_acc_lane_info, sd_navigation_city_->GetInfo());

  if (auto map_output = messenger_->GetFusionMapInfo(); map_output) {
    map_output->mutable_traffic_light_left()->CopyFrom(traffic_light_mapping_->GetLeftStatus());
    map_output->mutable_traffic_light_right()->CopyFrom(traffic_light_mapping_->GetRightStatus());
    map_output->mutable_traffic_light_uturn()->CopyFrom(traffic_light_mapping_->GetUturnStatus());
    map_output->mutable_traffic_light_straight()->CopyFrom(traffic_light_mapping_->GetStraightStatus());
    double   adc_to_junction = traffic_light_mapping_->DistanceToJunction();
    fusion_manager_->SetAdc2Junction(adc_to_junction);
    uint64_t ld_counter      = routing_map == nullptr ? 0 : routing_map->header.cycle_counter;
    uint64_t bev_counter     = lane_guidance_->GetMapInfo() == nullptr ? 0 : lane_guidance_->GetMapInfo()->header.cycle_counter;
    map_output->mutable_map_info()->mutable_env_status()->CopyFrom(*env_status);
    if (IS_ON_HIGHWAY_FLAG) {
      map_output->mutable_map_info()->set_map_version(
          fmt::format("ld_counter:{} bev_counter:{} vision_counter:{} "
                      "adc_to_junction:{:.2f} IS_ON_HIGHWAY_FLAG:{:d}  "
                      "used_ld_map:{:d} is_switched:{:d} switch_reason:{}",
                      ld_counter, bev_counter, traffic_light_mapping_->GetPerceptionCyberCounter(),
                      adc_to_junction, IS_ON_HIGHWAY_FLAG,
                      used_ld_map, env_info->is_switched_to_LD_, env_info->switch_ld_reason) +
          sd_navigation_highway_->GethighInfo());
    } else {
      map_output->mutable_map_info()->set_map_version(
          fmt::format("ld_counter:{} bev_counter:{} vision_counter:{} "
                      "adc_to_junction:{:.2f} IS_ON_HIGHWAY_FLAG:{:d}  "
                      "used_ld_map:{:d} is_switched:{:d} switch_reason:{}",
                      ld_counter, bev_counter, traffic_light_mapping_->GetPerceptionCyberCounter(),
                      adc_to_junction, IS_ON_HIGHWAY_FLAG,
                      used_ld_map, env_info->is_switched_to_LD_, env_info->switch_ld_reason) +
          sd_navigation_city_->GetInfo());
    }
    map_output->mutable_map_info()->set_engine_version(
        bev_ld_map_geometry_match_info_.GetDebugInfo() +
        fmt::format("is_right:{:d} ", sd_navigation_city_->IsEgoInDedicatedRightLane()) +
        traffic_light_mapping_->TrafficLightLaneDebug());
    GEOMETRY_LOG << map_output->map_info().map_version();
    TRAFFIC_LOG << map_output->map_info().engine_version();
  }
}

bool Master::GetIsOnHighwayFlag(const RoutingMapPtr &routing_map_raw) {
  auto is_on_highway_flag = [](const SDSectionInfo &sd_section_info) {
    auto    &current_road_class   = sd_section_info.road_class;
    uint32_t current_section_type = sd_section_info.link_type;
    uint32_t section_type_extend = sd_section_info.link_type_extend;
    uint64_t sd_section_id = sd_section_info.id / 10;
    if ((sd_section_id == 1678534592) || (sd_section_id == 1699361336) ||
        (sd_section_id == 1699374097) || (sd_section_id == 1691246158) ||
        (sd_section_id == 1607693456) || (sd_section_id == 1607693601) ||
        (sd_section_id == 1648672436) || (sd_section_id == 1648672824) ||
        (sd_section_id == 1678890528) || (sd_section_id == 1705771743) ||
        (sd_section_id == 1705771555) || (sd_section_id == 1699402002) ||
        (sd_section_id == 1699402004) || (sd_section_id == 1613352349) ||
        (sd_section_id == 1678533134) || (sd_section_id == 1699372997) ||
        (sd_section_id == 1699374094) || (sd_section_id == 1607690036) ||
        (sd_section_id == 1607690332) || (sd_section_id == 1664789284) ||
        (sd_section_id == 1664789271) || (sd_section_id == 1648672814) ||
        (sd_section_id == 1678890452) || (sd_section_id == 1699443498) ||
        (sd_section_id == 1705771744) || (sd_section_id == 1705771880) ||
        (sd_section_id == 1705771278) || (sd_section_id == 1705771745) ||
        (sd_section_id == 1699402006) || (sd_section_id == 1699402007) ||
        (sd_section_id == 1705771745) || (sd_section_id == 1663998523) ||
        (sd_section_id == 1582251492)) {
      return false;
    }

    if (current_road_class == SDRoadClass::SD_HIGHWAY ||
        current_road_class == SDRoadClass::SD_CITY_FAST_WAY ||
        (current_section_type & (uint32_t)SDLinkTypeMask::SDLT_IC) ==
            (uint32_t)SDLinkTypeMask::SDLT_IC ||
        (current_section_type & (uint32_t)SDLinkTypeMask::SDLT_JCT) ==
            (uint32_t)SDLinkTypeMask::SDLT_JCT ||
        (current_section_type & (uint32_t)SDLinkTypeMask::SDLT_SERVICE) ==
            (uint32_t)SDLinkTypeMask::SDLT_SERVICE ||
        (section_type_extend &
         (uint32_t)SDLinkTypeExtendMask::SDLTE_GAS_STATION_ENUUTRANCE) ==
            (uint32_t)SDLinkTypeExtendMask::SDLTE_GAS_STATION_ENUUTRANCE) {
      return true;
    }
    return false;
  };

  auto is_highway_ramp = [](const SDSectionInfo &sd_section_info) {
    uint32_t section_type = sd_section_info.link_type;
    uint32_t section_type_extend = sd_section_info.link_type_extend;
    if ((section_type & (uint32_t)SDLinkTypeMask::SDLT_JCT) ==
            (uint32_t)SDLinkTypeMask::SDLT_JCT ||
        (section_type & (uint32_t)SDLinkTypeMask::SDLT_IC) ==
            (uint32_t)SDLinkTypeMask::SDLT_IC ||
        (section_type & (uint32_t)SDLinkTypeMask::SDLT_SERVICE) ==
            (uint32_t)SDLinkTypeMask::SDLT_SERVICE ||
        (section_type & (uint32_t)SDLinkTypeMask::SDLT_PARK) ==
            (uint32_t)SDLinkTypeMask::SDLT_PARK ||
        (section_type & (uint32_t)SDLinkTypeMask::SDLT_RAMP) ==
            (uint32_t)SDLinkTypeMask::SDLT_RAMP ||
        (sd_section_info.road_class == SDRoadClass::SD_OTHER_ROAD &&
         (section_type & (uint32_t)SDLinkTypeMask::SDLT_POICONNECTION) ==
             (uint32_t)SDLinkTypeMask::SDLT_POICONNECTION) ||
        (section_type_extend &
         (uint32_t)SDLinkTypeExtendMask::SDLTE_GAS_STATION_ENUUTRANCE) ==
            (uint32_t)SDLinkTypeExtendMask::SDLTE_GAS_STATION_ENUUTRANCE) {
      return true;
    } else {
      return false;
    }
  };

  auto is_highway_city_connection =
      [is_on_highway_flag, is_highway_ramp](
          const SDSectionInfo &pre_sd_section_info,
          const SDSectionInfo &cur_sd_section_info,
          const SDSectionInfo &suc_sd_section_info) {
        uint32_t cur_section_type = cur_sd_section_info.link_type;
        if ((pre_sd_section_info.successor_section_id_list.size() == 1) &&
            (is_on_highway_flag(pre_sd_section_info)) &&
            ((cur_section_type & (uint32_t)SDLinkTypeMask::SDLT_IC) ==
             (uint32_t)SDLinkTypeMask::SDLT_IC) &&
            (!is_on_highway_flag(suc_sd_section_info) &&
             (!is_highway_ramp(suc_sd_section_info)))) {
          return true;
        } else {
          return false;
        }
      };

  if (routing_map_raw == nullptr) {
    return false;
  }

  auto    &sd_mpp_sections   = routing_map_raw->sd_route.mpp_sections;
  uint64_t sd_ego_sectionid  = routing_map_raw->sd_route.navi_start.section_id;
  int      ego_section_index = -1;
  for (unsigned int i = 0; i < sd_mpp_sections.size(); i++) {
    if (sd_ego_sectionid == sd_mpp_sections[i].id) {
      ego_section_index = i;
      break;
    }
  }
  if (ego_section_index == -1) {
    return routing_map_raw->is_on_highway;
  }
  auto &current_section = sd_mpp_sections[ego_section_index];

  bool IS_ON_HIGHWAY_FLAG = routing_map_raw->is_on_highway;
  if (IS_ON_HIGHWAY_FLAG) {
    if (!is_on_highway_flag(current_section)) {
      return false;
    }
    double ego_s_offset = routing_map_raw->sd_route.navi_start.s_offset;
    double ego_s_global = ego_s_offset;
    if (ego_section_index >= 1 &&
        ego_section_index < sd_mpp_sections.size() - 1) {
      if (is_highway_city_connection(sd_mpp_sections[ego_section_index - 1],
                                     current_section,
                                     sd_mpp_sections[ego_section_index + 1])) {
        for (int idx = ego_section_index - 1; idx >= 0; --idx) {
          const auto &prev_sec = sd_mpp_sections[idx];
          if ((prev_sec.link_type & (uint32_t)SDLinkTypeMask::SDLT_IC) !=
              (uint32_t)SDLinkTypeMask::SDLT_IC) {
            break;
          }
          // AINFO << "prev_sec.length: " << prev_sec.length;
          ego_s_global += prev_sec.length;
          // AINFO << "ego distance to IC: " << ego_s_global;
          if (std::fabs(ego_s_global) > 100.0) {
            return false;
          }
        }
      }
    }

    if (is_highway_ramp(current_section)) {
      return true;
    }
    double length_tmp = fabs(current_section.length - routing_map_raw->sd_route.navi_start.s_offset);
    double max_range  = 1500;
    for (unsigned int i = ego_section_index + 1; i < sd_mpp_sections.size(); i++) {
      if (max_range < length_tmp) {
        break;
      }

      if (i >= 1 && i < sd_mpp_sections.size() - 1) {
        if (is_highway_city_connection(sd_mpp_sections[i - 1],
                                       sd_mpp_sections[i],
                                       sd_mpp_sections[i + 1])) {
          IS_ON_HIGHWAY_FLAG = false;
          break;
        }
      }

      if (is_on_highway_flag(sd_mpp_sections[i]) && is_highway_ramp(sd_mpp_sections[i])) {
        break;
      }
      if (!is_on_highway_flag(sd_mpp_sections[i])) {
        IS_ON_HIGHWAY_FLAG = false;
        break;
      }
      length_tmp += sd_mpp_sections[i].length;
    }
    return IS_ON_HIGHWAY_FLAG;

  } else {
    return false;
  }
}

/**********************************发送消息的数据结构接口************************************/

std::shared_ptr<byd::msg::basic::ModuleStatus> Master::GetModuleStatusPtr() {
  return this->messenger_->GetDiagnoInfo();
}

std::shared_ptr<byd::msg::orin::routing_map::RoutingMap> Master::GetENVRoutingMapPtr() {
  return this->messenger_->GetRoutingMapInfo();
}

std::shared_ptr<byd::msg::orin::routing_map::TrafficLightsE2EInfo> Master::GetTrafficLightsE2EPtr() {
  return this->messenger_->GetTrafficLightsE2EInfo();
}

/***************************接收消息callback函数在此处实现************************************/

void Master::OnVehInfoCallback(const std::shared_ptr<byd::msg::drivers::VehInfo> &msg) {
  this->messenger_->Callback<byd::msg::drivers::VehInfo, cem::message::sensor::VehicleSignal>(msg);
  this->messenger_->Callback<byd::msg::drivers::VehInfo, cem::message::sensor::Bcm50msPdu02>(msg);
  this->messenger_->Callback<byd::msg::drivers::VehInfo, cem::message::sensor::Ibs20msPdu08>(msg);
  this->messenger_->Callback<byd::msg::drivers::VehInfo, cem::message::sensor::Imcu20msPdu05>(msg);
  this->messenger_->Callback<byd::msg::drivers::VehInfo, cem::message::sensor::Eps010msPdu00>(msg);
  this->messenger_->Callback<byd::msg::drivers::VehInfo, cem::message::sensor::Ibs20msPdu15>(msg);
  this->messenger_->Callback<byd::msg::drivers::VehInfo, cem::message::sensor::Sas10msPdu00>(msg);
  this->messenger_->Callback<byd::msg::drivers::VehInfo, cem::message::sensor::Mpd100msPdu04>(msg);
  this->messenger_->Callback<byd::msg::drivers::VehInfo, cem::message::sensor::Icm100msPdu29>(msg);
  this->messenger_->Callback<byd::msg::drivers::VehInfo, cem::message::sensor::AdasIpd100msPdu12>(msg);
}

void Master::OnEnvModelLocalMapInfoCallback(const std::shared_ptr<byd::msg::env_model::LocalMapInfo> &msg) {
  this->messenger_->Callback<byd::msg::env_model::LocalMapInfo, cem::message::sensor::BevMapInfo>(msg);
}

void Master::OnRoutingMapCallBack(const std::shared_ptr<byd::msg::orin::routing_map::RoutingMap> &msg) {
  this->messenger_->Callback<byd::msg::orin::routing_map::RoutingMap, cem::message::env_model::RoutingMap>(msg);
}

void Master::OnMsgMapEventCallback(const std::shared_ptr<byd::msg::orin::routing_map::MapEvent> &msg) {
  this->messenger_->Callback<byd::msg::orin::routing_map::MapEvent, cem::message::env_model::MapEvent>(msg);
} /* OnMsgMapEventCallback */

void Master::OnPercepTrfInfoCallback(const std::shared_ptr<byd::msg::perception::PerceptionTrafficInfo> &msg) {
  this->messenger_->Callback<byd::msg::perception::PerceptionTrafficInfo, cem::message::sensor::PercepTrfInfo>(msg);
}

void Master::OnVisionTrfInfoCallback(const std::shared_ptr<byd::msg::perception::PerceptionTrafficInfo> &msg) {
  this->messenger_->Callback<byd::msg::perception::PerceptionTrafficInfo, cem::message::sensor::VisionTrfInfo>(msg);
}

void Master::OnPerceptionObstaclesInfoCallback(const std::shared_ptr<byd::msg::perception::PerceptionObstacles> &msg) {
  this->messenger_->Callback<byd::msg::perception::PerceptionObstacles, cem::message::sensor::FusObjInfo>(msg);
}

void Master::OnMsgDriverInsCallback(const std::shared_ptr<byd::msg::drivers::Ins> &msg) {
  this->messenger_->Callback<byd::msg::drivers::Ins, cem::message::sensor::LocalizationData>(msg);
}

void Master::OnMsgLocationDrCallback(const std::shared_ptr<byd::msg::localization::LocalizationEstimate> &msg) {
  this->messenger_->Callback<byd::msg::localization::LocalizationEstimate, cem::message::sensor::LocalizationData>(msg);
}

void Master::OnEnvModelLidaRoadedgeCallback(const std::shared_ptr<byd::msg::env_model::LocalMapInfo> &msg) {
  this->messenger_->Callback<byd::msg::env_model::LocalMapInfo, cem::message::sensor::LidarRoadEdgeInfo>(msg);
}

void Master::OnPlanResultCallback(const std::shared_ptr<byd::msg::planning::PLanningResultProto> &msg) {
  this->messenger_->Callback<byd::msg::planning::PLanningResultProto, cem::message::env_model::PLanningResultData>(msg);
}

void Master::OnPlanFuncStateCallback(const std::shared_ptr<byd::msg::pnc::PlanFuncState> &msg) {
  this->messenger_->Callback<byd::msg::pnc::PlanFuncState, cem::message::env_model::PLanFuncState>(msg);
}
void Master::OncanoutCallback(const std::shared_ptr<byd::msg::orin::sm_msgs::MsgSM_BYD_CAN_Output> &msg) {
  this->messenger_->Callback<byd::msg::orin::sm_msgs::MsgSM_BYD_CAN_Output, cem::message::env_model::CAN1>(msg);
}
void Master::OnOccCallback(const std::shared_ptr<byd::msg::perception::OCCInfo>& msg){
  this->messenger_->Callback<byd::msg::perception::OCCInfo,cem::env_model::occ::OCCInfo>(msg);
  return;

}
void Master::OnNaviTrafCallback(const std::shared_ptr<byd::msg::drivers::TrafficInfoNotify> &msg) {
  this->messenger_->Callback<byd::msg::drivers::TrafficInfoNotify, cem::message::env_model::NaviTrafficInfo>(msg);
}
void Master::OnDriversEventCallback(const std::shared_ptr<byd::msg::drivers::Event> &msg) {
  this->messenger_->Callback<byd::msg::drivers::Event, cem::message::env_model::DriversEvent>(msg);
}

void Master::OnSDTrafficLightCallback(const std::shared_ptr<byd::msg::drivers::sdTrafficLight> &msg) {
#if defined(PC_X86_RUN_FLAG) && (PC_X86_RUN_FLAG)
  if (msg && msg->has_header() && msg->header().has_publish_timestamp()) {
    msg->mutable_header()->set_publish_timestamp(GetMwTimeNowSec());
  }
#endif
  SensorDataManager::Instance()->AddSensorMeasurements(msg);
}

void Master::OnMapLocationResultCallback(const std::shared_ptr<byd::modules::localization::MapMatchResult> &msg) {
#if defined(PC_X86_RUN_FLAG) && (PC_X86_RUN_FLAG)
  if (msg && msg->has_header() && msg->header().has_publish_timestamp()) {
    msg->mutable_header()->set_publish_timestamp(GetMwTimeNowSec());
  }
#endif
  SensorDataManager::Instance()->AddSensorMeasurements(msg);
}

void Master::OnMapLocationResultBaiduCallback(
    const std::shared_ptr<byd::modules::localization::MapMatchResult> &msg) {
  this->messenger_->Callback<byd::modules::localization::MapMatchResult,
                             cem::message::sensor::MapMatchResultBaidu>(msg);
}

void Master::OnMsgObstaclesCallback(const std::shared_ptr<byd::msg::pnc::ObjInfoFusn> &msg) {
  ;
}
/***************************************************************************************************/

}  // namespace fusion
}  // namespace cem
