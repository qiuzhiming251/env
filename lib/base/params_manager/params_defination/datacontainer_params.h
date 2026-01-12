#ifndef DATA_CONTAINER_H_
#define DATA_CONTAINER_H_

#include <unordered_map>
#include <utility>

#include "common/enum.h"
#include "message/env_model/navigation/navigation.h"
#include "message/internal_message.h"
using cem::fusion::navigation::RecommendLane;
using cem::fusion::navigation::VirtualSection;

namespace cem {
namespace fusion {

typedef const BevLaneMarker *const                                 ConstBevLaneMarker;
typedef const BevLaneInfo *const                                   ConstBevLaneInfo;
typedef const SDSectionInfo *const                                 ConstSDSectionInfo;
typedef const SectionInfo *const                                   ConstSectionInfo;
typedef const SDLaneGroupInfo *const                               ConstSDLaneGroupInfo;
typedef const LaneInfo *const                                      ConstLaneInfo;
typedef std::shared_ptr<std::vector<navigation::JunctionInfo>>     SdJunctionInfoPtr;
typedef std::shared_ptr<std::vector<navigation::JunctionInfoCity>> SdJunctionInfoCityPtr;

using LDRoutingMap  = cem::message::env_model::RoutingMap;
using LDRouteInfo   = cem::message::env_model::RouteInfo;
using LDSectionInfo = cem::message::env_model::SectionInfo;
using LDLaneInfo    = cem::message::env_model::LaneInfo;
using MergeTopoInfo    = cem::fusion::navigation::MergeTopoInfo;

using LDRoutingMapPtr      = std::shared_ptr<LDRoutingMap>;
using LDRoutingMapConstPtr = std::shared_ptr<const LDRoutingMap>;
using LDRouteInfoPtr       = std::shared_ptr<LDRouteInfo>;
using LDRouteInfoConstPtr  = std::shared_ptr<const LDRouteInfo>;

struct NaviDebugInfos {
  bool                                               is_on_highway_flag = false;
  uint64_t                                           bev_counter;
  std::vector<uint64_t>                              guide_laneids         = {};
  std::vector<uint64_t>                              guide_laneids_refer   = {};
  std::vector<std::vector<int>>                      guide_lanes_length    = {};
  std::vector<uint64_t>                              target_junction_ids   = {};
  std::vector<std::string>                           junction_type         = {};
  std::vector<std::string>                           split_merge           = {};
  std::vector<int>                                   junction_offset       = {};
  std::vector<std::string>                           match_type            = {};
  std::vector<std::string>                           junction_action       = {};
  std::vector<std::string>                           lane_type             = {};
  std::vector<std::vector<uint64_t>>                 all_sections_in       = {};
  std::vector<uint64_t>                              target_section        = {};
  std::vector<uint64_t>                              hold_target_section   = {};
  bool                                               enter_hold_lanes_flag = false;
  std::vector<uint64_t>                              hold_guide_lanes      = {};
  std::vector<std::vector<int>>                      hold_guides_length    = {};
  std::vector<uint64_t>                              target_section_filter = {};
  std::vector<uint64_t>                              select_road_junction  = {};
  std::vector<uint64_t>                              sections_without_bus  = {};
  std::vector<int>                                   sd_arrows             = {};
  std::vector<int>                                   target_lanes_index    = {};
  std::vector<std::vector<std::pair<uint64_t, int>>> all_separators        = {};
  std::vector<VirtualSection>                        virtual_sections;
  int                                                selected_vs_idx               = -1;
  std::vector<uint64_t>                              guide_lanes_topo              = {};
  std::vector<uint64_t>                              guide_lanes_mapless           = {};
  std::map<double, RecommendLane>                    sd_recommend_lane             = {};
  int                                                ReachableLaneCountToRamp      = 0;
  int                                                ExclusiveLaneCountToRamp      = 0;
  int                                                mapLaneNum                    = 0;
  int                                                bevLaneNum                    = 0;
  bool                                               isSplitRoadSelectWorked       = 0;
  std::vector<uint64_t>                              lane_backward                 = {};
  std::vector<int>                                   main_road_navigation_lane_num = {};
  std::string                                        lanegroup_failed              = "";
  uint64_t                                           bev_emergency_id;
};

class BevDataContainer {
 public:
  void ClearContainer() {
    bevMapPtrRaw = nullptr;
    laneboundary_list_.clear();
    road_edge_list_.clear();
    laneinfo_list_.clear();
  }

  ConstBevLaneMarker GetLaneBoundaryById(uint32_t id) const {
    if (laneboundary_list_.find(id) != laneboundary_list_.end()) {
      return laneboundary_list_.at(id);
    } else {
      return nullptr;
    }
  }

  ConstBevLaneInfo GetLaneInfoById(uint32_t id) const {
    if (laneinfo_list_.find(id) != laneinfo_list_.end()) {
      return laneinfo_list_.at(id);
    } else {
      return nullptr;
    }
  }

  ConstBevLaneMarker GetRoadEdgeById(uint32_t id) const {
    if (road_edge_list_.find(id) != road_edge_list_.end()) {
      return road_edge_list_.at(id);
    } else {
      return nullptr;
    }
  }

  BevMapInfoConstPtr GetRawBevMapPtr() const { return bevMapPtrRaw; }

  std::unordered_map<uint32_t, BevLaneMarker *> GetRoadEdgeList() { return road_edge_list_; }

  void SetBevMapPtr(BevMapInfoPtr bevMapPtr) { bevMapPtrRaw = bevMapPtr; }

  void AddLaneInfo(uint32_t id, BevLaneInfo *bev_lane_info) { laneinfo_list_.insert({id, bev_lane_info}); }

  void AddLaneBoundary(uint32_t id, BevLaneMarker *bev_lane_marker) { laneboundary_list_.insert({id, bev_lane_marker}); }

  void AddRoadEdge(uint32_t id, BevLaneMarker *bev_lane_marker) { road_edge_list_.insert({id, bev_lane_marker}); }

 private:
  BevMapInfoPtr                                 bevMapPtrRaw = nullptr;
  std::unordered_map<uint32_t, BevLaneMarker *> laneboundary_list_;
  std::unordered_map<uint32_t, BevLaneInfo *>   laneinfo_list_;
  std::unordered_map<uint32_t, BevLaneMarker *> road_edge_list_;
};

class SDMapDataContainer {

 public:
  void ClearContainer() {
    sd_section_info_list_.clear();
    sd_lane_groups_list_.clear();
    sd_lanes_list_.clear();
    lane_to_section_group_cache_.clear();
    routingMapPtr.reset();
    owner_.reset();
  }

  void SetOwnerRoutingMap(std::shared_ptr<RoutingMap> p) { owner_ = std::move(p); }

  SDRouteInfoConstPtr GetSDRouteInfoPtr() const { return routingMapPtr; }

  // void SetSDRouteInfoPtr(SDRouteInfoPtr routingMapPtr_in) { routingMapPtr = routingMapPtr_in; }
  void               SetSDRouteInfoPtr(SDRouteInfoPtr routingMapPtr_in) { routingMapPtr = std::move(routingMapPtr_in); }
  ConstSDSectionInfo GetSDSectionInfoById(uint64_t id) const {
    if (sd_section_info_list_.find(id) != sd_section_info_list_.end()) {
      return sd_section_info_list_.at(id);
    } else {
      return nullptr;
    }
  }

  void AddSDSectionInfoById(uint64_t id, SDSectionInfo *sd_section_info) { sd_section_info_list_.insert({id, sd_section_info}); }

  ConstSDLaneGroupInfo GetSDLaneGroupInfoById(uint64_t id) const {
    if (sd_lane_groups_list_.find(id) != sd_lane_groups_list_.end()) {
      return sd_lane_groups_list_.at(id);
    } else {
      return nullptr;
    }
  }

  ConstLaneInfo GetSDLaneInfoById(uint64_t id) const {
    if (sd_lanes_list_.find(id) != sd_lanes_list_.end()) {
      return sd_lanes_list_.at(id);
    } else {
      return nullptr;
    }
  }

  void AddSDLaneInfoById(uint64_t id, LaneInfo *sd_lane_info) { sd_lanes_list_.insert({id, sd_lane_info}); }

  void AddSDLaneGroupInfoById(uint64_t id, SDLaneGroupInfo *sd_lane_group_info) { sd_lane_groups_list_.insert({id, sd_lane_group_info}); }

  const std::unordered_map<uint64_t, SDLaneGroupInfo *> &GetSDLaneGroups() const { return sd_lane_groups_list_; }

  void BuildLaneCache() {
    lane_to_section_group_cache_.clear();
    for (const auto &pair : sd_lane_groups_list_) {
      const auto &lane_group   = pair.second;
      uint64_t    lanegroup_id = lane_group->id;
      for (const auto &section_pair : sd_section_info_list_) {
        const auto &section = section_pair.second;
        for (const auto &lg_idx : section->lane_group_idx) {
          if (lg_idx.id == lanegroup_id) {
            for (const auto &lane : lane_group->lane_info) {
              lane_to_section_group_cache_[lane.id] = {section->id, lanegroup_id};
            }
            break;
          }
        }
      }
    }
  }

  std::pair<uint64_t, uint64_t> GetLaneSectionAndGroup(uint64_t lane_id) const {
    auto it = lane_to_section_group_cache_.find(lane_id);
    if (it != lane_to_section_group_cache_.end()) {
      return it->second;
    }
    return {0, 0};
  }

 private:
  SDRouteInfoPtr                                  routingMapPtr = nullptr;
  std::shared_ptr<RoutingMap>                     owner_{nullptr};  //隐式地兜底生命周期,防止上游地图数据异常
  std::unordered_map<uint64_t, SDSectionInfo *>   sd_section_info_list_;
  std::unordered_map<uint64_t, SDLaneGroupInfo *> sd_lane_groups_list_;
  std::unordered_map<uint64_t, LaneInfo *>        sd_lanes_list_;
  // 映射 lane_id 到 (section_id, lanegroup_id)
  std::unordered_map<uint64_t, std::pair<uint64_t, uint64_t>> lane_to_section_group_cache_;
};

class LDMapDataContainer {
 public:
  void ClearContainer() {
    routingMapPtr_.reset();
    routeInfoPtr_.reset();
    ld_section_info_list_.clear();
    ld_lanes_list_.clear();
  }

  LDRouteInfoConstPtr GetLDRouteInfoPtr() const { return routeInfoPtr_; }
  void                SetLDRouteInfoPtr(LDRouteInfoPtr route_in) { routeInfoPtr_ = std::move(route_in); }

  LDRoutingMapConstPtr GetLDRoutingMapPtr() const { return routingMapPtr_; }
  void                 SetLDRoutingMapPtr(LDRoutingMapPtr map_in) { routingMapPtr_ = std::move(map_in); }

  ConstSectionInfo GetLDSectionInfoById(uint64_t id) const {
    auto it = ld_section_info_list_.find(id);
    return it != ld_section_info_list_.end() ? it->second : nullptr;
  }
  void AddLDSectionInfoById(uint64_t id, LDSectionInfo *sec) { ld_section_info_list_.insert({id, sec}); }

  ConstLaneInfo GetLDLaneInfoById(uint64_t id) const {
    auto it = ld_lanes_list_.find(id);
    return it != ld_lanes_list_.end() ? it->second : nullptr;
  }
  void AddLDLaneInfoById(uint64_t id, LDLaneInfo *lane) { ld_lanes_list_.insert({id, lane}); }

  void InitFromRoutingMap(const LDRoutingMapPtr &routing_map) {
    ClearContainer();
    if (!routing_map)
      return;

    routingMapPtr_ = routing_map;

    routeInfoPtr_ = std::make_shared<LDRouteInfo>(routing_map->route);

    for (auto &sec : routeInfoPtr_->sections) {
      sec.is_mpp_section = true;
      ld_section_info_list_.emplace(sec.id, &sec);
    }

    for (auto &sp : routeInfoPtr_->subpaths) {
      for (auto &sec : sp.sections) {
        sec.is_mpp_section = false;
        ld_section_info_list_.emplace(sec.id, &sec);
      }
    }

    for (auto &lane : routingMapPtr_->lanes) {
      ld_lanes_list_.emplace(lane.id, &lane);
    }
  }

  const std::unordered_map<uint64_t, LDSectionInfo *> &GetLDSections() const { return ld_section_info_list_; }
  const std::unordered_map<uint64_t, LDLaneInfo *>    &GetLDLanes() const { return ld_lanes_list_; }

 private:
  LDRoutingMapPtr routingMapPtr_{nullptr};
  LDRouteInfoPtr  routeInfoPtr_{nullptr};

  std::unordered_map<uint64_t, LDSectionInfo *> ld_section_info_list_;
  std::unordered_map<uint64_t, LDLaneInfo *>    ld_lanes_list_;
};

class NavigationInfoContainer {

 public:
  void ClearContainer() {
    junction_info_city_ptr_       = nullptr;
    junction_info_ptr_            = nullptr;
    target_crossroad_junction_id_ = 0;
    target_crossroad_junction_bev_lanes.clear();
    v2_junction_ids.clear();
    ResetNaviDebugInfos();
  }

  SdJunctionInfoCityPtr GetJunctionInfoCityPtr() const { return junction_info_city_ptr_; }

  void SetJunctionInfoCityPtr(SdJunctionInfoCityPtr sd_junction_info_city_ptr) { junction_info_city_ptr_ = sd_junction_info_city_ptr; }

  SdJunctionInfoPtr GetJunctionInfoPtr() const { return junction_info_ptr_; }

  void SetJunctionInfoPtr(SdJunctionInfoPtr sd_junction_info_ptr) { junction_info_ptr_ = sd_junction_info_ptr; }

  NaviDebugInfos &navi_debug_infos() { return navi_debug_infos_; }
  void            set_navi_debug_infos(const NaviDebugInfos &navi_debug_infos) { navi_debug_infos_ = navi_debug_infos; }

  void ResetNaviDebugInfos() {
    NaviDebugInfos navi_debug_infos{};
    navi_debug_infos_ = navi_debug_infos;
  }

  uint64_t target_crossroad_junction_id() const { return target_crossroad_junction_id_; }
  void     set_target_crossroad_junction_id(uint64_t target_crossroad_junction_id) {
        target_crossroad_junction_id_ = target_crossroad_junction_id;
  }
  const std::vector<uint64_t> &target_crossroad_junction_bev_lanes1() const { return target_crossroad_junction_bev_lanes; }
  void                         set_target_crossroad_junction_bev_lanes(const std::vector<uint64_t> &target_crossroad_junction_bev_lanes) {
                            this->target_crossroad_junction_bev_lanes = target_crossroad_junction_bev_lanes;
  }

  const std::vector<uint64_t> &get_v2_junction_ids() const { return v2_junction_ids; }

  void set_v2_junction_ids(const std::vector<uint64_t> &v2_junction_ids) { this->v2_junction_ids = v2_junction_ids; }

  const std::vector<uint64_t> &get_bev_root_section_ids() const { return bev_ego_root_section_x0_; }

  void set_bev_root_section_ids(const std::vector<uint64_t> &bev_ego_root_section_x0) {
    this->bev_ego_root_section_x0_ = bev_ego_root_section_x0;
  }

 private:
  SdJunctionInfoCityPtr junction_info_city_ptr_ = nullptr;  ///自车前方2km的地图导航信息 城区
  SdJunctionInfoPtr     junction_info_ptr_      = nullptr;  ///自车前方2km的高速导航信息

  uint64_t              target_crossroad_junction_id_ = 0;      ///当前关注的十字路口id
  std::vector<uint64_t> target_crossroad_junction_bev_lanes{};  ///当前关注的十字路口绑定的lane

  std::vector<uint64_t> v2_junction_ids{};  ///v2关注的路口
  std::vector<uint64_t> bev_ego_root_section_x0_;

  NaviDebugInfos navi_debug_infos_{};  ///导航调试debug信息
};
class BevLDMatchInfoContainer {
 public:
 void ClearContainer() {
      bev_map_matched_ids_.clear();
 }
  const std::map<uint64_t, std::vector<uint64_t>>& GetMatchInfo() const { return bev_map_matched_ids_; }
  void  SetMatchInfo(std::map<uint64_t, std::vector<uint64_t>>& match_result) { 
    bev_map_matched_ids_ = match_result; 
  }
 private:
   std::map<uint64_t, std::vector<uint64_t>> bev_map_matched_ids_;
};

class GeosMatchMergeInfoContainer {
 public:
   void ClearContainer() {
      geos_matched_merge_infos_.clear();
   }
   std::map<uint64_t, MergeTopoInfo>& GetMatchMergeInfo() { return geos_matched_merge_infos_; }
   void  SetMatchMergeInfo(uint64_t bev_id, MergeTopoInfo match_merge_topo) { 
         if(geos_matched_merge_infos_.find(bev_id) == geos_matched_merge_infos_.end()) {
            geos_matched_merge_infos_.insert({bev_id, match_merge_topo}); 
         }
   }
 private:
   std::map<uint64_t, MergeTopoInfo> geos_matched_merge_infos_;
};
}  // namespace fusion
}  // namespace cem

#endif
