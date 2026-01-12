#include "lib/perception_and_ld_map_fusion/data_preprocessor/lane_type_checker.h"
#define CHECK_TOPO_DEBUG_MODE 0

namespace cem {
namespace fusion {

LaneTypeChecker::LaneTypeChecker() {}
LaneTypeChecker::~LaneTypeChecker() {}

void LaneTypeChecker::KeeperGetFrameTopos()
{
  if(data_ == nullptr)
  {
    return;
  }

  frame_split_topos.clear();
  frame_merge_topos.clear();

  std::map<uint64_t, cem::message::sensor::BevLaneInfo*> lane_info;
  for(auto& bev_lane : data_->lane_infos)
  {
    SplitTopo split_topo;
    MergeTopo merge_topo;
    std::set <uint64_t> next_lane_ids;
    std::set <uint64_t> prev_lane_ids;

    if (!bev_lane.id || bev_lane.start_dx > max_keep_offset)
    {
      continue;
    }

    lane_info.insert({bev_lane.id, &bev_lane});
    for (auto &next_lane_id: bev_lane.next_lane_ids)
    {
      if (!next_lane_id) continue;
      auto next_lane_exist = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                          [&](const BevLaneInfo &p) { return p.id == next_lane_id;});

      if (next_lane_exist != data_->lane_infos.end()) {
        lane_info.insert({next_lane_id, &(*next_lane_exist)});
        next_lane_ids.insert(next_lane_id);
      }
    }

    for (auto& prev_lane_id: bev_lane.previous_lane_ids)
    {
      if(!prev_lane_id) continue;
      auto prev_lane_exist = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                          [&](const BevLaneInfo &p) { return p.id == prev_lane_id;});
      if(prev_lane_exist != data_->lane_infos.end())
      {
        lane_info.insert({prev_lane_id, &(*prev_lane_exist)});
        prev_lane_ids.insert(prev_lane_id);
      }
    }

    if(next_lane_ids.size() == 2)
    {
      uint64_t firstElement = *next_lane_ids.begin();
      uint64_t lastElement  = *next_lane_ids.rbegin();

      split_topo.main_lane  = bev_lane.id;
      split_topo.main_lane_type = bev_lane.split_topo_extend;
      if(lane_info.find(firstElement) != lane_info.end() && lane_info.find(lastElement) != lane_info.end())
      {
        auto sub_lane_1_id  = lane_info[firstElement]->id;
        auto sub_lane_2_id  = lane_info[lastElement]->id;

        auto sub_lane_1_type = lane_info[firstElement]->split_topo_extend;
        auto sub_lane_2_type = lane_info[lastElement]->split_topo_extend;

        if( sub_lane_1_type == SplitTopoExtendType::TOPOLOGY_SPLIT_NONE &&
            (sub_lane_2_type == SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT ||
             sub_lane_2_type == SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT))
        {
          split_topo.normal_lane  = sub_lane_1_id;
          split_topo.split_lane   = sub_lane_2_id;

          split_topo.normal_lane_type = sub_lane_1_type;
          split_topo.split_lane_type  = sub_lane_2_type;

          split_topo.split_lane_ids = {split_topo.main_lane, split_topo.normal_lane, split_topo.split_lane};
        }

        if( sub_lane_2_type == SplitTopoExtendType::TOPOLOGY_SPLIT_NONE &&
            (sub_lane_1_type == SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT ||
             sub_lane_1_type == SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT))
        {
          split_topo.normal_lane  = sub_lane_2_id;
          split_topo.split_lane   = sub_lane_1_id;

          split_topo.normal_lane_type = sub_lane_2_type;
          split_topo.split_lane_type  = sub_lane_1_type;

          split_topo.split_lane_ids = {split_topo.main_lane, split_topo.normal_lane, split_topo.split_lane};
        }
      }
      frame_split_topos.push_back(split_topo);
    }

    if(prev_lane_ids.size() == 2)
    {
      uint64_t firstElement = *prev_lane_ids.begin();
      uint64_t lastElement  = *prev_lane_ids.rbegin();

      merge_topo.main_lane = bev_lane.id;
      merge_topo.main_lane_type = bev_lane.merge_topo_extend;
      if(lane_info.find(firstElement) != lane_info.end() && lane_info.find(lastElement) != lane_info.end())
      {
        auto sub_lane_1_id  = lane_info[firstElement]->id;
        auto sub_lane_2_id  = lane_info[lastElement]->id;

        auto sub_lane_1_type = lane_info[firstElement]->merge_topo_extend;
        auto sub_lane_2_type = lane_info[lastElement]->merge_topo_extend;

        if( sub_lane_1_type  == MergeTopoExtendType::TOPOLOGY_MERGE_NONE &&
            (sub_lane_2_type == MergeTopoExtendType::TOPOLOGY_MERGE_LEFT ||
             sub_lane_2_type == MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT))
        {
          merge_topo.normal_lane  = sub_lane_1_id;
          merge_topo.merge_lane   = sub_lane_2_id;

          merge_topo.normal_lane_type  = sub_lane_1_type;
          merge_topo.merge_lane_type   = sub_lane_2_type;

          merge_topo.merge_lane_ids = {merge_topo.main_lane, merge_topo.normal_lane, merge_topo.merge_lane};
        }

        if( sub_lane_2_type  == MergeTopoExtendType::TOPOLOGY_MERGE_NONE &&
            (sub_lane_1_type == MergeTopoExtendType::TOPOLOGY_MERGE_LEFT ||
             sub_lane_1_type == MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT))
        {
          merge_topo.normal_lane  = sub_lane_2_id;
          merge_topo.merge_lane   = sub_lane_1_id;

          merge_topo.normal_lane_type  = sub_lane_2_type;
          merge_topo.merge_lane_type   = sub_lane_1_type;

          merge_topo.merge_lane_ids = {merge_topo.main_lane, merge_topo.normal_lane, merge_topo.merge_lane};
        }
      }
      frame_merge_topos.push_back(merge_topo);
    }
  }
}

void LaneTypeChecker::KeeperRemove()
{
  if(keeper_type == split_mode)
  {
    if(!split_keeper.empty())
    {
      split_keeper.erase(
          std::remove_if(split_keeper.begin(), split_keeper.end(), [&](const SplitTopo& p){
            return std::find_if(frame_split_topos.begin(), frame_split_topos.end(),
                                [&](const SplitTopo& split_topo){
                                  return split_topo.split_lane_ids == p.split_lane_ids;
                                }) == frame_split_topos.end();
          }), split_keeper.end());
    }
  }

  if(keeper_type == merge_mode)
  {
    if(!merge_keeper.empty())
    {
      merge_keeper.erase(
          std::remove_if(merge_keeper.begin(), merge_keeper.end(), [&](const MergeTopo& p){
            return std::find_if(frame_merge_topos.begin(), frame_merge_topos.end(),
                                [&](const MergeTopo& merger_topo){
                                  return merger_topo.merge_lane_ids == p.merge_lane_ids;
                                }) == frame_merge_topos.end();
          }), merge_keeper.end());
    }
  }
}

void LaneTypeChecker::KeeperCounter()
{
  if(keeper_type == split_mode)
  {
    for(const auto& split : frame_split_topos)
    {
      auto split_iter = std::find(split_keeper.begin(), split_keeper.end(), split);
      auto prev_split_iter = std::find(prev_frame_split_topos.begin(), prev_frame_split_topos.end(), split);
      if(split_iter != split_keeper.end())
      {
        if(prev_split_iter != prev_frame_split_topos.end())
        {
          if(split_iter->count < threshold)
          {
            split_iter->count++;
          }
        }
      }else{
        auto split_tp_exist =
            std::find_if(split_keeper.begin(), split_keeper.end(), [&](const SplitTopo& p){
              return p.split_lane_ids == split.split_lane_ids;});
        if(split_tp_exist == split_keeper.end())
        {
          split_keeper.push_back(split);
        }else{
          // TODO
          (*split_tp_exist)(split);
        }
      }
    }
    prev_frame_split_topos = frame_split_topos;
  }

  if(keeper_type == merge_mode)
  {
    for(const auto& merge : frame_merge_topos)
    {
      auto merge_iter = std::find(merge_keeper.begin(), merge_keeper.end(), merge);
      auto prev_merge_iter = std::find(prev_frame_merge_topos.begin(), prev_frame_merge_topos.end(), merge);

      if(merge_iter != merge_keeper.end())
      {
        if(prev_merge_iter != prev_frame_merge_topos.end())
        {
          if(merge_iter->count < threshold)
          {
            merge_iter->count++;
          }
        }
      }else{
        auto merge_tp_exist =
            std::find_if(merge_keeper.begin(), merge_keeper.end(), [&](const MergeTopo& p){
              return p.merge_lane_ids == merge.merge_lane_ids;});
        if(merge_tp_exist == merge_keeper.end())
        {
          merge_keeper.push_back(merge);
        }else{
          // TODO
          (*merge_tp_exist)(merge);
        }
      }
    }
    prev_frame_merge_topos = frame_merge_topos;
  }
}

bool LaneTypeChecker::GetKeeperStatus(const uint64_t main_lane_id,
                                      const uint64_t normal_lane_id,
                                      const uint64_t topo_lane_id,
                                      const int type)
{
  bool k_valid = false;
  std::set<uint64_t> topo_lane_ids = {main_lane_id, normal_lane_id, topo_lane_id};
  if(type == split_mode)
  {
    auto split_keeper_iter =
        std::find_if(split_keeper.begin(), split_keeper.end(), [&](const SplitTopo& p){
          return topo_lane_ids == p.split_lane_ids && p.count == 5;
        });

    if(split_keeper_iter != split_keeper.end())
    {
      auto main_lane_info = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                         [&](const BevLaneInfo &p) { return p.id == split_keeper_iter->main_lane;});

      auto normal_lane_info = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                           [&](const BevLaneInfo &p) { return p.id == split_keeper_iter->normal_lane;});

      auto split_lane_info = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                          [&](const BevLaneInfo &p) { return p.id == split_keeper_iter->split_lane;});

      if( main_lane_info != data_->lane_infos.end() &&
          normal_lane_info != data_->lane_infos.end() &&
          split_lane_info != data_->lane_infos.end())
      {
        main_lane_info->split_topo_extend    = split_keeper_iter->main_lane_type;
        normal_lane_info->split_topo_extend  = split_keeper_iter->normal_lane_type;
        split_lane_info->split_topo_extend   = split_keeper_iter->split_lane_type;
        k_valid = true;
      }
    }
  }

  if(type == merge_mode)
  {
    auto merge_keeper_iter =
        std::find_if(merge_keeper.begin(), merge_keeper.end(),[&](const MergeTopo& p){
          return topo_lane_ids == p.merge_lane_ids && p.count == 5;
        });

    if(merge_keeper_iter != merge_keeper.end())
    {
      auto main_lane_info = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                         [&](const BevLaneInfo &p) { return p.id == merge_keeper_iter->main_lane;});

      auto normal_lane_info = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                           [&](const BevLaneInfo &p) { return p.id == merge_keeper_iter->normal_lane;});

      auto merge_lane_info = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                          [&](const BevLaneInfo &p) { return p.id == merge_keeper_iter->merge_lane;});

      if( main_lane_info != data_->lane_infos.end() &&
          normal_lane_info != data_->lane_infos.end() &&
          merge_lane_info != data_->lane_infos.end())
      {
        main_lane_info->merge_topo_extend    = merge_keeper_iter->main_lane_type;
        normal_lane_info->merge_topo_extend  = merge_keeper_iter->normal_lane_type;
        merge_lane_info->merge_topo_extend   = merge_keeper_iter->merge_lane_type;
        k_valid = true;
      }
    }
  }
  return k_valid;
}

void LaneTypeChecker::UpdateKeeperStatus(const int type)
{
  keeper_type = type;

  KeeperGetFrameTopos();
  KeeperRemove();
  KeeperCounter();

  if(CHECK_TOPO_DEBUG_MODE)
  {
    KeeperDebugInfo();
  }
}

void LaneTypeChecker::KeeperDebugInfo()
{
  auto split_debug_info = [&](const SplitTopo& split){

    std::string ids = "";
    for(const auto& lane_id : split.split_lane_ids)
    {
      ids += std::to_string(lane_id) + ",";
    }

    AINFO << "[SPLIT] count:        " << split.count << " \n"
          << "[SPLIT] main_lane:    " << split.main_lane << " \n"
          << "[SPLIT] normal_lane:  " << split.normal_lane << " \n"
          << "[SPLIT] split_lane:   " << split.split_lane << " \n"
          << "[SPLIT] main_lane_type:    " << split_type[split.main_lane_type] << " \n"
          << "[SPLIT] normal_lane_type:  " << split_type[split.normal_lane_type] << " \n"
          << "[SPLIT] split_lane_type:   " << split_type[split.split_lane_type] << " \n"
          << "[SPLIT] split_lane_ids SIZE:" << split.split_lane_ids.size() << " \n"
          << "[SPLIT] ids:" << ids;
  };

  auto merge_debug_info = [&](const MergeTopo& merge){

    std::string ids = "";
    for(const auto& lane_id : merge.merge_lane_ids)
    {
      ids += std::to_string(lane_id) + ",";
    }

    AINFO << "[MERGE] count:        " << merge.count << " \n"
          << "[MERGE] main_lane:    " << merge.main_lane << " \n"
          << "[MERGE] normal_lane:  " << merge.normal_lane << " \n"
          << "[MERGE] merge_lane:   " << merge.merge_lane << " \n"
          << "[MERGE] main_lane_type:    " << merge_type[merge.main_lane_type] << " \n"
          << "[MERGE] normal_lane_type:  " << merge_type[merge.normal_lane_type] << " \n"
          << "[MERGE] merge_lane_type:   " << merge_type[merge.merge_lane_type] << " \n"
          << "[MERGE] merge_lane_ids SIZE:" << merge.merge_lane_ids.size() << " \n"
          << "[MERGE] ids: " << ids;
  };

  if(keeper_type == split_mode)
  {
    // 当前帧拓扑信息
    int frame_split_topo_cnt = 0;
    AINFO << "[frame_split_topo] SIZE: " << frame_split_topos.size();
    for(const auto& split_info : frame_split_topos)
    {
      frame_split_topo_cnt++;
      AINFO << "[frame_split_topo] " << frame_split_topo_cnt << " : ";
      split_debug_info(split_info);
    }

    // 前一帧拓扑信息
    int prev_frame_split_topo_cnt = 0;
    AINFO << "[prev_frame_split_topos] SIZE: " << prev_frame_split_topos.size();
    for(const auto& split_info : prev_frame_split_topos)
    {
      prev_frame_split_topo_cnt++;
      AINFO << "[prev_frame_split_topos] " << prev_frame_split_topo_cnt << " : ";
      split_debug_info(split_info);
    }

    // keeper拓扑信息
    int split_keeper_cnt = 0;
    AINFO << "[split_keeper] SIZE: " << split_keeper.size();
    for(const auto& split_info : split_keeper)
    {
      split_keeper_cnt++;
      AINFO << "[split_keeper] " << split_keeper_cnt << " : ";
      split_debug_info(split_info);
    }
  }

  if(keeper_type == merge_mode)
  {
    int frame_merge_topo_cnt = 0;
    AINFO << "[frame_merge_topos] SIZE: " << frame_merge_topos.size();
    for(const auto& merge_info : frame_merge_topos)
    {
      frame_merge_topo_cnt++;
      AINFO << "[frame_merge_topos] " << frame_merge_topo_cnt  << " : ";
      merge_debug_info(merge_info);
    }

    int prev_frame_merge_topo_cnt = 0;
    AINFO << "[prev_frame_merge_topos] SIZE: " << prev_frame_merge_topos.size();
    for(const auto& merge_info : prev_frame_merge_topos)
    {
      prev_frame_merge_topo_cnt++;
      AINFO << "[prev_frame_merge_topos] " << prev_frame_merge_topo_cnt << " : ";
      merge_debug_info(merge_info);
    }

    int merge_keeper_cnt = 0;
    AINFO << "[merge_keeper] SIZE: " << merge_keeper.size();
    for(const auto& merge_info : merge_keeper)
    {
      merge_keeper_cnt++;
      AINFO << "[merge_keeper] " << merge_keeper_cnt << " : ";
      merge_debug_info(merge_info);
    }
  }
}


void LaneTypeChecker::CheckSplitLaneType() {

  bool lane_type_keeper_strategy_enable = true;  // 车道属性锁定策略开关
  bool special_lane_strategy_enable     = true;  // 特殊车道校验策略开关
  bool bev_ld_matcher_strategy_enable   = true;  // 感知和地图匹配校验策略开关
  bool adjoin_lane_strategy_enable      = true;  // 左侧相邻车道校验策略开关
  bool bev_lane_slope_strategy_enable   = true;  // 拓扑车道斜率校验策略开关

  std::set<uint64_t> split_lane_ids;
  for (auto &lane : data_->lane_infos) {
    bool lane_type_keeper_valid_flag = false;  // 车道属性锁定校验有效标志
    bool special_lane_valid_flag     = false;  // 特殊车道校验有效标志
    bool bev_ld_matcher_vaild_flag   = false;  // 感知和地图匹配校验有效标志
    bool adjoin_lane_valid_flag      = false;  // 左侧相邻车道校验有效标志
    bool bev_lane_slope_valid_flag   = false;  // 拓扑车道斜率校验有效标志

    if (lane.next_lane_ids.size() != 2 || lane.is_build_split_marker_by_map) {
      continue;
    }

    auto main_lane_id   = lane.id;
    auto next_lane_1_id = lane.next_lane_ids[0];
    auto next_lane_2_id = lane.next_lane_ids[1];

    bool topo_is_valid = BevTopoIsValid(main_lane_id, next_lane_1_id, next_lane_2_id, split_mode);

    if (!topo_is_valid) {
      continue;
    }

    // 创建拓扑场景，用上游车道类型，此处不进行车道类型计算
    if (lane.is_build_split) {
      buildTopoTypeUpdate(main_lane_id, next_lane_1_id, next_lane_2_id, split_mode);
      continue;
    }

    // 如果已经在split_lane_ids，证明已经处理过这两个车道拓扑字段了，跳过
    if (split_lane_ids.count(lane.next_lane_ids[0]) && split_lane_ids.count(lane.next_lane_ids[1])) {
      continue;
    }

    // 规则1: 车道属性锁定校验
    if (lane_type_keeper_strategy_enable) {
      lane_type_keeper_valid_flag = GetKeeperStatus(main_lane_id, next_lane_1_id, next_lane_2_id, split_mode);
    }

    // 规则2: 特殊车道校验
    if (special_lane_strategy_enable && !lane_type_keeper_valid_flag) {
      special_lane_valid_flag = CheckTopoUseSpecialLane(next_lane_1_id, next_lane_2_id, split_mode);
    }

    // 规则3: 感知和地图匹配校验
    if (bev_ld_matcher_strategy_enable && !lane_type_keeper_valid_flag && !special_lane_valid_flag) {
      bev_ld_matcher_vaild_flag = CheckTopoUseMatchers(main_lane_id, split_mode);
    }

    // 规则4: 左侧相邻车道校验
    if (adjoin_lane_strategy_enable && !lane_type_keeper_valid_flag && !special_lane_valid_flag && !bev_ld_matcher_vaild_flag) {
      adjoin_lane_valid_flag = CheckTopoUseLeftAdjoinLane(main_lane_id, next_lane_1_id, next_lane_2_id, split_mode);
    }

    // 规则5: 拓扑车道斜率校验
    if (bev_lane_slope_strategy_enable && !lane_type_keeper_valid_flag && !special_lane_valid_flag && !bev_ld_matcher_vaild_flag &&
        !adjoin_lane_valid_flag) {
      bev_lane_slope_valid_flag = CheckTopoUseSlope(main_lane_id, next_lane_1_id, next_lane_2_id, split_mode);
    }

    if (CHECK_TOPO_DEBUG_MODE) {
      AINFO << "[SPLIT_LANE_TYPE]: ";
      AINFO << "lane_type_keeper_valid_flag: " << lane_type_keeper_valid_flag;
      AINFO << "special_lane_valid_flag: " << special_lane_valid_flag;
      AINFO << "bev_ld_matcher_vaild_flag: " << bev_ld_matcher_vaild_flag;
      AINFO << "adjoin_lane_valid_flag: " << adjoin_lane_valid_flag;
      AINFO << "bev_lane_slope_valid_flag: " << bev_lane_slope_valid_flag;
    }

    split_lane_ids.insert(next_lane_1_id);
    split_lane_ids.insert(next_lane_2_id);
  }

  if (CHECK_TOPO_DEBUG_MODE) {
    for (const auto &lane : data_->lane_infos) {
      AINFO << "DST id: " << lane.id << " split lane type: " << split_type[lane.split_topo_extend];
    }
  }

  // 更新keeper
  UpdateKeeperStatus(split_mode);
}

void LaneTypeChecker::CheckMergeLaneType() {

  bool lane_type_keeper_strategy_enable = true;  // 车道属性锁定策略开关
  bool special_lane_strategy_enable     = true;  // 特殊车道校验策略开关
  bool bev_ld_matcher_strategy_enable   = true;  // 感知和地图匹配校验策略开关
  bool adjoin_lane_strategy_enable      = true;  // 左侧相邻车道校验策略开关
  bool bev_lane_slope_strategy_enable   = true;  // 拓扑车道斜率校验策略开关

  std::set<uint64_t> merge_lane_ids;
  for (auto &lane : data_->lane_infos) {
    bool lane_type_keeper_valid_flag = false;  // 车道属性锁定校验有效标志
    bool special_lane_valid_flag     = false;  // 特殊车道校验有效标志
    bool bev_ld_matcher_vaild_flag   = false;  // 感知和地图匹配校验有效标志
    bool adjoin_lane_valid_flag      = false;  // 左侧相邻车道校验有效标志
    bool bev_lane_slope_valid_flag   = false;  // 拓扑车道斜率校验有效标志

    if (lane.previous_lane_ids.size() != 2 || lane.is_build_merge_marker_by_map) {
      continue;
    }

    auto main_lane_id   = lane.id;
    auto prev_lane_1_id = lane.previous_lane_ids[0];
    auto prev_lane_2_id = lane.previous_lane_ids[1];

    bool topo_is_valid = BevTopoIsValid(main_lane_id, prev_lane_1_id, prev_lane_2_id, merge_mode);

    if (!topo_is_valid) {
      continue;
    }

    // 创建拓扑场景，用上游车道类型，此处不进行车道类型计算
    if (lane.is_build_merge) {
      buildTopoTypeUpdate(main_lane_id, prev_lane_1_id, prev_lane_2_id, merge_mode);
      continue;
    }

    // 如果已经在merge_lane_ids，证明已经处理过这两个车道拓扑字段了，跳过
    if (merge_lane_ids.count(lane.previous_lane_ids[0]) && merge_lane_ids.count(lane.previous_lane_ids[1])) {
      continue;
    }

    // 规则1: 车道属性锁定校验
    if (lane_type_keeper_strategy_enable) {
      lane_type_keeper_valid_flag = GetKeeperStatus(main_lane_id, prev_lane_1_id, prev_lane_2_id, merge_mode);
    }

    // 规则2: 特殊车道校验
    if (special_lane_strategy_enable && !lane_type_keeper_valid_flag) {
      special_lane_valid_flag = CheckTopoUseSpecialLane(prev_lane_1_id, prev_lane_2_id, merge_mode);
    }

    // 规则3: 感知和地图匹配校验
    if (bev_ld_matcher_strategy_enable && !lane_type_keeper_valid_flag && !special_lane_valid_flag) {
      bev_ld_matcher_vaild_flag = CheckTopoUseMatchers(main_lane_id, merge_mode);
    }

    // 规则4: 左侧相邻车道校验
    if (adjoin_lane_strategy_enable && !lane_type_keeper_valid_flag && !special_lane_valid_flag && !bev_ld_matcher_vaild_flag) {
      adjoin_lane_valid_flag = CheckTopoUseLeftAdjoinLane(main_lane_id, prev_lane_1_id, prev_lane_2_id, merge_mode);
    }

    // 规则5: 拓扑车道斜率校验
    if (bev_lane_slope_strategy_enable && !lane_type_keeper_valid_flag && !special_lane_valid_flag && !bev_ld_matcher_vaild_flag &&
        !adjoin_lane_valid_flag) {
      bev_lane_slope_valid_flag = CheckTopoUseSlope(main_lane_id, prev_lane_1_id, prev_lane_2_id, merge_mode);
    }

    if (CHECK_TOPO_DEBUG_MODE) {
      AINFO << "[MERGE_LANE_TYPE]: ";
      AINFO << "lane_type_keeper_valid_flag: " << lane_type_keeper_valid_flag;
      AINFO << "special_lane_valid_flag: " << special_lane_valid_flag;
      AINFO << "bev_ld_matcher_vaild_flag: " << bev_ld_matcher_vaild_flag;
      AINFO << "adjoin_lane_valid_flag: " << adjoin_lane_valid_flag;
      AINFO << "bev_lane_slope_valid_flag: " << bev_lane_slope_valid_flag;
    }

    merge_lane_ids.insert(prev_lane_1_id);
    merge_lane_ids.insert(prev_lane_2_id);
  }

  if (CHECK_TOPO_DEBUG_MODE) {
    for (const auto &lane : data_->lane_infos) {
      AINFO << "DST id: " << lane.id << " merge lane type: " << merge_type[lane.merge_topo_extend]
            << " build_merge: " << lane.is_build_merge;
    }
  }

  // 更新keeper
  UpdateKeeperStatus(merge_mode);
}

bool LaneTypeChecker::HarborTopologyOperator() {
  bool harbor_valid = false;

  if (data_ == nullptr) {
    return false;
  }

  auto has_overlap = [&](std::pair<double, double> bev_range, std::vector<std::pair<double, double>> navi_ranges) -> bool {
    bool overlap_valid = false;
    for (const auto &pair : navi_ranges) {
      double max_start = std::max(pair.first, bev_range.first);
      double min_end   = std::min(pair.second, bev_range.second);
      if (max_start <= min_end) {
        overlap_valid = true;
        break;
      }
    }
    return overlap_valid;
  };

  auto get_lane_position = [&](uint64_t query_id, std::set<uint64_t> &topo_lane_ids) -> std::string {
    std::string position = "none";

    std::set<bool>               has_pos_left;
    std::vector<Eigen::Vector2f> query_pts;

    GetBevLaneGeos(query_id, query_pts);
    for (const auto &bev_lane : data_->lane_infos) {
      if (bev_lane.id == query_id || topo_lane_ids.find(bev_lane.id) != topo_lane_ids.end()) {
        continue;
      }

      auto lane_pts = *(bev_lane.geos);
      bool has_left = LaneGeometry::JudgeIsLeft(lane_pts, query_pts);
      has_pos_left.insert(has_left);
    }

    if (has_pos_left.find(false) == has_pos_left.end()) {
      position = "most_right";
    }

    if (has_pos_left.find(true) == has_pos_left.end()) {
      position = "most_left";
    }
    return position;
  };

  std::vector<BevHarborTopology> bev_harbor_topo_infos;
  // 获取地图harbor信息
  navigation::HarborStopInfo harbor_stop_info = GetHarborStopInfo();

  // 获取感知harbor信息
  std::unordered_map<uint64_t, std::set<uint64_t>> merge_prev_ids;
  std::unordered_map<uint64_t, std::set<uint64_t>> split_next_ids;
  for (const auto &bev_lane : data_->lane_infos) {
    if (!bev_lane.id)
      continue;
    if (bev_lane.next_lane_ids.size() == 2) {
      std::set<uint64_t> next_ids = {bev_lane.next_lane_ids[0], bev_lane.next_lane_ids[1]};
      split_next_ids.insert({bev_lane.id, next_ids});
    }

    if (bev_lane.previous_lane_ids.size() == 2) {
      std::set<uint64_t> prev_ids = {bev_lane.previous_lane_ids[0], bev_lane.previous_lane_ids[1]};
      merge_prev_ids.insert({bev_lane.id, prev_ids});
    }
  }

  for (const auto &merge_pair : merge_prev_ids) {
    BevHarborTopology bev_harbor_topo_info;
    for (const auto &split_pair : split_next_ids) {
      if (merge_pair.second == split_pair.second) {
        std::set<uint64_t>           topo_lane_ids;
        std::vector<Eigen::Vector2f> sub_1_pts;
        std::vector<Eigen::Vector2f> sub_2_pts;

        uint64_t sub_1_id = *split_pair.second.begin();
        uint64_t sub_2_id = *split_pair.second.rbegin();

        GetBevLaneGeos(sub_1_id, sub_1_pts);
        GetBevLaneGeos(sub_2_id, sub_2_pts);

        bool is_left = LaneGeometry::JudgeIsLeft(sub_1_pts, sub_2_pts);

        bev_harbor_topo_info.merge_main_id = merge_pair.first;
        bev_harbor_topo_info.split_main_id = split_pair.first;
        bev_harbor_topo_info.topo_sub_ids  = split_pair.second;

        if (is_left) {
          bev_harbor_topo_info.sub_left_id  = sub_1_id;
          bev_harbor_topo_info.sub_right_id = sub_2_id;
        } else {
          bev_harbor_topo_info.sub_right_id = sub_1_id;
          bev_harbor_topo_info.sub_left_id  = sub_2_id;
        }

        topo_lane_ids = {bev_harbor_topo_info.merge_main_id, bev_harbor_topo_info.split_main_id};

        if (get_lane_position(bev_harbor_topo_info.sub_left_id, topo_lane_ids) == "most_left" &&
            get_lane_position(bev_harbor_topo_info.sub_right_id, topo_lane_ids) == "most_right") {
          bev_harbor_topo_info.harbor_position = "most_right";
          bev_harbor_topo_info.harbor_lane_id  = bev_harbor_topo_info.sub_right_id;

        } else if (get_lane_position(bev_harbor_topo_info.sub_left_id, topo_lane_ids) != "most_left" &&
                   get_lane_position(bev_harbor_topo_info.sub_right_id, topo_lane_ids) == "most_right") {
          bev_harbor_topo_info.harbor_position = "most_right";
          bev_harbor_topo_info.harbor_lane_id  = bev_harbor_topo_info.sub_right_id;

        } else if (get_lane_position(bev_harbor_topo_info.sub_left_id, topo_lane_ids) == "most_left" &&
                   get_lane_position(bev_harbor_topo_info.sub_right_id, topo_lane_ids) != "most_right") {
          bev_harbor_topo_info.harbor_position = "most_left";
          bev_harbor_topo_info.harbor_lane_id  = bev_harbor_topo_info.sub_left_id;

        } else {
          bev_harbor_topo_info.harbor_position = "none";
          bev_harbor_topo_info.harbor_lane_id  = 0;
        }

        std::vector<Eigen::Vector2f> harbor_geos;
        GetBevLaneGeos(bev_harbor_topo_info.harbor_lane_id, harbor_geos);

        if (!harbor_geos.empty()) {
          bev_harbor_topo_info.range = {harbor_geos.front().x(), harbor_geos.back().x()};
        }
        bev_harbor_topo_infos.emplace_back(bev_harbor_topo_info);
        break;
      }
    }
  }

  for (auto &harbor_info : bev_harbor_topo_infos) {
    if (!harbor_info.harbor_lane_id)
      continue;

    uint64_t harbor_lane_id    = 0;
    uint64_t normal_lane_id    = 0;
    bool     range_has_overlap = has_overlap(harbor_info.range, harbor_stop_info.ranges);

    // AINFO << "range_has_overlap: " << range_has_overlap
    //       << " harbor_position: " << harbor_info.harbor_position
    //       << " is_left_most: " << harbor_stop_info.is_left_most
    //       << " is_right_most: " << harbor_stop_info.is_right_most;

    if (range_has_overlap && harbor_info.harbor_position == "most_left" && harbor_stop_info.is_left_most) {
      harbor_lane_id = harbor_info.sub_left_id;
      normal_lane_id = harbor_info.sub_right_id;
    }

    if (range_has_overlap && harbor_info.harbor_position == "most_right" && harbor_stop_info.is_right_most) {
      harbor_lane_id = harbor_info.sub_right_id;
      normal_lane_id = harbor_info.sub_left_id;
    }

    auto harbor_lane_iter =
        std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(), [&](const BevLaneInfo &p) { return p.id == harbor_lane_id; });

    auto normal_lane_iter =
        std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(), [&](const BevLaneInfo &p) { return p.id == normal_lane_id; });

    if (harbor_lane_iter != data_->lane_infos.end() && normal_lane_iter != data_->lane_infos.end()) {
      if (harbor_info.harbor_position == "most_left") {
        harbor_lane_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
        harbor_lane_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT;

        normal_lane_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
        normal_lane_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
      }

      if (harbor_info.harbor_position == "most_right") {
        harbor_lane_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
        harbor_lane_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;

        normal_lane_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
        normal_lane_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
      }
      harbor_valid = true;
    }
  }
  return harbor_valid;
}

bool LaneTypeChecker::CheckTopoUseSemanticInfo() {
  /*
       * merge拓扑和split拓扑同时存在
       * 凸出的车道为港湾车道
                        * * * * * * *
                       *		     *
          * * * * * * *	* * * * * * * * * * * * * *
    */
  bool harbor_valid = HarborTopologyOperator();

  if (CHECK_TOPO_DEBUG_MODE) {
    AINFO << "harbor_valid: " << harbor_valid;
    for (const auto &bev_lane : data_->lane_infos) {
      AINFO << "[harbor] bev_lane id: " << bev_lane.id << " split lane type: " << static_cast<int>(bev_lane.split_topo_extend)
            << " merge lane type: " << static_cast<int>(bev_lane.merge_topo_extend);
    }
  }
  return harbor_valid;
}

navigation::HarborStopInfo LaneTypeChecker::GetHarborStopInfo() {
  navigation::HarborStopInfo result;
  // 使用LD地图数据替换SD地图数据
  auto ld_route_info = INTERNAL_PARAMS.ld_map_data.GetLDRouteInfoPtr();
  if (!ld_route_info) {
    AWARN << "[LDNOA] Failed: ld_route_info is null!";
    return result;
  }

  // 获取自车位置信息
  uint64_t ego_section_id = ld_route_info->navi_start.section_id;
  double   ego_s_offset   = ld_route_info->navi_start.s_offset;
  // SD_COARSE_MATCH_LOG << "[GetHarborStopInfo] Ego section ID: " << ego_section_id << ", Ego s_offset: " << ego_s_offset;

  // 计算自车全局s坐标
  double ego_global_s      = 0.0;
  bool   ego_section_found = false;
  for (const auto &section : ld_route_info->sections) {
    if (section.id == ego_section_id) {
      ego_global_s += ego_s_offset;
      ego_section_found = true;
      break;
    }
    ego_global_s += section.length;
  }
  if (!ego_section_found) {
    AWARN << "[LDNOA] Failed: ego_section not found!";
    return result;
  }
  // SD_COARSE_MATCH_LOG << "[GetHarborStopInfo] Ego global s: " << ego_global_s;

  // 定义检测范围
  const double min_s = -100.0;  // -100.0
  const double max_s = 300.0;   // 300.0

  // 初始化港湾停靠站信息
  result.exists        = false;
  result.is_left_most  = false;
  result.is_right_most = false;
  result.ranges.clear();

  // 存储港湾停靠站区间
  std::vector<std::pair<double, double>> harbor_stop_ranges;
  bool                                   current_leftmost  = false;
  bool                                   current_rightmost = false;

  // 遍历所有路段
  double current_s = 0.0;
  for (const auto &section : ld_route_info->sections) {
    const double section_start_s = current_s;
    const double section_end_s   = current_s + section.length;
    current_s                    = section_end_s;  // 更新当前s位置

    // 跳过不在检测范围内的路段
    if (section_end_s < ego_global_s + min_s || section_start_s > ego_global_s + max_s) {
      continue;
    }
    //SD_COARSE_MATCH_LOG << "[GetHarborStopInfo] Section ID: " << section.id
    //                   << " start s: " << section_start_s << " end s: " << section_end_s;

    // 收集当前section的所有车道,直接处理车道
    std::vector<const cem::message::env_model::LaneInfo *> section_lanes;
    for (uint64_t lane_id : section.lane_ids) {
      // 使用LD地图获取车道信息
      auto lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_id);
      if (lane_info) {
        section_lanes.push_back(lane_info);
      }
    }

    if (section_lanes.empty())
      continue;

    // 检测港湾停靠站并记录位置
    for (size_t idx = 0; idx < section_lanes.size(); ++idx) {
      const auto *lane = section_lanes[idx];
      if (lane->type == cem::message::env_model::LaneType::LANE_HARBOR_STOP) {
        // 计算车道在相对坐标系中的位置
        const double lane_start_s = std::max(section_start_s - ego_global_s, min_s);
        const double lane_end_s   = std::min(section_end_s - ego_global_s, max_s);

        if (lane_start_s < lane_end_s) {
          // 合并连续区间
          if (!harbor_stop_ranges.empty() && lane_start_s <= harbor_stop_ranges.back().second) {
            harbor_stop_ranges.back().second = std::max(harbor_stop_ranges.back().second, lane_end_s);
          } else {
            harbor_stop_ranges.emplace_back(lane_start_s, lane_end_s);
          }

          // 检查是否在最左或最右位置
          if (idx == 0) {
            current_leftmost = true;
          }
          if (idx == section_lanes.size() - 1) {
            current_rightmost = true;
          }

          // SD_COARSE_MATCH_LOG << " [GetHarborStopInfo] Harbor stop lane detected, lane_seq: " << lane->lane_seq << ", range: ["
          //                     << lane_start_s << ", " << lane_end_s << "]";
        }
      }
    }
  }

  // 填充结果
  result.ranges        = harbor_stop_ranges;
  result.exists        = !harbor_stop_ranges.empty();
  result.is_left_most  = current_leftmost;
  result.is_right_most = current_rightmost;

  // 调试日志
  if (CHECK_TOPO_DEBUG_MODE) {
    if (result.exists) {
      AINFO << "[GetHarborStopInfo] Harbor stop exists in merged ranges:";
      for (const auto &range : result.ranges) {
        AINFO << "  [" << range.first << ", " << range.second << "]";
      }
      AINFO << "  is_left_most: " << result.is_left_most
            << ", is_right_most: " << result.is_right_most;
    } else {
      AINFO << "[GetHarborStopInfo] No harbor stop found.";
    }
  }
  return result;
}

uint64_t LaneTypeChecker::GetAdjoinLaneUseGeos(const uint64_t query_id, const uint64_t sub_lane_1_id, const uint64_t sub_lane_2_id, std::string& type)
{
  type = "";
  uint64_t adjoin_lane_id = 0;

  auto GetLanemarkerGeos = [&](const std::vector<cem::message::common::Point2DF>& lanemarker_pts) -> std::vector<Eigen::Vector2f> {

    std::vector<Eigen::Vector2f> lane_pts;
    for(const auto& point : lanemarker_pts)
    {
      Eigen::Vector2f pt;
      pt.x() = point.x;
      pt.y() = point.y;
      lane_pts.push_back(pt);
    }
    return lane_pts;
  };

  if(query_id == 0 || data_ == nullptr)
  {
    return 0;
  }

  uint64_t query_left_lanemarker_id = 0;
  uint64_t query_right_lanemarker_id = 0;

  std::vector<Eigen::Vector2f> query_lane_pts;
  std::vector<Eigen::Vector2f> query_sub_lane_1_pts;
  std::vector<Eigen::Vector2f> query_sub_lane_2_pts;
  std::unordered_map<uint64_t, std::vector<Eigen::Vector2f>> bev_lane_geos;
  std::unordered_map<uint64_t, std::vector<Eigen::Vector2f>> bev_lanemarker_geos;

  std::set<uint64_t> topo_lane_ids = {query_id, sub_lane_1_id, sub_lane_2_id};

  for(const auto& bev_lane : data_->lane_infos)
  {
    if(!bev_lane.id) continue;
    auto bev_lane_pts = *(bev_lane.geos);
    std::sort(bev_lane_pts.begin(), bev_lane_pts.end(),
              [](Eigen::Vector2f& pt1, Eigen::Vector2f& pt2){return pt1.x() < pt2.x();});
    if(topo_lane_ids.find(bev_lane.id) == topo_lane_ids.end())
    {
      bev_lane_geos.insert({bev_lane.id, bev_lane_pts});
    }else{
      if(bev_lane.id == query_id)
      {
        query_left_lanemarker_id  = bev_lane.left_lane_marker_id;
        query_right_lanemarker_id = bev_lane.right_lane_marker_id;
        query_lane_pts = bev_lane_pts;
      }
      if(bev_lane.id == sub_lane_1_id)
      {
        query_sub_lane_1_pts = bev_lane_pts;
      }
      if(bev_lane.id == sub_lane_2_id)
      {
        query_sub_lane_2_pts = bev_lane_pts;
      }
    }
  }

  for(const auto& bev_lanemarker : data_->lanemarkers)
  {
    if(!bev_lanemarker.id || bev_lanemarker.id == query_left_lanemarker_id ||
        bev_lanemarker.id == query_right_lanemarker_id){
      continue;
    }
    auto bev_lanemarker_pts = GetLanemarkerGeos(bev_lanemarker.line_points);
    if(!bev_lanemarker_pts.empty())
    {
      std::sort(bev_lanemarker_pts.begin(), bev_lanemarker_pts.end(),
                [](Eigen::Vector2f& pt1, Eigen::Vector2f& pt2){return pt1.x() < pt2.x();});
      bev_lanemarker_geos.insert({bev_lanemarker.id, bev_lanemarker_pts});
    }
  }

  for(auto& pair : bev_lane_geos)
  {
    std::pair<float, float> left_near_lane_dist = {0.0, 0.0};
    if(!pair.second.empty())
    {
      bool adjoin_valid =
          cem::fusion::LaneGeometry::CalculateAverageDistanceBetweenTwoLines(pair.second,
                                                                             query_lane_pts,
                                                                             left_near_lane_dist);
      bool is_left = LaneGeometry::JudgeIsLeft(pair.second, query_lane_pts);

      if(is_left && (left_near_lane_dist.first > 2 && left_near_lane_dist.first < 5))
      {
        adjoin_lane_id = pair.first;
        type = "lane";
        break;
      }
    }
  }

  if(adjoin_lane_id == 0)
  {
    for(auto& pair : bev_lanemarker_geos)
    {
      std::pair<float, float> left_near_right_lanemarker_dist = {0.0, 0.0};
      if(!pair.second.empty())
      {
        bool adjoin_valid =
            cem::fusion::LaneGeometry::CalculateAverageDistanceBetweenTwoLines(pair.second,
                                                                               query_lane_pts,
                                                                               left_near_right_lanemarker_dist);
        // AINFO << "[lanemarker] id: " << pair.first;
        // for(const auto& lm_pts : pair.second)
        // {
        //   AINFO << "[lanemarker]: " << lm_pts.x() << "," << lm_pts.y();
        // }
        //
        // AINFO << "[lane] id: " << query_id;
        // for(const auto& lane_pts: query_lane_pts)
        // {
        //   AINFO << "[lane]: " << lane_pts.x() << "," << lane_pts.y();
        // }

        bool lm_is_left = false;
        bool query_lm_main_is_left        = LaneGeometry::JudgeIsLeft(pair.second, query_lane_pts);
        bool query_lm_sub_lane_1_is_left  = LaneGeometry::JudgeIsLeft(pair.second, query_sub_lane_1_pts);
        bool query_lm_sub_lane_2_is_left  = LaneGeometry::JudgeIsLeft(pair.second, query_sub_lane_2_pts);

        if(query_lm_main_is_left && query_lm_sub_lane_1_is_left && query_lm_sub_lane_2_is_left)
        {
          lm_is_left = true;
        }
        if(lm_is_left && (left_near_right_lanemarker_dist.first > 0.0 && left_near_right_lanemarker_dist.first < 2.5))
        {
          adjoin_lane_id = pair.first;
          type = "lanemarker";
          break;
        }
      }
    }
  }
  return adjoin_lane_id;
}

bool LaneTypeChecker::CheckTopoUseSpecialLane(const uint64_t sub_lane_1_id, const uint64_t sub_lane_2_id, const int type) {
  bool special_lane_valid_flag = false;
  if (!sub_lane_1_id || !sub_lane_2_id || data_ == nullptr) {
    return false;
  }

  std::set<uint64_t> harbor_lane_set;
  std::set<uint64_t> emergency_lane_set;

  for (const auto &bev_lane : data_->lane_infos) {
    if (!bev_lane.id)
      continue;
    if (bev_lane.lane_type == BevLaneType::LANE_TYPE_EMERGENCY || bev_lane.lane_type == BevLaneType::LANE_TYPE_HARBOR_STOP) {
      // AINFO << "[ecy]: " << bev_lane.id;
      emergency_lane_set.insert(bev_lane.id);
    }
  }

  auto temp_lane_1 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                  [&](const cem::message::sensor::BevLaneInfo &lane) { return sub_lane_1_id == lane.id; });

  auto temp_lane_2 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                  [&](const cem::message::sensor::BevLaneInfo &lane) { return sub_lane_2_id == lane.id; });

  if (temp_lane_1 == data_->lane_infos.end() || temp_lane_2 == data_->lane_infos.end()) {
    return false;
  }

  auto sub_lane_1_pts = *(temp_lane_1->geos);
  auto sub_lane_2_pts = *(temp_lane_2->geos);

  if ((temp_lane_1->line_points.size() < 2) || (temp_lane_2->line_points.size() < 2) || (sub_lane_1_pts.size() < 2) ||
      (sub_lane_2_pts.size() < 2)) {
    return false;
  }

  // 计算两条目标车道的横向距离，排左右
  Eigen::Vector2d temp_lane_1_mid, temp_lane_2_mid, dir_temp_lane_1;
  dir_temp_lane_1.x() = temp_lane_1->line_points.back().x - temp_lane_1->line_points.front().x;
  dir_temp_lane_1.y() = temp_lane_1->line_points.back().y - temp_lane_1->line_points.front().y;
  dir_temp_lane_1     = dir_temp_lane_1 / dir_temp_lane_1.norm();

  temp_lane_1_mid.x() = (temp_lane_1->line_points.front().x + temp_lane_1->line_points.back().x) / 2;
  temp_lane_1_mid.y() = (temp_lane_1->line_points.front().y + temp_lane_1->line_points.back().y) / 2;
  temp_lane_2_mid.x() = (temp_lane_2->line_points.front().x + temp_lane_2->line_points.back().x) / 2;
  temp_lane_2_mid.y() = (temp_lane_2->line_points.front().y + temp_lane_2->line_points.back().y) / 2;

  auto   dist_vec      = temp_lane_2_mid - temp_lane_1_mid;
  double dist_lat_sign = copysign(1.0, dir_temp_lane_1.x() * dist_vec.y() - dir_temp_lane_1.y() * dist_vec.x());
  bool   sub_1_is_left = LaneGeometry::JudgeIsLeft(sub_lane_1_pts, sub_lane_2_pts);
  bool   sub_2_is_left = LaneGeometry::JudgeIsLeft(sub_lane_2_pts, sub_lane_1_pts);

  if (type == split_mode) {
    // 规则1：语义规则，强制将emergency 和 harbor lane拓扑属性设置为split
    if ((emergency_lane_set.count(temp_lane_1->id) && !emergency_lane_set.count(temp_lane_2->id)) ||
        (harbor_lane_set.count(temp_lane_1->id) && !harbor_lane_set.count(temp_lane_2->id))) {
      if (sub_1_is_left) {
        temp_lane_1->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
        temp_lane_2->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
      } else {
        temp_lane_1->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
        temp_lane_2->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
      }
      special_lane_valid_flag = true;
    } else if ((!emergency_lane_set.count(temp_lane_1->id) && emergency_lane_set.count(temp_lane_2->id)) ||
               (!harbor_lane_set.count(temp_lane_1->id) && harbor_lane_set.count(temp_lane_2->id))) {
      if (sub_2_is_left) {
        temp_lane_1->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
        temp_lane_2->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
      } else {
        temp_lane_1->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
        temp_lane_2->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
      }
      special_lane_valid_flag = true;
    }
  }

  if (type == merge_mode) {
    // 规则1：语义规则，强制将emergency 和 harbor lane拓扑属性设置为merge
    if ((emergency_lane_set.count(temp_lane_1->id) && !emergency_lane_set.count(temp_lane_2->id)) ||
        (harbor_lane_set.count(temp_lane_1->id) && !harbor_lane_set.count(temp_lane_2->id))) {
      if (sub_1_is_left) {
        temp_lane_1->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT;
        temp_lane_2->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
      } else {
        temp_lane_1->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
        temp_lane_2->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
      }
      special_lane_valid_flag = true;
    } else if ((!emergency_lane_set.count(temp_lane_1->id) && emergency_lane_set.count(temp_lane_2->id)) ||
               (!harbor_lane_set.count(temp_lane_1->id) && harbor_lane_set.count(temp_lane_2->id))) {
      if (sub_2_is_left) {
        temp_lane_1->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
        temp_lane_2->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT;
      } else {
        temp_lane_1->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
        temp_lane_2->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
      }
      special_lane_valid_flag = true;
    }
  }
  return special_lane_valid_flag;
}

bool LaneTypeChecker::CheckTopoUseMatchers(const uint64_t topo_lane_id, const int type) {
  bool matcher_valid = false;
  auto lane_iter     = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                    [&](const cem::message::sensor::BevLaneInfo &lane) { return topo_lane_id == lane.id; });
  if (lane_iter == data_->lane_infos.end()) {
    return false;
  }

  // 处理split拓扑
  if (type == split_mode) {
    auto match_vec_infos = SplitGetter(topo_lane_id);
    std::sort(match_vec_infos.begin(), match_vec_infos.end(),
              [](const BevSplitInfo *split_a, const BevSplitInfo *split_b) { return split_a->distance_to_ego < split_b->distance_to_ego; });

    std::vector<const BevMergeInfo *> _match_infos;
    bool match_valid = CheckMatchInfo(match_vec_infos, _match_infos, split_mode);
    if (match_valid && lane_iter->next_lane_ids.size() == 2 && !match_vec_infos.empty()) {
      for (auto &match_info : match_vec_infos) {
        if (CHECK_TOPO_DEBUG_MODE) {
          AINFO << "split match_info bev_lane_from: " << match_info->bev_lane_from;
          AINFO << "split match_info bev_lane_to size: " << match_info->bev_target_to.size();
          AINFO << "split match_info distance_to_ego: " << match_info->distance_to_ego;
          for (auto &split_lane : match_info->bev_target_to) {
            AINFO << "split match_info topo: " << split_lane.first << " match bev_lane_to:" << split_lane.second.bev_lane_to
                  << "match_info type: " << split_lane.second.type;
          }
        }

        if (match_info->bev_target_to.size() == 1) {
          uint64_t split_main_lane_id = match_info->bev_lane_from;

          uint64_t split_sub_lane_1 = lane_iter->next_lane_ids[0];
          uint64_t split_sub_lane_2 = lane_iter->next_lane_ids[1];

          LdMapProcessor::MergeSplitType split_sub_lane_1_type = LdMapProcessor::MergeSplitType::kConnectForward;
          LdMapProcessor::MergeSplitType split_sub_lane_2_type = LdMapProcessor::MergeSplitType::kConnectForward;

          if (lane_iter->next_lane_ids[0] == match_info->bev_target_to[0].second.bev_lane_to) {
            split_sub_lane_1_type = match_info->bev_target_to[0].second.type;
          }

          if (lane_iter->next_lane_ids[1] == match_info->bev_target_to[0].second.bev_lane_to) {
            split_sub_lane_2_type = match_info->bev_target_to[0].second.type;
          }

          if (CHECK_TOPO_DEBUG_MODE) {
            AINFO << "split_sub_lane_1: " << split_sub_lane_1 << " type: " << static_cast<int>(split_sub_lane_1_type);
            AINFO << "split_sub_lane_2: " << split_sub_lane_2 << " type: " << static_cast<int>(split_sub_lane_2_type);
          }

          auto split_sub_lane_1_exist = std::find(lane_iter->next_lane_ids.begin(), lane_iter->next_lane_ids.end(), split_sub_lane_1);

          auto split_sub_lane_2_exist = std::find(lane_iter->next_lane_ids.begin(), lane_iter->next_lane_ids.end(), split_sub_lane_2);

          if (split_sub_lane_1_exist != lane_iter->next_lane_ids.end() && split_sub_lane_2_exist != lane_iter->next_lane_ids.end()) {
            auto sp_sub_1_iter = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                              [&](const cem::message::sensor::BevLaneInfo &lane) { return split_sub_lane_1 == lane.id; });

            auto sp_sub_2_iter = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                              [&](const cem::message::sensor::BevLaneInfo &lane) { return split_sub_lane_2 == lane.id; });

            if (sp_sub_1_iter != data_->lane_infos.end() && sp_sub_2_iter != data_->lane_infos.end()) {
              bool                         sub_1_is_left           = false;
              std::vector<Eigen::Vector2f> split_sub_lane_1_points = *(sp_sub_1_iter->geos);
              std::vector<Eigen::Vector2f> split_sub_lane_2_points = *(sp_sub_2_iter->geos);

              if (split_sub_lane_1_type == LdMapProcessor::MergeSplitType::kConnectForward) {
                if (split_sub_lane_1_points.size() < 2 || split_sub_lane_2_points.size() < 2) {
                  continue;
                }

                sub_1_is_left = LaneGeometry::JudgeIsLeft(split_sub_lane_1_points, split_sub_lane_2_points);
                if (sub_1_is_left) {
                  sp_sub_1_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
                  sp_sub_2_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
                } else {
                  sp_sub_1_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
                  sp_sub_2_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
                }
              } else if (split_sub_lane_2_type == LdMapProcessor::MergeSplitType::kConnectForward) {
                if (split_sub_lane_1_points.size() < 2 || split_sub_lane_2_points.size() < 2) {
                  continue;
                }

                sub_1_is_left = LaneGeometry::JudgeIsLeft(split_sub_lane_1_points, split_sub_lane_2_points);
                if (sub_1_is_left) {
                  sp_sub_1_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
                  sp_sub_2_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
                } else {
                  sp_sub_1_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
                  sp_sub_2_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
                }
              } else if (split_sub_lane_1_type == LdMapProcessor::MergeSplitType::kSplitLeft) {
                sp_sub_1_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
                sp_sub_2_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
              } else if (split_sub_lane_1_type == LdMapProcessor::MergeSplitType::kSplitRight) {
                sp_sub_1_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
                sp_sub_2_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
              } else if (split_sub_lane_2_type == LdMapProcessor::MergeSplitType::kSplitLeft) {
                sp_sub_1_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
                sp_sub_2_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
              } else if (split_sub_lane_2_type == LdMapProcessor::MergeSplitType::kSplitRight) {
                sp_sub_1_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
                sp_sub_2_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
              }
              matcher_valid = true;
            }
          }
        }

        if (match_info->bev_target_to.size() == 2) {
          uint64_t split_main_lane_id = match_info->bev_lane_from;
          uint64_t split_sub_lane_1   = match_info->bev_target_to[0].second.bev_lane_to;
          uint64_t split_sub_lane_2   = match_info->bev_target_to[1].second.bev_lane_to;

          auto split_sub_lane_1_type = match_info->bev_target_to[0].second.type;
          auto split_sub_lane_2_type = match_info->bev_target_to[1].second.type;

          auto split_sub_lane_1_exist = std::find(lane_iter->next_lane_ids.begin(), lane_iter->next_lane_ids.end(), split_sub_lane_1);
          auto split_sub_lane_2_exist = std::find(lane_iter->next_lane_ids.begin(), lane_iter->next_lane_ids.end(), split_sub_lane_2);

          if (split_sub_lane_1_exist != lane_iter->next_lane_ids.end() && split_sub_lane_2_exist != lane_iter->next_lane_ids.end()) {
            auto sp_sub_1_iter = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                              [&](const cem::message::sensor::BevLaneInfo &lane) { return split_sub_lane_1 == lane.id; });

            auto sp_sub_2_iter = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                              [&](const cem::message::sensor::BevLaneInfo &lane) { return split_sub_lane_2 == lane.id; });

            if (sp_sub_1_iter != data_->lane_infos.end() && sp_sub_2_iter != data_->lane_infos.end()) {
              if (split_sub_lane_1_type == LdMapProcessor::MergeSplitType::kSplitLeft &&
                  split_sub_lane_2_type == LdMapProcessor::MergeSplitType::kConnectForward) {
                sp_sub_1_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
                sp_sub_2_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
              } else if (split_sub_lane_1_type == LdMapProcessor::MergeSplitType::kSplitRight &&
                         split_sub_lane_2_type == LdMapProcessor::MergeSplitType::kConnectForward) {
                sp_sub_1_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
                sp_sub_2_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;

              } else if (split_sub_lane_2_type == LdMapProcessor::MergeSplitType::kSplitLeft &&
                         split_sub_lane_1_type == LdMapProcessor::MergeSplitType::kConnectForward) {
                sp_sub_1_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
                sp_sub_2_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;

              } else if (split_sub_lane_2_type == LdMapProcessor::MergeSplitType::kSplitRight &&
                         split_sub_lane_1_type == LdMapProcessor::MergeSplitType::kConnectForward) {
                sp_sub_1_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
                sp_sub_2_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
              }
              matcher_valid = true;
            }
          }
        }
      }
    }
  }

  // 处理merge拓扑
  if (type == merge_mode) {
    auto match_vec_infos = MergeGetter(topo_lane_id, true);
    std::sort(match_vec_infos.begin(), match_vec_infos.end(),
              [](const BevMergeInfo *merge_a, const BevMergeInfo *merge_b) { return merge_a->distance_to_ego < merge_b->distance_to_ego; });
    std::vector<const BevSplitInfo *> _match_infos;
    bool match_valid = CheckMatchInfo(_match_infos, match_vec_infos, merge_mode);
    if (match_valid && !lane_iter->previous_lane_ids.empty() && !match_vec_infos.empty()) {
      for (auto &match_info : match_vec_infos) {
        if (CHECK_TOPO_DEBUG_MODE) {
          AINFO << "merge match_vec_infos SIZE: " << match_vec_infos.size();
          AINFO << "merge match_info bev_lane_to: " << match_info->bev_lane_to;
          AINFO << "merge match_info bev_lane_to size: " << match_info->bev_lanes_from.size();
          AINFO << "merge match_info distance_to_ego: " << match_info->distance_to_ego;
        }
        if (match_info->bev_lanes_from.size() == 1) {
          for (auto &merge_lane : match_info->bev_lanes_from) {
            if (CHECK_TOPO_DEBUG_MODE) {
              AINFO << "merge match_info topo: " << merge_lane.first << " merge bev_lane_from: " << merge_lane.second.bev_lane_from
                    << "merge type: " << merge_lane.second.type;
            }
            if (merge_lane.first == 0)
              continue;
            uint64_t merge_main_lane_id = topo_lane_id;

            auto merge_main_lane_iter = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                                     [&](const BevLaneInfo &p) { return p.id == merge_main_lane_id; });

            if (merge_main_lane_iter == data_->lane_infos.end() || merge_main_lane_iter->previous_lane_ids.size() != 2) {
              continue;
            }

            LdMapProcessor::MergeSplitType merge_sub_lane_1_type = LdMapProcessor::MergeSplitType::kConnectForward;
            LdMapProcessor::MergeSplitType merge_sub_lane_2_type = LdMapProcessor::MergeSplitType::kConnectForward;

            uint64_t merge_sub_lane_1_id = merge_main_lane_iter->previous_lane_ids[0];
            uint64_t merge_sub_lane_2_id = merge_main_lane_iter->previous_lane_ids[1];

            if (merge_sub_lane_1_id == merge_lane.second.bev_lane_from) {
              merge_sub_lane_1_type = merge_lane.second.type;
            }

            if (merge_sub_lane_2_id == merge_lane.second.bev_lane_from) {
              merge_sub_lane_2_type = merge_lane.second.type;
            }

            auto merge_sub_lane_1_iter =
                std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                             [&](const cem::message::sensor::BevLaneInfo &lane) { return merge_sub_lane_1_id == lane.id; });

            auto merge_sub_lane_2_iter =
                std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                             [&](const cem::message::sensor::BevLaneInfo &lane) { return merge_sub_lane_2_id == lane.id; });

            if (merge_sub_lane_1_iter != data_->lane_infos.end() && merge_sub_lane_2_iter != data_->lane_infos.end()) {
              bool                         sub_1_is_left           = false;
              std::vector<Eigen::Vector2f> merge_sub_lane_1_points = *(merge_sub_lane_1_iter->geos);
              std::vector<Eigen::Vector2f> merge_sub_lane_2_points = *(merge_sub_lane_2_iter->geos);

              if (merge_sub_lane_1_type == LdMapProcessor::MergeSplitType::kConnectForward) {
                if (merge_sub_lane_1_points.size() < 2 || merge_sub_lane_2_points.size() < 2) {
                  continue;
                }
                sub_1_is_left = LaneGeometry::JudgeIsLeft(merge_sub_lane_1_points, merge_sub_lane_2_points);
                if (sub_1_is_left) {
                  merge_sub_lane_1_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
                  merge_sub_lane_2_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
                } else {
                  merge_sub_lane_1_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
                  merge_sub_lane_2_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT;
                }
              } else if (merge_sub_lane_2_type == LdMapProcessor::MergeSplitType::kConnectForward) {
                if (merge_sub_lane_1_points.size() < 2 || merge_sub_lane_2_points.size() < 2) {
                  continue;
                }
                sub_1_is_left = LaneGeometry::JudgeIsLeft(merge_sub_lane_1_points, merge_sub_lane_2_points);
                if (sub_1_is_left) {
                  merge_sub_lane_1_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT;
                  merge_sub_lane_2_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
                } else {
                  merge_sub_lane_1_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
                  merge_sub_lane_2_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
                }
              } else if (merge_sub_lane_1_type == LdMapProcessor::MergeSplitType::kMergeLeft) {
                merge_sub_lane_1_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
                merge_sub_lane_2_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
              } else if (merge_sub_lane_1_type == LdMapProcessor::MergeSplitType::kMergeRight) {
                merge_sub_lane_1_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT;
                merge_sub_lane_2_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
              } else if (merge_sub_lane_2_type == LdMapProcessor::MergeSplitType::kMergeLeft) {
                merge_sub_lane_2_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
                merge_sub_lane_1_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
              } else if (merge_sub_lane_2_type == LdMapProcessor::MergeSplitType::kMergeRight) {
                merge_sub_lane_2_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT;
                merge_sub_lane_1_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
              }
              matcher_valid = true;
            }
            // AINFO << "DEBUG 2 merge_sub_lane_1: " << merge_sub_lane_1_iter->id << " merge_topo_extend: " << static_cast<int>(merge_sub_lane_1_iter->merge_topo_extend);
            // AINFO << "DEBUG 2 merge_sub_lane_2: " << merge_sub_lane_2_iter->id << " merge_topo_extend: " << static_cast<int>(merge_sub_lane_2_iter->merge_topo_extend);
          }
        }

        if (match_info->bev_lanes_from.size() == 2) {
          for (auto &merge_lane : match_info->bev_lanes_from) {
            if (CHECK_TOPO_DEBUG_MODE) {
              AINFO << "merge match_info topo: " << merge_lane.first << " merge bev_lane_from: " << merge_lane.second.bev_lane_from
                    << "merge type: " << merge_lane.second.type;
            }

            if (merge_lane.first == 0)
              continue;
            uint64_t merge_sub_lane_id   = merge_lane.second.bev_lane_from;
            auto     merge_sub_lane_type = merge_lane.second.type;

            auto merge_sub_lane_iter =
                std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                             [&](const cem::message::sensor::BevLaneInfo &lane) { return merge_sub_lane_id == lane.id; });

            if (merge_sub_lane_iter != data_->lane_infos.end()) {
              if (merge_sub_lane_type == LdMapProcessor::MergeSplitType::kMergeLeft) {
                merge_sub_lane_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
              } else if (merge_sub_lane_type == LdMapProcessor::MergeSplitType::kMergeRight) {
                merge_sub_lane_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT;
              } else if (merge_sub_lane_type == LdMapProcessor::MergeSplitType::kConnectForward) {
                merge_sub_lane_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
              }
              matcher_valid = true;
            }
          }
        }
      }
    }
  }
  return matcher_valid;
}

bool LaneTypeChecker::CheckTopoUseLeftAdjoinLane(const uint64_t m_lane_id, const uint64_t sub_lane_1_id, const uint64_t sub_lane_2_id,
                                                 const int type) {
  if (!m_lane_id || !sub_lane_1_id || !sub_lane_2_id || data_ == nullptr) {
    return false;
  }

  bool               adjoin_lane_vaild = false;
  std::set<uint64_t> temp_lane_ids     = {m_lane_id, sub_lane_1_id, sub_lane_2_id};

  auto m_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                             [&](const cem::message::sensor::BevLaneInfo &lane) { return m_lane_id == lane.id; });

  auto temp_lane_1 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                  [&](const cem::message::sensor::BevLaneInfo &lane) { return sub_lane_1_id == lane.id; });

  auto temp_lane_2 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                  [&](const cem::message::sensor::BevLaneInfo &lane) { return sub_lane_2_id == lane.id; });

  if (m_lane == data_->lane_infos.end() || temp_lane_1 == data_->lane_infos.end() || temp_lane_2 == data_->lane_infos.end()) {
    return false;
  }

  std::string line_type        = "lane";
  uint64_t    mainlane_left_id = 0;

  if (m_lane->left_lane_id > 0 && !temp_lane_ids.count(m_lane->left_lane_id)) {
    mainlane_left_id = m_lane->left_lane_id;
  } else if (temp_lane_1->left_lane_id > 0 && !temp_lane_ids.count(temp_lane_1->left_lane_id)) {
    mainlane_left_id = temp_lane_1->left_lane_id;
  } else if (temp_lane_2->left_lane_id > 0 && !temp_lane_ids.count(temp_lane_2->left_lane_id)) {
    mainlane_left_id = temp_lane_2->left_lane_id;
  }
  // 计算左侧相邻车道
  if (!mainlane_left_id) {
    mainlane_left_id = GetAdjoinLaneUseGeos(m_lane_id, sub_lane_1_id, sub_lane_2_id, line_type);
    // AINFO << "adjoin_id: " << mainlane_left_id;
  }

  std::pair<float, float> main_left_near_lane_dist = {0.0, 0.0};
  std::pair<float, float> sub1_left_near_lane_dist = {0.0, 0.0};
  std::pair<float, float> sub2_left_near_lane_dist = {0.0, 0.0};

  std::pair<float, float> main_left_near_lanemarker_dist = {0.0, 0.0};
  std::pair<float, float> sub1_left_near_lanemarker_dist = {0.0, 0.0};
  std::pair<float, float> sub2_left_near_lanemarker_dist = {0.0, 0.0};

  if (line_type == "lane") {
    auto mainlane_left = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [&](const cem::message::sensor::BevLaneInfo &lane) { return mainlane_left_id == lane.id; });

    if (mainlane_left != data_->lane_infos.end()) {
      bool main_dist_valid = false;
      bool sub1_dist_valid = false;
      bool sub2_dist_valid = false;

      std::vector<Eigen::Vector2f> mainlane_pts      = *(m_lane->geos);
      std::vector<Eigen::Vector2f> mainlane_left_pts = *(mainlane_left->geos);
      if (!mainlane_pts.empty() && !mainlane_left_pts.empty()) {
        main_dist_valid =
            cem::fusion::LaneGeometry::CalculateAverageDistanceBetweenTwoLines(mainlane_pts, mainlane_left_pts, main_left_near_lane_dist);

        if (main_left_near_lane_dist.first > 2 && main_left_near_lane_dist.first < 5) {

          std::vector<Eigen::Vector2f> sub1lane_pts = *(temp_lane_1->geos);
          sub1_dist_valid =
              cem::fusion::LaneGeometry::CalculateAverageDistanceBetweenTwoLines(sub1lane_pts, mainlane_left_pts, sub1_left_near_lane_dist);

          std::vector<Eigen::Vector2f> sub2lane_pts = *(temp_lane_2->geos);
          sub2_dist_valid =
              cem::fusion::LaneGeometry::CalculateAverageDistanceBetweenTwoLines(sub2lane_pts, mainlane_left_pts, sub2_left_near_lane_dist);
        }

        if (CHECK_TOPO_DEBUG_MODE) {
          AINFO << "line_type:" << line_type;
          AINFO << "adjoin_lane_id: " << mainlane_left_id;
          AINFO << "main_lane_id: " << m_lane_id << ", " << sub_lane_1_id << "," << sub_lane_2_id;
          AINFO << "main_dist_valid: " << main_dist_valid << " main_left_near_lane_dist: " << main_left_near_lane_dist.first << ","
                << main_left_near_lane_dist.second;
          AINFO << "sub1_dist_valid: " << sub1_dist_valid << " sub1_left_near_lane_dist: " << sub1_left_near_lane_dist.first << ","
                << sub1_left_near_lane_dist.second;
          AINFO << "sub2_dist_valid: " << sub2_dist_valid << " sub2_left_near_lane_dist: " << sub2_left_near_lane_dist.first << ","
                << sub2_left_near_lane_dist.second;
        }
      }
    }
  }

  if (line_type == "lanemarker") {
    auto mainlane_left =
        std::find_if(data_->lanemarkers.begin(), data_->lanemarkers.end(),
                     [&](const cem::message::sensor::BevLaneMarker &lanemarker) { return mainlane_left_id == lanemarker.id; });

    if (mainlane_left != data_->lanemarkers.end()) {
      bool main_dist_valid = false;
      bool sub1_dist_valid = false;
      bool sub2_dist_valid = false;

      std::vector<Eigen::Vector2f> main_left_lanemarker_pts;
      std::vector<Eigen::Vector2f> mainlane_pts = *(m_lane->geos);

      for (const auto &point : mainlane_left->line_points) {
        Eigen::Vector2f pt;
        pt.x() = point.x;
        pt.y() = point.y;
        main_left_lanemarker_pts.push_back(pt);
      }

      if (!mainlane_pts.empty() && !main_left_lanemarker_pts.empty()) {
        main_dist_valid = cem::fusion::LaneGeometry::CalculateAverageDistanceBetweenTwoLines(mainlane_pts, main_left_lanemarker_pts,
                                                                                             main_left_near_lanemarker_dist);

        if (main_left_near_lanemarker_dist.first > 0.0 && main_left_near_lanemarker_dist.first < 2.5) {
          std::vector<Eigen::Vector2f> sub1lane_pts = *(temp_lane_1->geos);
          sub1_dist_valid = cem::fusion::LaneGeometry::CalculateAverageDistanceBetweenTwoLines(sub1lane_pts, main_left_lanemarker_pts,
                                                                                               sub1_left_near_lanemarker_dist);

          std::vector<Eigen::Vector2f> sub2lane_pts = *(temp_lane_2->geos);
          sub2_dist_valid = cem::fusion::LaneGeometry::CalculateAverageDistanceBetweenTwoLines(sub2lane_pts, main_left_lanemarker_pts,
                                                                                               sub2_left_near_lanemarker_dist);
        }

        if (CHECK_TOPO_DEBUG_MODE) {
          AINFO << "line_type:" << line_type;
          AINFO << "adjoin_lane_id: " << mainlane_left_id;
          AINFO << "main_lane_id: " << m_lane_id << ", " << sub_lane_1_id << "," << sub_lane_2_id;
          AINFO << "main_dist_valid: " << main_dist_valid << " main_left_near_lanemarker_dist: " << main_left_near_lanemarker_dist.first
                << "," << main_left_near_lanemarker_dist.second;
          AINFO << "sub1_dist_valid: " << sub1_dist_valid << " sub1_left_near_lanemarker_dist: " << sub1_left_near_lanemarker_dist.first
                << "," << sub1_left_near_lanemarker_dist.second;
          AINFO << "sub2_dist_valid: " << sub2_dist_valid << " sub2_left_near_lanemarker_dist: " << sub2_left_near_lanemarker_dist.first
                << "," << sub2_left_near_lanemarker_dist.second;
        }
      }
    }
  }

  if (type == split_mode) {
    if (line_type == "lane") {
      if (sub1_left_near_lane_dist.first < sub2_left_near_lane_dist.first) {
        if (sub1_left_near_lane_dist.first > 2 && sub1_left_near_lane_dist.first < 5) {
          temp_lane_1->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
          temp_lane_2->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
          adjoin_lane_vaild              = true;
        }
      } else {
        if (sub2_left_near_lane_dist.first > 2 && sub2_left_near_lane_dist.first < 5) {
          temp_lane_1->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
          temp_lane_2->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
          adjoin_lane_vaild              = true;
        }
      }
    }

    if (line_type == "lanemarker") {
      if (sub1_left_near_lanemarker_dist.first < sub2_left_near_lanemarker_dist.first) {
        if (sub1_left_near_lanemarker_dist.first > 0.0 && sub1_left_near_lanemarker_dist.first < 2.5) {
          temp_lane_1->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
          temp_lane_2->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
          adjoin_lane_vaild              = true;
        }
      } else {
        if (sub2_left_near_lanemarker_dist.first > 0.0 && sub2_left_near_lanemarker_dist.first < 2.5) {
          temp_lane_1->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
          temp_lane_2->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
          adjoin_lane_vaild              = true;
        }
      }
    }
  }

  if (type == merge_mode) {
    if (line_type == "lane") {
      if (sub1_left_near_lane_dist.first < sub2_left_near_lane_dist.first) {
        if (sub1_left_near_lane_dist.first > 2 && sub1_left_near_lane_dist.first < 5) {
          temp_lane_1->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
          temp_lane_2->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
          adjoin_lane_vaild              = true;
        }
      } else {
        if (sub2_left_near_lane_dist.first > 2 && sub2_left_near_lane_dist.first < 5) {
          temp_lane_1->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
          temp_lane_2->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
          adjoin_lane_vaild              = true;
        }
      }
    }

    if (line_type == "lanemarker") {
      if (sub1_left_near_lanemarker_dist.first < sub2_left_near_lanemarker_dist.first) {
        if (sub1_left_near_lanemarker_dist.first > 0.0 && sub1_left_near_lanemarker_dist.first < 2.5) {
          temp_lane_1->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
          temp_lane_2->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
          adjoin_lane_vaild              = true;
        }
      } else {
        if (sub2_left_near_lanemarker_dist.first > 0.0 && sub2_left_near_lanemarker_dist.first < 2.5) {
          temp_lane_1->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
          temp_lane_2->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
          adjoin_lane_vaild              = true;
        }
      }
    }
  }
  return adjoin_lane_vaild;
}

bool LaneTypeChecker::CheckTopoUseSlope(const uint64_t main_lane_id, const uint64_t sub_lane_1_id, const uint64_t sub_lane_2_id,
                                        const int type) {
  bool slop_valid    = false;
  auto main_lan_iter = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                    [&](const cem::message::sensor::BevLaneInfo &lane) { return main_lane_id == lane.id; });

  auto sub_lane_1_iter = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [&](const cem::message::sensor::BevLaneInfo &lane) { return sub_lane_1_id == lane.id; });

  auto sub_lane_2_iter = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [&](const cem::message::sensor::BevLaneInfo &lane) { return sub_lane_2_id == lane.id; });

  if (main_lan_iter == data_->lane_infos.end() || sub_lane_1_iter == data_->lane_infos.end() ||
      sub_lane_2_iter == data_->lane_infos.end()) {
    return false;
  }

  auto main_lane_pts  = *(main_lan_iter->geos);
  auto sub_lane_1_pts = *(sub_lane_1_iter->geos);
  auto sub_lane_2_pts = *(sub_lane_2_iter->geos);

  std::sort(main_lane_pts.begin(), main_lane_pts.end(), [](Eigen::Vector2f &pt1, Eigen::Vector2f &pt2) { return pt1.x() < pt2.x(); });

  std::sort(sub_lane_1_pts.begin(), sub_lane_1_pts.end(), [](Eigen::Vector2f &pt1, Eigen::Vector2f &pt2) { return pt1.x() < pt2.x(); });

  std::sort(sub_lane_2_pts.begin(), sub_lane_2_pts.end(), [](Eigen::Vector2f &pt1, Eigen::Vector2f &pt2) { return pt1.x() < pt2.x(); });

  std::pair<float, float> angle1;
  std::pair<float, float> angle2;
  bool                    angle_flag1 = GetVetorialAngle(main_lane_pts, sub_lane_1_pts, type, angle1);
  bool                    angle_flag2 = GetVetorialAngle(main_lane_pts, sub_lane_2_pts, type, angle2);

  if (!angle_flag1 || !angle_flag2) {
    return false;
  }

  if (CHECK_TOPO_DEBUG_MODE) {
    AINFO << " slope: " << main_lane_id << "," << sub_lane_1_id << " angle1: " << angle1.first << " m_angle1: " << angle1.second;
    AINFO << " slope: " << main_lane_id << "," << sub_lane_2_id << " angle2: " << angle2.first << " m_angle2: " << angle2.second;
  }

  bool  sub_1_is_left = LaneGeometry::JudgeIsLeft(sub_lane_1_pts, sub_lane_2_pts);
  float delta_angle   = fabs(angle1.first - angle2.first);
  if (type == split_mode) {
    if (delta_angle > 0.2) {
      if (angle1.first > angle2.first) {
        sub_lane_1_iter->split_topo_extend =
            sub_1_is_left ? SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT : SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
        sub_lane_2_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
      } else {
        sub_lane_1_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
        sub_lane_2_iter->split_topo_extend =
            sub_1_is_left ? SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT : SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
      }
    } else {
      if (angle1.second > angle2.second) {
        sub_lane_1_iter->split_topo_extend =
            sub_1_is_left ? SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT : SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
        sub_lane_2_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
      } else {
        sub_lane_1_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
        sub_lane_2_iter->split_topo_extend =
            sub_1_is_left ? SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT : SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
      }
    }
    slop_valid = true;
  }

  if (type == merge_mode) {
    if (delta_angle > 0.2) {
      if (angle1.first > angle2.first) {
        sub_lane_1_iter->merge_topo_extend =
            sub_1_is_left ? MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT : MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
        sub_lane_2_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
      } else {
        sub_lane_1_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
        sub_lane_2_iter->merge_topo_extend =
            sub_1_is_left ? MergeTopoExtendType::TOPOLOGY_MERGE_LEFT : MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT;
      }
    } else {
      if (angle1.second > angle2.second) {
        sub_lane_1_iter->merge_topo_extend =
            sub_1_is_left ? MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT : MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
        sub_lane_2_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
      } else {
        sub_lane_1_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
        sub_lane_2_iter->merge_topo_extend =
            sub_1_is_left ? MergeTopoExtendType::TOPOLOGY_MERGE_LEFT : MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT;
      }
    }
    slop_valid = true;
  }
  return slop_valid;
}

bool LaneTypeChecker::CheckHorizontalDisMatchInfo(std::vector<const BevSplitInfo *> &split_match_infos,
                                                  std::vector<const BevMergeInfo *> &merge_match_infos, const int type) {
  bool matcher_valid = true;
  if (type == split_mode) {
    std::sort(split_match_infos.begin(), split_match_infos.end(),
              [](const BevSplitInfo *split_a, const BevSplitInfo *split_b) { return split_a->distance_to_ego < split_b->distance_to_ego; });

    if (split_match_infos.empty()) {
      return false;
    }

    for (auto &s_match_info : split_match_infos) {
      bool s_match_flag = true;
      if (s_match_info->map_lanes_from.empty() || s_match_info->bev_target_to.empty()) {
        s_match_flag = false;
      } else {
        if (s_match_info->bev_lane_from != 0 && !s_match_info->bev_target_to.empty()) {
          for (const auto &lane_info : s_match_info->bev_target_to) {
            // SPLIT子路匹配车道距离校验
            bool                         sub_lane_match_flag = false;
            std::vector<Eigen::Vector2f> m_sub_lane_pts;
            std::vector<Eigen::Vector2f> b_sub_lane_pts;
            std::pair<float, float>      sub_lane_dist = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
            GetMapLaneGeos(lane_info.second.map_lanes_to, m_sub_lane_pts);
            GetBevLaneGeos(lane_info.first, b_sub_lane_pts);
            bool sub_lane_flag = LaneGeometry::CalculateAverageDistanceBetweenTwoLines(b_sub_lane_pts, m_sub_lane_pts, sub_lane_dist);

            if (sub_lane_flag && sub_lane_dist.first >= 0 && sub_lane_dist.first <= 1.5) {
              sub_lane_match_flag = true;
            } else {
              s_match_flag = false;
              break;
            }
          }
        }
      }
      matcher_valid = matcher_valid && s_match_flag;
    }
  }

  if (type == merge_mode) {
    std::sort(merge_match_infos.begin(), merge_match_infos.end(),
              [](const BevMergeInfo *merge_a, const BevMergeInfo *merge_b) { return merge_a->distance_to_ego < merge_b->distance_to_ego; });
    if (merge_match_infos.empty()) {
      return false;
    }

    for (auto &m_match_info : merge_match_infos) {
      bool m_match_flag = true;
      if (m_match_info->map_lanes_to.empty() || m_match_info->bev_lanes_from.empty()) {
        m_match_flag = false;
      } else {
        if (m_match_info->bev_lane_to != 0 && !m_match_info->bev_lanes_from.empty()) {
          for (const auto &lane_info : m_match_info->bev_lanes_from) {
            // MERGE子路匹配车道距离校验
            bool                         sub_lane_match_flag = false;
            std::vector<Eigen::Vector2f> m_sub_lane_pts;
            std::vector<Eigen::Vector2f> b_sub_lane_pts;
            std::pair<float, float>      sub_lane_dist = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
            GetMapLaneGeos(lane_info.second.map_lanes_from, m_sub_lane_pts);
            GetBevLaneGeos(lane_info.first, b_sub_lane_pts);

            bool sub_lane_flag = LaneGeometry::CalculateAverageDistanceBetweenTwoLines(b_sub_lane_pts, m_sub_lane_pts, sub_lane_dist);
            if (sub_lane_flag && sub_lane_dist.first >= 0 && sub_lane_dist.first <= 1.5) {
              sub_lane_match_flag = true;
            } else {
              m_match_flag = false;
              break;
            }
          }
        }
      }
      matcher_valid = matcher_valid && m_match_flag;
    }
  }
  return matcher_valid;
}

bool LaneTypeChecker::CheckPositionMatchInfo(std::vector<const BevSplitInfo *> &split_match_infos,
                                             std::vector<const BevMergeInfo *> &merge_match_infos, const int type) {
  bool matcher_valid = true;
  if (type == split_mode) {
    std::sort(split_match_infos.begin(), split_match_infos.end(),
              [](const BevSplitInfo *split_a, const BevSplitInfo *split_b) { return split_a->distance_to_ego < split_b->distance_to_ego; });

    if (split_match_infos.empty()) {
      return false;
    }

    for (auto &s_match_info : split_match_infos) {
      bool s_match_flag = false;
      if (s_match_info->map_lanes_from.empty() || s_match_info->bev_target_to.empty()) {
        s_match_flag = false;
      } else {

        // AINFO << "[pos] bev_lane_from size: " << s_match_info->bev_lane_from;
        // AINFO << "[pos] bev_target_to size: " << s_match_info->bev_target_to.size();
        if (s_match_info->bev_lane_from != 0) {
          if (s_match_info->bev_target_to.size() == 2) {
            // SPLIT感知子路匹配车道左右位置校验
            bool                         b_sub_lane_left_flag = false;
            std::vector<Eigen::Vector2f> b_sub_lane_1_pts;
            std::vector<Eigen::Vector2f> b_sub_lane_2_pts;
            GetBevLaneGeos(s_match_info->bev_target_to[0].second.bev_lane_to, b_sub_lane_1_pts);
            GetBevLaneGeos(s_match_info->bev_target_to[1].second.bev_lane_to, b_sub_lane_2_pts);
            b_sub_lane_left_flag = LaneGeometry::JudgeIsLeft(b_sub_lane_1_pts, b_sub_lane_2_pts);

            // SPLIT地图子路匹配车道左右位置校验
            bool                         m_sub_lane_left_flag = false;
            std::vector<Eigen::Vector2f> m_sub_lane_1_pts;
            std::vector<Eigen::Vector2f> m_sub_lane_2_pts;
            GetMapLaneGeos(s_match_info->bev_target_to[0].second.map_lanes_to, m_sub_lane_1_pts);
            GetMapLaneGeos(s_match_info->bev_target_to[1].second.map_lanes_to, m_sub_lane_2_pts);
            m_sub_lane_left_flag = LaneGeometry::JudgeIsLeft(m_sub_lane_1_pts, m_sub_lane_2_pts);

            if (b_sub_lane_left_flag == m_sub_lane_left_flag) {
              s_match_flag = true;
            }
          }
          if (s_match_info->bev_target_to.size() == 1) {
            s_match_flag = true;
          }
        }
      }
      matcher_valid = matcher_valid && s_match_flag;
    }
  }

  if (type == merge_mode) {
    std::sort(merge_match_infos.begin(), merge_match_infos.end(),
              [](const BevMergeInfo *merge_a, const BevMergeInfo *merge_b) { return merge_a->distance_to_ego < merge_b->distance_to_ego; });

    if (merge_match_infos.empty()) {
      return false;
    }

    for (auto &m_match_info : merge_match_infos) {
      bool m_match_flag = false;
      if (m_match_info->map_lanes_to.empty() || m_match_info->bev_lanes_from.empty()) {
        m_match_flag = false;
      } else {
        if (m_match_info->bev_lane_to != 0) {
          if (m_match_info->bev_lanes_from.size() == 2) {
            std::vector<std::pair<uint64_t, BevMergeSource>> m_bev_lanes_from;
            for (const auto &lane_info : m_match_info->bev_lanes_from) {
              m_bev_lanes_from.emplace_back(lane_info);
            }

            // MERGE感知子路匹配车道左右位置校验
            bool                         b_sub_lane_left_flag = false;
            std::vector<Eigen::Vector2f> b_sub_lane_1_pts;
            std::vector<Eigen::Vector2f> b_sub_lane_2_pts;
            GetBevLaneGeos(m_bev_lanes_from[0].first, b_sub_lane_1_pts);
            GetBevLaneGeos(m_bev_lanes_from[1].first, b_sub_lane_2_pts);
            b_sub_lane_left_flag = LaneGeometry::JudgeIsLeft(b_sub_lane_1_pts, b_sub_lane_2_pts);

            // MERGE地图子路匹配车道左右位置校验
            bool                         m_sub_lane_left_flag = false;
            std::vector<Eigen::Vector2f> m_sub_lane_1_pts;
            std::vector<Eigen::Vector2f> m_sub_lane_2_pts;
            GetMapLaneGeos(m_bev_lanes_from[0].second.map_lanes_from, m_sub_lane_1_pts);
            GetMapLaneGeos(m_bev_lanes_from[1].second.map_lanes_from, m_sub_lane_2_pts);
            m_sub_lane_left_flag = LaneGeometry::JudgeIsLeft(m_sub_lane_1_pts, m_sub_lane_2_pts);

            if (b_sub_lane_left_flag == m_sub_lane_left_flag) {
              m_match_flag = true;
            }
          }
          if (m_match_info->bev_lanes_from.size() == 1) {
            m_match_flag = true;
          }
        }
      }
      matcher_valid = matcher_valid && m_match_flag;
    }
  }
  return matcher_valid;
}

bool LaneTypeChecker::CheckMatchInfo(std::vector<const BevSplitInfo *> &split_match_infos,
                                     std::vector<const BevMergeInfo *> &merge_match_infos,
                                     const int type) {
  // 距离校验
  bool dist_valid = CheckHorizontalDisMatchInfo(split_match_infos, merge_match_infos, type);
  // 位置校验
  bool pos_valid = CheckPositionMatchInfo(split_match_infos, merge_match_infos, type);

  if (CHECK_TOPO_DEBUG_MODE) {
    AINFO << "[CheckMatchInfo] dist_valid: " << dist_valid << " pos_valid: " << pos_valid;
  }

  return dist_valid && pos_valid;
}

void LaneTypeChecker::GetBevLaneGeos(uint64_t bev_lane_id, std::vector<Eigen::Vector2f> &bev_lane_pts) {
  std::vector<Eigen::Vector2f> t_map_lane_pts;
  if (data_ == nullptr || !bev_lane_id) {
    return;
  }

  auto bev_lane_iter = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                    [&](const cem::message::sensor::BevLaneInfo &lane) { return bev_lane_id == lane.id; });
  if (bev_lane_iter != data_->lane_infos.end()) {
    bev_lane_pts = *(bev_lane_iter->geos);
  }
}

void LaneTypeChecker::GetMapLaneGeos(std::vector<uint64_t> m_group_ids, std::vector<Eigen::Vector2f> &map_lane_pts) {
  std::vector<Eigen::Vector2f> t_map_lane_pts;
  if (ld_map_ == nullptr || m_group_ids.empty()) {
    return;
  }

  for (const auto &m_lane_id : m_group_ids) {
    if (m_lane_id == 0)
      continue;
    auto m_lane_iter = std::find_if(ld_map_->lanes.begin(), ld_map_->lanes.end(),
                                    [&](const cem::message::env_model::LaneInfo &lane) { return m_lane_id == lane.id; });

    if (m_lane_iter == ld_map_->lanes.end())
      continue;

    for (const auto &point : m_lane_iter->points) {
      Eigen::Vector2f m_point(point.x, point.y);
      t_map_lane_pts.push_back(m_point);
    }
  }

  std::sort(t_map_lane_pts.begin(), t_map_lane_pts.end(), [](Eigen::Vector2f &pt1, Eigen::Vector2f &pt2) { return pt1.x() < pt2.x(); });

  map_lane_pts = t_map_lane_pts;
}

bool LaneTypeChecker::BevTopoIsValid(const uint64_t main_lane_id, const uint64_t sub_lane_1_id, const uint64_t sub_lane_2_id,
                                     const int type) {
  bool topo_valid = false;
  if (!main_lane_id || !sub_lane_1_id || !sub_lane_2_id || data_ == nullptr) {
    return false;
  }

  auto main_lane_iter =
      std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(), [&](const BevLaneInfo &p) { return p.id == main_lane_id; });

  auto sub_lane_1_iter =
      std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(), [&](const BevLaneInfo &p) { return p.id == sub_lane_1_id; });

  auto sub_lane_2_iter =
      std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(), [&](const BevLaneInfo &p) { return p.id == sub_lane_2_id; });

  if (main_lane_iter == data_->lane_infos.end() || sub_lane_1_iter == data_->lane_infos.end() ||
      sub_lane_2_iter == data_->lane_infos.end()) {
    return false;
  }

  if (type == split_mode) {
    std::set<uint64_t> next_lane_ids(main_lane_iter->next_lane_ids.begin(), main_lane_iter->next_lane_ids.end());
    if (next_lane_ids.size() != 2) {
      // TODO 子路数量为1时，不进行车道类型校验
      topo_valid = false;
    } else {
      auto prev_lane_1_iter = std::find(sub_lane_1_iter->previous_lane_ids.begin(), sub_lane_1_iter->previous_lane_ids.end(), main_lane_id);

      auto prev_lane_2_iter = std::find(sub_lane_2_iter->previous_lane_ids.begin(), sub_lane_2_iter->previous_lane_ids.end(), main_lane_id);

      if (prev_lane_1_iter != sub_lane_1_iter->previous_lane_ids.end() && prev_lane_2_iter != sub_lane_2_iter->previous_lane_ids.end() &&
          next_lane_ids.find(sub_lane_1_id) != next_lane_ids.end() && next_lane_ids.find(sub_lane_2_id) != next_lane_ids.end()) {
        topo_valid = true;
      }
    }
  }

  if (type == merge_mode) {
    std::set<uint64_t> prev_lane_ids(main_lane_iter->previous_lane_ids.begin(), main_lane_iter->previous_lane_ids.end());
    if (prev_lane_ids.size() != 2) {
      // TODO 子路数量为1时，不进行车道类型校验
      topo_valid = false;
    } else {
      auto next_lane_1_iter = std::find(sub_lane_1_iter->next_lane_ids.begin(), sub_lane_1_iter->next_lane_ids.end(), main_lane_id);

      auto next_lane_2_iter = std::find(sub_lane_2_iter->next_lane_ids.begin(), sub_lane_2_iter->next_lane_ids.end(), main_lane_id);

      if (next_lane_1_iter != sub_lane_1_iter->next_lane_ids.end() && next_lane_2_iter != sub_lane_2_iter->next_lane_ids.end() &&
          prev_lane_ids.find(sub_lane_1_id) != prev_lane_ids.end() && prev_lane_ids.find(sub_lane_2_id) != prev_lane_ids.end()) {
        topo_valid = true;
      }
    }
  }
  return topo_valid;
}

bool LaneTypeChecker::GetVetorialAngle(std::vector<Eigen::Vector2f> &main_lane, std::vector<Eigen::Vector2f> &sub_lane, const int type,
                                       std::pair<float, float> &angle) {
  if (main_lane.size() < 2 || sub_lane.size() < 2) {
    return false;
  }

  float control_point_first_length  = 5.0f;
  float control_point_second_length = 5.0f;

  // for(const auto& pt : main_lane)
  // {
  //     AINFO << "[main_lane]: " << pt.x() << "," << pt.y();
  // }
  //
  // for(const auto& pt : sub_lane)
  // {
  //     AINFO << "[sub_lane]: " << pt.x() << "," << pt.y();
  // }

  Eigen::Vector2f p0, p1, p2, p3;
  Eigen::Vector2f delta, delta2, delta3, delta4;
  if (type == split_mode) {
    p0 = main_lane.front();
    p1 = main_lane.back();

    for (int i = (int)(main_lane.size() - 2); i >= 0; i--) {
      if (fabs(p1.x() - main_lane[i].x()) > control_point_first_length && fabs(p1.x() - main_lane[i].x()) > 1e-5) {
        p0 = main_lane[i];
        break;
      } else if (fabs(p1.x() - main_lane[i].x()) > 1e-5) {
        p0 = main_lane[i];
      }
    }

    p2 = sub_lane.front();
    p3 = sub_lane.back();

    for (int i = 0; i < sub_lane.size(); i++) {
      if (fabs(sub_lane[i].x() - p2.x()) > control_point_second_length) {
        p3 = sub_lane[i];
        break;
      } else if (fabs(sub_lane[i].x() - p2.x()) > 1e-5) {
        p3 = sub_lane[i];
      }
    }
    delta(0)  = p1.x() - p0.x();
    delta(1)  = p1.y() - p0.y();
    delta2(0) = p3.x() - p2.x();
    delta2(1) = p3.y() - p2.y();
    delta3(0) = p2.x() - p1.x();
    delta3(1) = p2.y() - p1.y();
    delta4(0) = p3.x() - p1.x();
    delta4(1) = p3.y() - p1.y();
  }

  if (type == merge_mode) {
    p0 = main_lane.back();
    p1 = main_lane.front();
    for (int i = 0; i < main_lane.size(); i++) {
      if (fabs(main_lane[i].x() - p1.x()) > control_point_second_length) {
        p0 = main_lane[i];
        break;
      } else if (fabs(main_lane[i].x() - p1.x()) > 1e-5) {
        p0 = main_lane[i];
      }
    }

    p2 = sub_lane.back();
    p3 = sub_lane.front();

    for (int i = (int)(sub_lane.size() - 2); i >= 0; i--) {
      if (fabs(p2.x() - sub_lane[i].x()) > 15.0 && fabs(p2.x() - sub_lane[i].x()) > 1e-5) {
        p3 = sub_lane[i];
        break;
      } else if (fabs(p3.x() - sub_lane[i].x()) > 1e-5) {
        p3 = sub_lane[i];
      }
    }
    delta(0)  = p1.x() - p0.x();
    delta(1)  = p1.y() - p0.y();
    delta2(0) = p3.x() - p2.x();
    delta2(1) = p3.y() - p2.y();
    delta3(0) = p2.x() - p1.x();
    delta3(1) = p2.y() - p1.y();
    delta4(0) = p3.x() - p1.x();
    delta4(1) = p3.y() - p1.y();
  }

  if (CHECK_TOPO_DEBUG_MODE) {
    AINFO << "p0: " << p0.x() << "," << p0.y();
    AINFO << "p1: " << p1.x() << "," << p1.y();
    AINFO << "p3: " << p3.x() << "," << p3.y();
  }
  if (delta.norm() < 1e-5 || delta2.norm() < 1e-5 || delta3.norm() < 1e-5 || delta4.norm() < 1e-5) {
    return false;
  }

  delta  = delta / delta.norm();
  delta2 = delta2 / delta2.norm();
  delta3 = delta3 / delta3.norm();
  delta4 = delta4 / delta4.norm();

  double cosAngle = acos(delta.dot(delta4) / (delta.norm() * delta4.norm())) * (180.0 / M_PI);
  // double cosAngle = acos(delta.dot(delta2)/(delta.norm()*delta2.norm()))*(180.0 / M_PI);
  double cosAngle2 = acos(delta.dot(delta3) / (delta.norm() * delta3.norm())) * (180.0 / M_PI);

  float delta_angle = cosAngle;
  // float delta_angle = fabs(cosAngle - cosAngle2);
  float mean_angle = (cosAngle2 + cosAngle) / 2;

  angle = {delta_angle, mean_angle};
  return true;
}

bool LaneTypeChecker::buildTopoTypeUpdate(const uint64_t main_lane_id, 
                                          const uint64_t sub_lane_1_id, 
                                          const uint64_t sub_lane_2_id,
                                          const int type) {
  bool build_topo_type_valid = false;
  if (data_ == nullptr) {
    return false;
  }

  auto main_lane_iter =
      std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(), [&](const BevLaneInfo &p) { return p.id == main_lane_id; });

  auto sub_lane_1_iter =
      std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(), [&](const BevLaneInfo &p) { return p.id == sub_lane_1_id; });

  auto sub_lane_2_iter =
      std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(), [&](const BevLaneInfo &p) { return p.id == sub_lane_2_id; });

  if (main_lane_iter == data_->lane_infos.end() || sub_lane_1_iter == data_->lane_infos.end() ||
      sub_lane_2_iter == data_->lane_infos.end()) {
    return false;
  }

  if (type == split_mode) {
    std::set<uint64_t> next_lane_ids(main_lane_iter->next_lane_ids.begin(), main_lane_iter->next_lane_ids.end());
    auto prev_lane_1_iter = std::find(sub_lane_1_iter->previous_lane_ids.begin(), sub_lane_1_iter->previous_lane_ids.end(), main_lane_id);

    auto prev_lane_2_iter = std::find(sub_lane_2_iter->previous_lane_ids.begin(), sub_lane_2_iter->previous_lane_ids.end(), main_lane_id);

    if (prev_lane_1_iter != sub_lane_1_iter->previous_lane_ids.end() && prev_lane_2_iter != sub_lane_2_iter->previous_lane_ids.end() &&
        next_lane_ids.find(sub_lane_1_id) != next_lane_ids.end() && next_lane_ids.find(sub_lane_2_id) != next_lane_ids.end()) {
      if (sub_lane_1_iter->is_build_split) {
        if (sub_lane_1_iter->connect_type == BevLaneConnectType::NORMAL) {
          sub_lane_1_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
        }

        if (sub_lane_1_iter->connect_type == BevLaneConnectType::SPLIT) {
          if (sub_lane_1_iter->is_build_split_left) {
            sub_lane_1_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
          } else {
            sub_lane_1_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
          }
        }
      }

      if (sub_lane_2_iter->is_build_split) {
        if (sub_lane_2_iter->connect_type == BevLaneConnectType::NORMAL) {
          sub_lane_2_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
        }

        if (sub_lane_2_iter->connect_type == BevLaneConnectType::SPLIT) {
          if (sub_lane_2_iter->is_build_split_left) {
            sub_lane_2_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
          } else {
            sub_lane_2_iter->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
          }
        }
      }
      build_topo_type_valid = true;
    }
    if (CHECK_TOPO_DEBUG_MODE) {
      AINFO << "[buildTopoTypeUpdate]: \n"
            << "split_mode: " << split_mode << " \n"
            << "sub 1 id: " << sub_lane_1_iter->id << " \n"
            << "  is_build_split_left: " << sub_lane_1_iter->is_build_split_left << " \n"
            << "  split_lane_type: " << split_type[sub_lane_1_iter->split_topo_extend] << " \n"
            << "sub 2 id: " << sub_lane_2_iter->id << " \n"
            << "  is_build_split_left: " << sub_lane_2_iter->is_build_split_left << " \n"
            << "  split_lane_type: " << split_type[sub_lane_2_iter->split_topo_extend];
    }
  }

  if (type == merge_mode) {
    std::set<uint64_t> prev_lane_ids(main_lane_iter->previous_lane_ids.begin(), main_lane_iter->previous_lane_ids.end());
    auto next_lane_1_iter = std::find(sub_lane_1_iter->next_lane_ids.begin(), sub_lane_1_iter->next_lane_ids.end(), main_lane_id);

    auto next_lane_2_iter = std::find(sub_lane_2_iter->next_lane_ids.begin(), sub_lane_2_iter->next_lane_ids.end(), main_lane_id);

    if (next_lane_1_iter != sub_lane_1_iter->next_lane_ids.end() && next_lane_2_iter != sub_lane_2_iter->next_lane_ids.end() &&
        prev_lane_ids.find(sub_lane_1_id) != prev_lane_ids.end() && prev_lane_ids.find(sub_lane_2_id) != prev_lane_ids.end()) {
      if(sub_lane_1_iter->is_build_merge)
      {
        if(sub_lane_1_iter->connect_type == BevLaneConnectType::NORMAL)
        {
          sub_lane_1_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
        }

        if(sub_lane_1_iter->connect_type == BevLaneConnectType::MERGE)
        {
          if(sub_lane_1_iter->is_build_merge_left)
          {
            sub_lane_1_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
          }else{
            sub_lane_1_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT;
          }
        }
      }

      if(sub_lane_2_iter->is_build_merge)
      {
        if(sub_lane_2_iter->connect_type == BevLaneConnectType::NORMAL)
        {
          sub_lane_2_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
        }
        if(sub_lane_2_iter->connect_type == BevLaneConnectType::MERGE)
        {
          if(sub_lane_2_iter->is_build_merge_left)
          {
            sub_lane_2_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
          }else{
            sub_lane_2_iter->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT;
          }
        }
      }
      build_topo_type_valid = true;
    }
    if(CHECK_TOPO_DEBUG_MODE)
    {
      AINFO << "[buildTopoTypeUpdate]:\n"
            << "merge_mode: " << merge_mode << " \n"
            << "sub 1 id: " << sub_lane_1_iter->id << " \n"
            << "  is_build_merge_left: " << sub_lane_1_iter->is_build_merge_left << " \n"
            << "  merge_lane_type: " << merge_type[sub_lane_1_iter->merge_topo_extend] << " \n"
            << "sub 2 id: " << sub_lane_2_iter->id << " \n"
            << "  is_build_merge_left: " << sub_lane_2_iter->is_build_merge_left << " \n"
            << "  merge_lane_type: " << merge_type[sub_lane_2_iter->merge_topo_extend];
    }
  }
  return build_topo_type_valid;
}

void LaneTypeChecker::UpdateBevTopologyLaneType(std::shared_ptr<cem::message::sensor::BevMapInfo>& bev_info,
                                                const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map,
                                                const std::function<std::vector<const BevSplitInfo*>(uint64_t)>& LaneSplitGetter,
                                                const std::function<std::vector<const BevMergeInfo*>(uint64_t, bool)>& LaneMergeGetter)
{
  if(bev_info == nullptr)
  {
    return;
  }

  if(LaneSplitGetter && LaneMergeGetter)
  {
    ld_map_ = ld_map;
    SplitGetter = LaneSplitGetter;
    MergeGetter = LaneMergeGetter;
  }
  data_ = bev_info;

  if(CHECK_TOPO_DEBUG_MODE)
  {
    AINFO << "sequence number:" << data_->header.cycle_counter;
    for (const auto &lane : data_->lane_infos) {
      AINFO << "id: " << lane.id;
      AINFO << "lane_type: "    << static_cast<int>(lane.lane_type);
      AINFO << "split_type: "   << split_type[lane.split_topo_extend];
      AINFO << "merge_type: "   << merge_type[lane.merge_topo_extend];
      AINFO << "prev_ids_size: "<< lane.previous_lane_ids.size();
      AINFO << "next_ids_size: "<< lane.next_lane_ids.size();
      AINFO << "build topo:   " << lane.is_build_split << "," << lane.is_build_merge << "," << lane.is_build_split_marker_by_map << ","
            << lane.is_build_merge_marker_by_map;

      std::string prev_ids = "";
      for (const auto prev_lane_id : lane.previous_lane_ids) {
        prev_ids += std::to_string(prev_lane_id) + ",";
      }

      std::string next_ids = "";
      for (const auto next_lane_id : lane.next_lane_ids) {
        next_ids += std::to_string(next_lane_id) + ",";
      }
      AINFO << "prev_ids: " << prev_ids;
      AINFO << "next_ids: " << next_ids;
    }
  }

  // SPLIT车道属性校验
  CheckSplitLaneType();

  // MERGE车道属性校验
  CheckMergeLaneType();

  // 特殊拓扑语义信息校验
  CheckTopoUseSemanticInfo();
}

} // namespace fusion
} // namespace cem
