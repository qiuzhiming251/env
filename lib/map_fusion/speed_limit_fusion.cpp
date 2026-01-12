#include "speed_limit_fusion.h"

namespace cem {
namespace fusion {
  

void CnoaSpeedLimit::Process(DetectBevMap &bev_map, const RoutingMapPtr routing_map_ptr) {
  if (!bev_map.bev_map_ptr || !routing_map_ptr) {
    return;
  }
  bev_map.bev_map_ptr->speed_info.condition_limits.clear();
  routing_map_ptr->speed_info.condition_limits.clear();
  json debug_json;

  // 存储所有lanes以及导航路径sections，便于访问
  std::unordered_map<uint64_t, const LaneInfo*> all_lanes;
  for (const auto& lane : routing_map_ptr->lanes) {
    all_lanes.insert({lane.id, &lane});
  }
  std::unordered_map<uint64_t, const SectionInfo*> route_sections;
  for (const auto& section : routing_map_ptr->route.sections) {
    route_sections.insert({section.id, &section});
  }

  // 根据当前道路可行驶车道数，计算当前道路限速值
  ConditionLimit cur_limit;
  cur_limit.condition_distance = 0;
  uint64_t cur_section_id = routing_map_ptr->route.navi_start.section_id;
  double cur_s_offset = routing_map_ptr->route.navi_start.s_offset;
  debug_json["cur_section_id"] = cur_section_id;
  for (const auto &section : routing_map_ptr->route.sections) {
    if (section.id == cur_section_id) {
      bool is_keep_speed = false;
      int cur_lane_num = 0;
      for (const auto& lane_id : section.lane_ids) {
        const LaneInfo* lane_ptr = nullptr;
        if (all_lanes.count(lane_id)) {
          lane_ptr = all_lanes.at(lane_id);
        }
        if (lane_ptr == nullptr) {
          continue;
        }
        if (lane_ptr->turn_type == TurnType::RIGHT_TURN || lane_ptr->junction_id > 0) {
          is_keep_speed = true;
        }
        // 需求1：统计车道类型不为(非机动车道 && 导流区车道 && 港湾停靠站 && 障碍物车道)的车道数
        if (lane_ptr->type != LaneType::LANE_NON_MOTOR && lane_ptr->type != LaneType::LANE_HARBOR_STOP &&
            lane_ptr->type != LaneType::LANE_DIVERSION && lane_ptr->type != LaneType::LANE_OBSTACLE) {
          cur_lane_num++;
        }
      }
      if (cur_lane_num <= 0) cur_lane_num = section.lane_num;

      // 需求2：如果包含右转专用道，那么统计上一道路的车道数
      const SectionInfo* section_ptr = &section;
      int pre_lane_num = 0;
      for (int i=0; is_keep_speed && i<10; i++) {
        if (section_ptr == nullptr) {
          break;
        }
        const SectionInfo* pre_section_ptr = nullptr;
        for (const auto& pre_section_id : section_ptr->predecessor_section_id_list) {
          if (route_sections.count(pre_section_id)) {
            pre_section_ptr = route_sections.at(pre_section_id);
            break;
          }
        }
        if (pre_section_ptr == nullptr) {
          break;
        }
        is_keep_speed = false;
        pre_lane_num = 0;
        for (const auto& pre_lane_id : pre_section_ptr->lane_ids) {
          const LaneInfo* pre_lane_ptr = nullptr;
          if (all_lanes.count(pre_lane_id)) {
            pre_lane_ptr = all_lanes.at(pre_lane_id);
          }
          if (pre_lane_ptr == nullptr) {
            continue;
          }
          if (pre_lane_ptr->turn_type == TurnType::RIGHT_TURN || pre_lane_ptr->junction_id > 0) {
            is_keep_speed = true;
          }
          if (pre_lane_ptr->type != LaneType::LANE_NON_MOTOR && pre_lane_ptr->type != LaneType::LANE_HARBOR_STOP &&
              pre_lane_ptr->type != LaneType::LANE_DIVERSION && pre_lane_ptr->type != LaneType::LANE_OBSTACLE) {
            pre_lane_num++;
          }
        }
        section_ptr = pre_section_ptr;
      }
      if (pre_lane_num > 0) cur_lane_num = pre_lane_num;
      
      // 需求3：如果可行驶车道数=1/2/3，那么道路限速=40/50/60kph
      int cur_speed_value = cur_lane_num <= 1 ? 40 : (cur_lane_num == 2 ? 50 : 60);
      cur_limit.condition_limit_speed_value = static_cast<float>(cur_speed_value);
      bev_map.bev_map_ptr->speed_info.condition_limits.emplace_back(cur_limit);
      routing_map_ptr->speed_info.condition_limits.emplace_back(cur_limit);
      debug_json["cur_lane_num"] = cur_lane_num;

      // 根据下一道路可行驶车道数，计算下一道路限速值
      ConditionLimit next_limit;
      next_limit.condition_distance = section.length - cur_s_offset;
      section_ptr = &section;
      for (int i=0; next_limit.condition_distance < kMaxOffset && i<30; i++) {
        if (section_ptr == nullptr) {
          break;
        }
        const SectionInfo* next_section_ptr = nullptr;
        for (const auto& next_section_id : section_ptr->successor_section_id_list) {
          if (route_sections.count(next_section_id)) {
            next_section_ptr = route_sections.at(next_section_id);
            break;
          }
        }
        if (next_section_ptr == nullptr) {
          break;
        }
        is_keep_speed = false;
        int next_lane_num = 0;
        for (const auto& next_lane_id : next_section_ptr->lane_ids) {
          const LaneInfo* next_lane_ptr = nullptr;
          if (all_lanes.count(next_lane_id)) {
            next_lane_ptr = all_lanes.at(next_lane_id);
          }
          if (next_lane_ptr == nullptr) {
            continue;
          }
          if (next_lane_ptr->turn_type == TurnType::RIGHT_TURN || next_lane_ptr->junction_id > 0) {
            is_keep_speed = true;
          }
          if (next_lane_ptr->type != LaneType::LANE_NON_MOTOR && next_lane_ptr->type != LaneType::LANE_HARBOR_STOP &&
              next_lane_ptr->type != LaneType::LANE_DIVERSION && next_lane_ptr->type != LaneType::LANE_OBSTACLE) {
            next_lane_num++;
          }
        }
        int next_speed_value = next_lane_num <= 1 ? 40 : (next_lane_num == 2 ? 50 : 60);
        // 需求4：如果包含右转专用道或者限速值与上一道路相同，那么更新限速距离，并搜索下一道路
        if (is_keep_speed || next_speed_value == cur_speed_value) {
          next_limit.condition_distance += next_section_ptr->length;
          section_ptr = next_section_ptr;
        }
        // 需求5：如果限速值不相同，那么更新为下一道路限速值，并结束搜索
        else {
          next_limit.condition_limit_speed_value = static_cast<float>(next_speed_value);
          bev_map.bev_map_ptr->speed_info.condition_limits.emplace_back(next_limit);
          routing_map_ptr->speed_info.condition_limits.emplace_back(next_limit);
          debug_json["next_section_id"] = next_section_ptr->id;
          debug_json["next_lane_num"] = next_lane_num;
          break;
        }
      }
      break;
    }
  }
  std::string debug_str = debug_json.dump();
  bev_map.bev_map_ptr->debug_infos += debug_str;
}


// 基于交通标志识别的限速融合
void SpeedLimitFusion::Process(DetectBevMap &bev_map, const RoutingMapPtr routing_map_ptr) {
  // 接收地图、视觉、导航的限速输入
  if (bev_map.bev_map_ptr == nullptr) {
    return;
  }
  cur_measurement_timestamp = bev_map.bev_map_ptr->header.timestamp;

  MapEventPtr map_event_raw{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(map_event_raw);
  EHR_speedlimit = 0;
  if (map_event_raw != nullptr && !map_event_raw->speed_limit_info.empty()) {
    EHR_speedlimit = map_event_raw->speed_limit_info.front().speed_limit; //LD地图输入限速(C平台)
  }
  
  NaviTrafficInfoPtr navi_traf_raw{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(navi_traf_raw);
  Navi_speedlimit = 0;
  road_class = no_information;
  if (navi_traf_raw != nullptr) {
    Navi_speedlimit = navi_traf_raw->spd_lmt_speed_value;
    road_class = static_cast<cem::fusion::RoadClass>(navi_traf_raw->road_class);
  }

  VisionTrfInfoPtr vision_traf_raw{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(vision_traf_raw);
  vision_speedlimit_id = 0;
  vision_speedlimit_value = 0;
  vision_speedlimit_type = sp_unknown;
  vision_speedlimit_position = {1000,0,0};
  if (vision_traf_raw != nullptr) {
    for (const auto &vision_traf_obj : vision_traf_raw->objects) {
      if (vision_traf_obj.type != SPEED_RESTRICTION_BOARD) {
        continue;  //非限速牌过滤
      }
      if (vision_traf_obj.attributes.speed_restriction_type != SP_MAX_SPEED &&
          vision_traf_obj.attributes.speed_restriction_type != SP_CANCEL_RESTRICTION) {
        continue;  //非最高限速牌和取消限速牌过滤
      }
      Eigen::Vector3d vision_traf_obj_position = {vision_traf_obj.position.x,
                                                  vision_traf_obj.position.y,
                                                  vision_traf_obj.position.z};
      if (road_class == freeway || road_class == city_freeway || Navi_speedlimit >= 80) {
        if (vision_traf_obj_position.y() <= -8.0) {
          continue;  //自车位于主路时过滤匝道限速牌
        }
      }

      // 如果有取消限速牌，那么优先取消限速
      if (vision_traf_obj.attributes.speed_restriction_type == SP_CANCEL_RESTRICTION) {
        vision_speedlimit_id = vision_traf_obj.id;
        vision_speedlimit_value = 0;
        vision_speedlimit_type = sp_cancel_restriction;
        vision_speedlimit_position = vision_traf_obj_position;
        break;
      }
      // 如果有多个最高限速牌，那么取距离最近的限速牌
      if (vision_speedlimit_type == sp_unknown) {
        vision_speedlimit_id = vision_traf_obj.id;
        vision_speedlimit_value = vision_traf_obj.attributes.speed_restriction_num;
        vision_speedlimit_type = static_cast<cem::fusion::VisionSpeedLimitType>(
                                 vision_traf_obj.attributes.speed_restriction_type);
        vision_speedlimit_position = vision_traf_obj_position;
      } else {
        // 当y值较小时，x项的系数约为0，所以距离采用y比较，即龙门架限速牌取y值最小的。
        // 当y值较大时，x项的系数约为1，所以距离为欧氏距离，即路旁限速牌取距离最近的。
        double cur_x = vision_speedlimit_position.x(), cur_y = vision_speedlimit_position.y();
        double cur_distance = (cur_y * cur_y / (cur_y * cur_y + 1)) * cur_x * cur_x + cur_y * cur_y;
        double next_x = vision_traf_obj_position.x(), next_y = vision_traf_obj_position.y();
        double next_distance = (next_y * next_y / (next_y * next_y + 1)) * next_x * next_x + next_y * next_y;
        if (next_distance < cur_distance) {
          vision_speedlimit_id = vision_traf_obj.id;
          vision_speedlimit_value = vision_traf_obj.attributes.speed_restriction_num;
          vision_speedlimit_type = static_cast<cem::fusion::VisionSpeedLimitType>(
                                   vision_traf_obj.attributes.speed_restriction_type);
          vision_speedlimit_position = vision_traf_obj_position;
        }
      }
    }
  }

  DriversEventPtr drivers_event_raw{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(drivers_event_raw);
  function_support_status = 0;
  if (drivers_event_raw != nullptr) {
    function_support_status = drivers_event_raw->function_support_status;
  }

  VehicleSignalPtr veh_signal_raw{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(veh_signal_raw);

  // 限速融合模式
  if (EHR_speedlimit != 0) {
    speed_limit_mode = hdmap_mode;
  } else {
    if ((function_support_status & 0x40000000000) == 0 && Navi_speedlimit != 0) {
      speed_limit_mode = fusion_mode;
    } else if ((function_support_status & 0x40000000000) == 0) {
      speed_limit_mode = vision_mode;
    } else if (Navi_speedlimit != 0) {
      speed_limit_mode = map_mode;
    } else {
      speed_limit_mode = passive_mode;
    }
  }

  // 限速融合逻辑
  vision_filter_value = 0;
  holding_distance_update = false;
  if (EHR_speedlimit != 0) {  //地图优先
    if (EHR_speedlimit != pre_EHR_speedlimit) {
      speed_limit_value = EHR_speedlimit;
      holding_distance_update = true;
      speed_limit_status = normal;
    }
  } else {
    if (speed_limit_value == 0) {
      if (vision_speedlimit_value != 0 && vision_speedlimit_type == sp_max_speed) { //视觉其次
        if (vision_speedlimit_value == pre_vision_speedlimit && vision_speedlimit_value == prepre_vision_speedlimit) {
          vision_filter_value = vision_speedlimit_value;
          speed_limit_value = vision_speedlimit_value;
          holding_distance_update = true;
          speed_limit_status = normal;
        }
      } else if (Navi_speedlimit != 0 && Navi_speedlimit != pre_Navi_speedlimit) {  //导航最后
        speed_limit_value = Navi_speedlimit;
        holding_distance_update = true;
        speed_limit_status = normal;
      }
    } else {
      if (vision_speedlimit_type == sp_cancel_restriction) {  //视觉取消限速
        speed_limit_value = 0;
        speed_limit_status = cancelled;
      } else if (vision_speedlimit_value != 0 && vision_speedlimit_type == sp_max_speed) {
        if (vision_speedlimit_value == pre_vision_speedlimit && vision_speedlimit_value == prepre_vision_speedlimit) {
          vision_filter_value = vision_speedlimit_value;
          speed_limit_value = vision_speedlimit_value;
          holding_distance_update = true;
          speed_limit_status = normal;
        }
      } else if (Navi_speedlimit != 0 && Navi_speedlimit != pre_Navi_speedlimit) {
        if (speed_limit_value == Navi_speedlimit) {  //过滤掉导航限速跳变，只有当导航限速与当前限速一致时，更新保持阈值
          speed_limit_value = Navi_speedlimit;
          holding_distance_update = true;
          speed_limit_status = normal;
        }
      }
    }
  }

  // 更新保持阈值，或者计算累积距离
  if (holding_distance_update) {
    UpdateHoldingDistance();
  } else if (speed_limit_value != 0) {
    if (veh_signal_raw != nullptr) {
      current_vehicle_speed = veh_signal_raw->average_speed;
    } else {
      current_vehicle_speed = speed_limit_value / 3.6;
    }
    CalcAccumulatedDistance();
  }

  // 发布环境模型的限速融合输出
  bev_map.bev_map_ptr->speed_info.speed_value = static_cast<float>(speed_limit_value);
  bev_map.bev_map_ptr->speed_info.normal_speed_status = static_cast<cem::message::env_model::NormalSpeedStatus>(speed_limit_status);
  bev_map.bev_map_ptr->speed_info.speed_limit_mode = static_cast<cem::message::env_model::SpeedLimitMode>(speed_limit_mode);
  if (routing_map_ptr != nullptr) {
    routing_map_ptr->speed_info.speed_value = static_cast<float>(speed_limit_value);
    routing_map_ptr->speed_info.normal_speed_status = static_cast<cem::message::env_model::NormalSpeedStatus>(speed_limit_status);
    routing_map_ptr->speed_info.speed_limit_mode = static_cast<cem::message::env_model::SpeedLimitMode>(speed_limit_mode);
  }
  SetLaneSpeedLimit(bev_map);
  json debug_json;
  debug_json["accumulated_distance"] = accumulated_distance;
  debug_json["holding_distance"] = holding_distance;
  debug_json["current_vehicle_speed"] = current_vehicle_speed;
  debug_json["delta_time_second"] = cur_measurement_timestamp - pre_measurement_timestamp;
  debug_json["vision_speedlimit_value"] = vision_speedlimit_value;
  debug_json["vision_speedlimit_id"] = vision_speedlimit_id;
  std::string debug_str = debug_json.dump();
  bev_map.bev_map_ptr->debug_infos += debug_str;

  // 记录地图限速、导航限速和时间戳
  if (map_event_raw != nullptr && !map_event_raw->speed_limit_info.empty()) {
    pre_EHR_speedlimit = EHR_speedlimit;
  }
  if (navi_traf_raw != nullptr) {
    pre_Navi_speedlimit = Navi_speedlimit;
  }
  prepre_vision_speedlimit = pre_vision_speedlimit;
  pre_vision_speedlimit = vision_speedlimit_value;
  pre_measurement_timestamp = cur_measurement_timestamp;
}

void SpeedLimitFusion::UpdateHoldingDistance() {
  // 根据限速值计算保持阈值
  if (speed_limit_value < 20) {
    holding_distance = 200;
  } else if (speed_limit_value < 30) {
    holding_distance = 400;
  } else if (speed_limit_value < 40) {
    holding_distance = 500;
  } else if (speed_limit_value < 50) {
    holding_distance = 600;
  } else if (speed_limit_value < 70) {
    holding_distance = 1000;
  } else if (speed_limit_value < 80) {
    holding_distance = 1900;
  } else if (speed_limit_value < 90) {
    holding_distance = 2200;
  } else if (speed_limit_value < 100) {
    holding_distance = 2800;
  } else if (speed_limit_value < 110) {
    holding_distance = 3500;
  } else if (speed_limit_value < 130) {
    holding_distance = 3800;
  } else {
    holding_distance = 4000;
  }
  //将累积距离清空，从零计算距离
  accumulated_distance = 0;
}

void SpeedLimitFusion::CalcAccumulatedDistance() {
  if (std::abs(pre_measurement_timestamp) < 1e-5) {
    return;
  }
  // 根据当前车速和两帧时间差计算累积距离
  accumulated_distance += current_vehicle_speed * (cur_measurement_timestamp - pre_measurement_timestamp);
  // 如果累积距离大于保持阈值，那么取消限速
  if (accumulated_distance > holding_distance) {
    speed_limit_value = 0;
    speed_limit_status = cancelled;
  }
}

void SpeedLimitFusion::SetLaneSpeedLimit(DetectBevMap &bev_map) {
  if (bev_map.bev_map_ptr == nullptr) {
    return;
  }

  if (speed_limit_value != 0) {
    lane_speed_limit = speed_limit_value / 3.6;       //优先取TSR限速，km/h-->m/s
  } else if (EHR_speedlimit != 0) {
    lane_speed_limit = EHR_speedlimit / 3.6;          //其次取地图限速，km/h-->m/s
  } else if (vision_filter_value != 0) {
    lane_speed_limit = vision_filter_value / 3.6;     //然后取视觉限速，km/h-->m/s
  } else if (Navi_speedlimit != 0) {
    lane_speed_limit = Navi_speedlimit / 3.6;         //最后取导航限速，km/h-->m/s
  } else {
    lane_speed_limit = 20.0;
  }

  for (auto &lane : bev_map.bev_map_ptr->lane_infos) {
    lane.speed_limit = lane_speed_limit;
  }
}

}  // namespace fusion
}  // namespace cem