#include "routing_map_stopline_processor.h"
#include "crossroad_construction/cross_common.h"
namespace cem {
namespace fusion {

// 如果存在感知停止线和地图虚拟线纵向距离小于1.5m时，该路口就不再调整
constexpr double kNotAdjMinDis = 1.5;
constexpr double kNotAdjMaxtDis = 10;
// 调整后感知停止线和历史调整点纵向距离小于1.2m就不更新调整点
constexpr double kAdjMinDis = 1.2;

RoutingMapStopLineProcessor::RoutingMapStopLineProcessor() {}

RoutingMapStopLineProcessor::~RoutingMapStopLineProcessor() {}

void RoutingMapStopLineProcessor::Process(BevMapInfoPtr &bev_map_raw, const Eigen::Isometry3d &Twb, RoutingMapPtr routing_map_ptr,
const GeometryMatchInfo& geometry_match_info) {
  if (bev_map_raw == nullptr || routing_map_ptr == nullptr || routing_map_ptr->lanes.empty()) {
    XLOG << "stop-" << " bev_map_raw == nullptr";
    return;
  }
  routing_lane_unmap_.clear();
  for (auto &lane : routing_map_ptr->lanes) {
    routing_lane_unmap_[lane.id] = &lane;
  }
  Twb_ = Twb;
  Tbw_ = Twb.inverse();
  //TODO Reset ld_ego_lane_
  bool has_bev_stopline = FindBevStopline(bev_map_raw);
  ld_ego_lane_ = nullptr;
  bool has_traffic_light_lane = FindTrafficLightCrossroadLane(routing_map_ptr,geometry_match_info);
  ResampleLDEgoLanePoints();
  if(!ld_ego_lane_ || ld_ego_lane_->points.empty()){
    return;
  }
  // XLOG << "stop-"
  //            << "has_bev_stopline: " << has_bev_stopline << " , bev_stopline_dis: " << bev_stopline_dis_
  //            << " ,has_traffic_light_lane: " << has_traffic_light_lane;
  XLOG << "////////////////////////////////////////停止线  "
  << std::to_string(bev_map_raw->header.timestamp)<<"///////////////////////////////////////////////";
  XLOG << "has_bev_stopline: " << has_bev_stopline << " , bev_stopline_dis: " << bev_stopline_dis_
             << " ,has_traffic_light_lane: " << has_traffic_light_lane;
  // if (has_traffic_light_lane) {
  //   XLOG << "stop-"
  //              << "ld_ego_lane_id: " << ld_ego_lane_->id;
  // }
  // if (!(has_bev_stopline &&has_traffic_light_lane)) {
  if (!has_bev_stopline) {//没有停止线的情况
    //检查历史是否有调整cross_lanes_路口内的线
    if (cross_lanes_.empty()) {
      XLOG << "不调整-"
                 << " cross_lanes_.empty and has no history adj!!!";
      return;
    }
    // 历史有调整，自车超过裁剪线30m后清除返回，30m以内继续保持
    if (TransformPoint(dr_cut_point_, Tbw_).x < -30.0) {
      cross_lanes_.clear();
      ego_lane_ = cem::message::env_model::LaneInfo{};
      XLOG << "不调整-"
                 << "x < -30.0 clear!!!";
      return;
    }
    if (routing_lane_unmap_.find(ego_lane_.id) != routing_lane_unmap_.end()) {
      routing_lane_unmap_[ego_lane_.id]->points = ego_lane_.points;
    }
    for (const auto &lane : cross_lanes_) {
      if (routing_lane_unmap_.find(lane.id) != routing_lane_unmap_.end()) {
        // routing_lane_unmap_[lane.id]->points = lane.points;//直接使用历史点
      }
    }
    XLOG << "不调整-"
               << "has no stop and using history!!!";
    return;
  }

  // 有停止线和带红绿灯的lane时

  float ld_stop_lane_dis =  TransformPoint(ld_ego_lane_->points.back(), Tbw_).x;
  XLOG << " ++++++diff " << std::abs(bev_stopline_dis_ - ld_stop_lane_dis);
  // 如果出现偏差小于1.5m就不再调整
  if (std::abs(bev_stopline_dis_ - ld_stop_lane_dis) > kNotAdjMaxtDis || std::abs(bev_stopline_dis_ - ld_stop_lane_dis) < kNotAdjMinDis) {
    cross_lanes_.clear();
    not_adjust_lane_id_ = ld_ego_lane_->id;
    XLOG << "stop-"<< "not_adjust_lane_id: " << not_adjust_lane_id_;
    return;
  }

  if(cross_lanes_.empty()){
    // 进行调整
    AdjustLDLane(routing_map_ptr);
    XLOG << "第一次调整-"
               << "first adj ";
    return;
  }

  double cut_point_x = TransformPoint(dr_cut_point_, Tbw_).x;
  XLOG << "stop-"
             << "cut_point_x: " << cut_point_x;
  // 判断当前停止线和历史cut_point的位置关系
  double his_diff = std::abs(cut_point_x - bev_stopline_dis_);
  if (his_diff > kAdjMinDis) {
    XLOG << "stop-"
               << "his_diff new adj: " << his_diff;
    AdjustLDLane(routing_map_ptr);
    return;
  } else {
    XLOG << "stop-"
               << "his_diff using history: " << his_diff;
    if (routing_lane_unmap_.find(ego_lane_.id) != routing_lane_unmap_.end()) {
      routing_lane_unmap_[ego_lane_.id]->points = ego_lane_.points;
    }
    for (const auto &lane : cross_lanes_) {
      if (routing_lane_unmap_.find(lane.id) != routing_lane_unmap_.end()) {
        routing_lane_unmap_[lane.id]->points = lane.points;
      }
    }
  }
  return;
}

bool RoutingMapStopLineProcessor::AdjustLDLane(RoutingMapPtr routing_map_ptr) {
  float ld_stop_lane_dis = TransformPoint(ld_ego_lane_->points.back(), Tbw_).x;
  // 感知停止线提前，需要裁剪路口前车道，把点分别放到路口线中
  std::vector<cem::message::env_model::Point> fit_points;
  XLOG << "USED bev_stopline_dis_: " << bev_stopline_dis_;
  if (ld_stop_lane_dis > bev_stopline_dis_) {
    int cut_index = ld_ego_lane_->points.size();
    for (int i = 0; i < ld_ego_lane_->points.size(); i++) {
      XLOG << "EGO PT " << ld_ego_lane_->id  << " " << i << " " << TransformPoint(ld_ego_lane_->points[i], Tbw_).x << " " << ld_ego_lane_->points[i].y;
      if (TransformPoint(ld_ego_lane_->points[i], Tbw_).x > bev_stopline_dis_) {
        cut_index = i;
        break;
      }
    }
    XLOG << "stop-"
                 << "ego cut_index: " << cut_index << " ,ld_ego_lane_->points.size: " << ld_ego_lane_->points.size();
    if (cut_index == ld_ego_lane_->points.size() || cut_index == 0) {
      return false;
    } else {
      fit_points.reserve(ld_ego_lane_->points.size() - cut_index + 1);
      auto                           pre_pt  = TransformPoint(ld_ego_lane_->points[cut_index - 1], Tbw_);
      auto                           back_pt = TransformPoint(ld_ego_lane_->points[cut_index], Tbw_);
      double                         y = back_pt.y - (back_pt.y - pre_pt.y) * ((back_pt.x - bev_stopline_dis_) / (back_pt.x - pre_pt.x));
      cem::message::env_model::Point s_p;
      s_p.x                                 = bev_stopline_dis_;
      s_p.y                                 = y;
      XLOG << "S_P: " << s_p.x << " " << s_p.y;
      dr_cut_point_ = TransformPoint(s_p, Twb_);
      fit_points.emplace_back(dr_cut_point_);
      for (int i = cut_index; i < ld_ego_lane_->points.size(); i++) {
        fit_points.emplace_back(ld_ego_lane_->points[i]);
      }
      ld_ego_lane_->points[cut_index].x = dr_cut_point_.x;
      ld_ego_lane_->points[cut_index].y = dr_cut_point_.y;
      ld_ego_lane_->points.resize(cut_index + 1);
      ego_lane_ = *ld_ego_lane_;
      // 接着修改路口后的lane
      cross_lanes_.clear();
      for (auto id : ld_ego_lane_->next_lane_ids) {
        if(routing_lane_unmap_.find(id) == routing_lane_unmap_.end()){
            continue;
        }
        auto* lane = routing_lane_unmap_[id];
        if(lane->points.empty()){
            continue;
        }
        lane->points.insert(lane->points.begin(), fit_points.begin(), fit_points.end());
        cross_lanes_.emplace_back(*lane);
      }
      return !cross_lanes_.empty();
    }
  }
  // 感知停止线在地图点的后面，需要裁剪路口后的多个车道，其中一个线的点放到前面线中
  cross_lanes_.clear();
  for (auto id : ld_ego_lane_->next_lane_ids) {
    if (routing_lane_unmap_.find(id) == routing_lane_unmap_.end()) {
      continue;
    }
    auto *lane = routing_lane_unmap_[id];
    if (lane->points.empty()) {
      continue;
    }
    int cut_index = lane->points.size();
    for (int i = 0; i < lane->points.size(); i++) {
      if (TransformPoint(lane->points[i], Tbw_).x > bev_stopline_dis_) {
        cut_index = i;
        break;
      }
    }
    XLOG << "stop-"
               << "virtual cut_index: " << cut_index << " ,lane->points.size: " << lane->points.size();
    if (cut_index == lane->points.size() || cut_index == 0) {
      return false;
    } else {
      fit_points.clear();
      fit_points.reserve(lane->points.size() - cut_index + 1);
      auto                           pre_pt  = TransformPoint(lane->points[cut_index - 1], Tbw_);
      auto                           back_pt = TransformPoint(lane->points[cut_index], Tbw_);
      double                         y = back_pt.y - (back_pt.y - pre_pt.y) * ((back_pt.x - bev_stopline_dis_) / (back_pt.x - pre_pt.x));
      cem::message::env_model::Point s_p;
      s_p.x         = bev_stopline_dis_;
      s_p.y         = y;
      dr_cut_point_ = TransformPoint(s_p, Twb_);
      for (int i = 0; i < cut_index; i++) {
        fit_points.emplace_back(lane->points[i]);
      }
      fit_points.emplace_back(dr_cut_point_);
      
      lane->points.assign(lane->points.begin() + cut_index - 1, lane->points.end());
      lane->points[0].x = dr_cut_point_.x;
      lane->points[0].y = dr_cut_point_.y;
      cross_lanes_.emplace_back(*lane);
    }
  }

  if (fit_points.size() > 0 && !cross_lanes_.empty()) {
    ld_ego_lane_->points.insert(ld_ego_lane_->points.end(), fit_points.begin(), fit_points.end());
    ego_lane_ = *ld_ego_lane_;
    return true;
  }
  return false;
}

// 检查50m内有绑定红灯的路口中心线
bool RoutingMapStopLineProcessor::FindTrafficLightCrossroadLane(RoutingMapPtr            routing_map_ptr,
                                                                const GeometryMatchInfo &geometry_match_info) {
  // 取出与bev相匹配的LD的egolaneid
  if (geometry_match_info.GetEgoLanes() == std::nullopt || geometry_match_info.GetEgoLanes()->ld_lane_.empty()) {
    return false;
  }
  auto ego_lane_id = geometry_match_info.GetEgoLanes()->ld_lane_.front().lane_id;
  XLOG << " RoutingMapStopLineProcessor LDEgoLaneId: " << ego_lane_id;
  if (routing_lane_unmap_.find(ego_lane_id) == routing_lane_unmap_.end()) {
    return false;
  }
  auto *it_ego_lane = routing_lane_unmap_[ego_lane_id];
  if (it_ego_lane->points.empty()) {
    return false;
  }
  // 当前lane绑定了红绿灯也返回
  if (it_ego_lane->light_status == cem::message::env_model::LightStatus::YELLOW_LIGHT ||
      it_ego_lane->light_status == cem::message::env_model::LightStatus::YELLOW_LIGHT ||
      it_ego_lane->light_status == cem::message::env_model::LightStatus::RED_LIGHT ||
      it_ego_lane->light_status == cem::message::env_model::LightStatus::GREEN_LIGHT) {
    return false;
  }
  double now_length = TransformPoint(it_ego_lane->points.back(), Tbw_).x;
  if (now_length > 50.0 || it_ego_lane->next_lane_ids.empty()) {
    return false;
  }
  int  index{0};
  bool has_traffic_light_lane{false};//has_traffic_light_lane查询
  while (now_length < 50.0 && index < 10) {
    index++;
    if (it_ego_lane->next_lane_ids.empty()) {
      break;
    }
    for (auto next_id : it_ego_lane->next_lane_ids) {
      if (routing_lane_unmap_.find(next_id) == routing_lane_unmap_.end()) {
        continue;
      }
      auto *it_next_lane = routing_lane_unmap_[next_id];
      if (it_next_lane->light_status == cem::message::env_model::LightStatus::YELLOW_LIGHT ||
          it_next_lane->light_status == cem::message::env_model::LightStatus::GREEN_BLINKING ||
          it_next_lane->light_status == cem::message::env_model::LightStatus::YELLOW_BLINKING ||
          it_next_lane->light_status == cem::message::env_model::LightStatus::RED_LIGHT) {
        has_traffic_light_lane = true;
        XLOG << "has_traffic_light_lane";
        break;
      }
    }
    if (has_traffic_light_lane) {
      break;
    }
    if (it_ego_lane->next_lane_ids.size() != 1) {
      break;
    }
    auto next_lane_id = it_ego_lane->next_lane_ids.front();
    if (routing_lane_unmap_.find(next_lane_id) == routing_lane_unmap_.end()) {
      break;
    }
    it_ego_lane = routing_lane_unmap_[next_lane_id];//更新迭代器
    if (it_ego_lane->points.size() < 2) {
      break;
    }
    now_length += it_ego_lane->length;
  }
  ld_ego_lane_ = it_ego_lane;//更新ld_ego_lane_
  if (!has_traffic_light_lane) {
    XLOG << "stop-"
               << "has_traffic_light_lane: " << has_traffic_light_lane << " ,it_ego_lane->id: " << it_ego_lane->id;
    return false;
  }
  XLOG << "while " << ld_ego_lane_->id;
  return has_traffic_light_lane;
}

bool RoutingMapStopLineProcessor::FindBevStopline(BevMapInfoPtr &bev_map_raw) {
  double min_dis = 100.0;
  bool   has_stop_line{false};
  // 寻找35m以内的最近的且在自车前方的停止线
  for (const auto &stop_lane : bev_map_raw->stop_lines) {
    if (stop_lane.line_points.size() < 2) {
      continue;
    }
    if (stop_lane.line_points.front().x > 35.0f || stop_lane.line_points.front().x < 0.0f) {
      continue;
    }
    auto bc = Eigen::Vector2f(stop_lane.line_points[1].x -stop_lane.line_points[0].x, stop_lane.line_points[1].y - stop_lane.line_points[0].y);
    auto ob = Eigen::Vector2f(0.0f - stop_lane.line_points[0].x, 0.0f - stop_lane.line_points[0].y);
    bool is_between = false;
    auto t = ob.dot(bc)/bc.squaredNorm();
    if (abs(bc.norm()) < 0.0001 || t < -cross_epslion || t > 1.0f + cross_epslion) {
      continue;//侧边停止线不进入判断
    }
    if (min_dis < stop_lane.line_points.front().x) {
      XLOG << " 295";
      continue;
    }
    min_dis = stop_lane.line_points.front().x;
    if (GetDistPointLane(Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(stop_lane.line_points[0].x, stop_lane.line_points[0].y),
                         Eigen::Vector2f(stop_lane.line_points[1].x, stop_lane.line_points[1].y), bev_stopline_dis_)) {
      has_stop_line = true;
    }
  }
  return has_stop_line;
}
void RoutingMapStopLineProcessor::ResampleLDEgoLanePoints() {
  if (!ld_ego_lane_ || ld_ego_lane_->points.size() < 2) {
    return;
  }
  
  // 计算原始点列的总长度
  double total_length = 0.0;
  std::vector<double> segment_lengths;
  segment_lengths.reserve(ld_ego_lane_->points.size() - 1);
  
  for (size_t i = 0; i < ld_ego_lane_->points.size() - 1; ++i) {
    const auto& p1 = ld_ego_lane_->points[i];
    const auto& p2 = ld_ego_lane_->points[i + 1];
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double seg_len = std::sqrt(dx * dx + dy * dy);
    segment_lengths.push_back(seg_len);
    total_length += seg_len;
  }
  
  if (total_length < 1e-6) {
    return;  // 长度太短，不需要重采样
  }
  
  // 设置采样间隔（例如0.5米），可以根据实际需求调整
  const double sample_interval = 0.5;
  int num_samples = static_cast<int>(std::ceil(total_length / sample_interval));
  if (num_samples < 2) {
    num_samples = 2;  // 至少保留起点和终点
  }
  
  std::vector<cem::message::env_model::Point> resampled_points;
  resampled_points.reserve(num_samples);
  
  // 添加起点
  resampled_points.push_back(ld_ego_lane_->points.front());
  
  // 线性插值采样
  double current_dist = 0.0;
  size_t seg_idx = 0;
  double seg_start_dist = 0.0;
  
  for (int i = 1; i < num_samples - 1; ++i) {
    double target_dist = i * sample_interval;
    
    // 找到目标距离所在的线段
    while (seg_idx < segment_lengths.size() && 
           current_dist + segment_lengths[seg_idx] < target_dist - 1e-6) {
      current_dist += segment_lengths[seg_idx];
      seg_start_dist = current_dist;
      seg_idx++;
    }
    
    if (seg_idx >= segment_lengths.size()) {
      break;  // 不应该发生，但安全处理
    }
    
    // 在线段内插值
    double seg_progress = (target_dist - seg_start_dist) / segment_lengths[seg_idx];
    seg_progress = std::max(0.0, std::min(1.0, seg_progress));
    
    const auto& p1 = ld_ego_lane_->points[seg_idx];
    const auto& p2 = ld_ego_lane_->points[seg_idx + 1];
    
    cem::message::env_model::Point new_point;
    new_point.x = p1.x + seg_progress * (p2.x - p1.x);
    new_point.y = p1.y + seg_progress * (p2.y - p1.y);
    
    resampled_points.push_back(new_point);
    // XLOG << "resampled_points "  << new_point.x  << " " << new_point.y;
  }
  
  // 添加终点
  resampled_points.push_back(ld_ego_lane_->points.back());
  
  // 更新车道点
  ld_ego_lane_->points = resampled_points;
  return;
}
}  // namespace fusion
}  // namespace cem