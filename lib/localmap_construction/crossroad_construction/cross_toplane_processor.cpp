#include "cross_toplane_processor.h"
#include "lib/common/fitter/least_square_fitter.h"
#include "common/utils/lane_geometry.h"

namespace cem {
namespace fusion {

CrossToplaneProcessor::CrossToplaneProcessor() {}

void CrossToplaneProcessor::Init(const std::shared_ptr<CrossDataManager>& data_manager){
    data_manager_ = data_manager;
    average_heading_filter_.SetCoefficient(0.16);
}

void CrossToplaneProcessor::Reset() {
  cross_section_id_ = 0;
  top_lane_ids_.clear();
  lane_id_dis_.clear();
  top_edge_ids_.clear();
  connect_lane_id_ = 0;
  connect_lane_points_.clear();
  his_average_heading_ = 0.0;
  average_heading_filter_.ReSet();
  opening_ = {};
  his_connect_top_lane_.Reset();
  his_connect_top_lane_index_ = 0;
}

void CrossToplaneProcessor::DataUpdate(){}

void CrossToplaneProcessor::GetCrossSectionId() {
  const auto &navi_start = data_manager_->routing_map_raw()->sd_route.navi_start;
  const auto &mpp_section = data_manager_->routing_map_raw()->sd_route.mpp_sections;
  const auto it_navi_seciton = std::find_if(
      mpp_section.begin(), mpp_section.end(),
      [&navi_start](const cem::message::env_model::SDSectionInfo &p) { return p.id == navi_start.section_id; });
  if (it_navi_seciton == mpp_section.end()) {
    return;
  }

  for (auto it = it_navi_seciton; it != mpp_section.end(); it++) {
    if (it->has_junction) {
      auto it_junction = it + 1;
      if (it_junction != mpp_section.end()) {
        cross_section_id_ = it_junction->id;
        break;
      }
    }
  }
}

void CrossToplaneProcessor::Preprocessing() {
  opening_.Clear();
  double top_lane_dis =
      data_manager_->GetSdSections().has_intersection_zone ? data_manager_->GetSdSections().cross_point.x() : 15.0;
  top_lane_ids_.clear();
  std::vector<double> top_lane_dis_vec;
  vector<std::tuple<double, double, double>> heading_infos;
  std::set<uint32_t> lanemark_ids;
  // 收集中心线和车道线以及存储起始点
  for (const auto &lane : data_manager_->bev_map_raw()->lane_infos) {
    XLOG << "opening top_lane_ids_ " <<  "lane.previous_lane_ids.empty() " << lane.previous_lane_ids.empty() <<  " lane.line_points.size() " <<  lane.line_points.size() << " lane.line_points.front().x " << lane.line_points.front().x;
    if (lane.previous_lane_ids.empty() && lane.line_points.size() > 2  && lane.line_points.front().x > 1.f
        // lane.line_points.front().x > top_lane_dis + 2.0 && lane.length > 15.0) {
      ) {
      top_lane_ids_.emplace_back(lane.id);
      XLOG << "FIND CLOSED lane.id"   << lane.id;
      top_lane_dis_vec.emplace_back(lane.line_points.front().x);
      double heading{0.0};
      double average_point_err{100.0};
      double interior_points_percent{0.0};
      if (GetPointsHeading(lane.line_points, heading, average_point_err, interior_points_percent)) {
        // FLOG_CROSS << "lane id: " << lane.id << " ,heading_it heading: " << heading << " ,average_point_err: " << average_point_err
        //            << " ,interior_points_percent: " << interior_points_percent;
        heading_infos.emplace_back(heading, average_point_err, interior_points_percent);
      }
    }
    if(data_manager_->GetLaneMarkerById(lane.left_lane_id) ==nullptr){
      continue;
    }
    const auto *left_lanemarker = data_manager_->GetLaneMarkerById(lane.left_lane_id);
    if (left_lanemarker != nullptr && left_lanemarker->line_points.size() > 2) {
      lanemark_ids.insert(lane.left_lane_id);
    }
    if(data_manager_->GetLaneMarkerById(lane.right_lane_id) ==nullptr){
      continue;
    }
    const auto *right_lanemarker = data_manager_->GetLaneMarkerById(lane.right_lane_id);
    if (right_lanemarker != nullptr && right_lanemarker->line_points.size() > 2) {
      lanemark_ids.insert(lane.right_lane_id);
    }
  }
  // for(const auto& id:top_lane_ids_){
  //   FLOG_CROSS << " first top lane id: " << id;
  // }
  // 收集lanemarker的起始点
  for (auto id : lanemark_ids) {
    top_lane_dis_vec.emplace_back(data_manager_->GetLaneMarkerById(id)->line_points.front().x);
    double heading{0.0};
    double average_point_err{100.0};
    double interior_points_percent{0.0};
    if (GetPointsHeading(data_manager_->GetLaneMarkerById(id)->line_points, heading, average_point_err, interior_points_percent)) {
      // FLOG_CROSS << "lanemarker id: " << id << " ,heading_it heading: " << heading << " ,average_point_err: " << average_point_err
      //            << " ,interior_points_percent: " << interior_points_percent;
      heading_infos.emplace_back(heading, average_point_err, interior_points_percent);
    }
  }


  // 计算top_lane_dis和路口后的点
  byd::common::math::Vec2d sd_cross_top_point;
  if (data_manager_->GetSdSections().has_intersection_zone) {
    top_lane_dis = data_manager_->GetSdSections().end_point.x();
    sd_cross_top_point = data_manager_->GetSdSections().end_point;
  } else if (top_lane_dis_vec.size() > 2 || top_lane_ids_.size() > 0) {
    top_lane_dis = std::accumulate(top_lane_dis_vec.begin(), top_lane_dis_vec.end(), 0.0) / top_lane_dis_vec.size();
    const auto &points = data_manager_->GetSdSections().points;
    for (int i = 1; i < points.size(); i++) {
      if (points[i].x() > top_lane_dis) {
        double y = points[i - 1].y() +
                   ((points[i].y() - points[i - 1].y()) / (points[i].x() - points[i - 1].x())) * (top_lane_dis - points[i - 1].x());
        sd_cross_top_point = byd::common::math::Vec2d(top_lane_dis, y);
        break;
      }
    }
  } else {
    his_average_heading_ = 0.0;
    XLOG << " has no top lanes!!! ";
    return;
  }
  XLOG << " top_lane_dis: " << top_lane_dis << " ,sd_cross_top_point_X:" << sd_cross_top_point.x()
             << " ,sd_cross_top_point_y:" << sd_cross_top_point.y();
  // 收集路沿并计算heading
  top_edge_ids_.clear();
  for (const auto& edge : data_manager_->bev_map_raw()->edges) {
    if(edge.id > 100) continue;
    if (!edge.line_points.empty() && edge.line_points.back().x > top_lane_dis - 20 &&
        edge.line_points.front().x < top_lane_dis + 30) {
      // FLOG_CROSS << " top_edges id: " << edge.id;
      top_edge_ids_.emplace_back(edge.id);
      double heading{0.0};
      double average_point_err{100.0};
      double interior_points_percent{0.0};
      if (GetPointsHeading(edge.line_points, heading, average_point_err, interior_points_percent)) {
        // FLOG_CROSS << "edge id: " << edge.id << " ,heading_it heading: " << heading << " ,average_point_err: " << average_point_err
        //            << " ,interior_points_percent: " << interior_points_percent;
        heading_infos.emplace_back(heading, average_point_err, interior_points_percent);
        // AINFO << "EDAGE heading: " << heading << "id : " << edge.id;

      }
    }
  }

  // 计算路口后的heading
  double average_heading = 0.0;
  if (!GetAverageHeading(heading_infos, average_heading)) {
    his_average_heading_ = 0.0;
    FLOG_CROSS << " has no top heading!!! ";
    return;
  }
  opening_.has_heading = true;
  // CalculateRoadHeading();
  his_average_heading_ = average_heading;

  // 再次对lane进行过滤，主要去掉过远的lane
  // for (auto it = top_lane_ids_.begin(); it != top_lane_ids_.end();) {
  //   // if(data_manager_ == nullptr ||data_manager_->GetLaneById(*it) == nullptr){
  //   //   continue;
  //   // }
  //   const auto& line_points = data_manager_->GetLaneById(*it)->line_points;
  //   // if (line_points.back().x < top_lane_dis - 20 || line_points.front().x > top_lane_dis + 30) {
  //   if (0) {
  //     it = top_lane_ids_.erase(it);
  //   } else {
  //     it++;
  //   }
  // }
  // for(const auto& id:top_lane_ids_){
  //   FLOG_CROSS << " filter top lane id: " << id;
  // }

  // 求取车道中心线在top_lane_dis处的y值
  lane_id_dis_.clear();
  for (auto id : top_lane_ids_) {
    if(data_manager_ == nullptr ||data_manager_->GetLaneById(id) == nullptr){
      continue;
    }
    const auto& line_points = data_manager_->GetLaneById(id)->line_points;
    if(line_points.empty() ){
      continue;
    }
    double y = GetPointsY(line_points, top_lane_dis, average_heading);
    lane_id_dis_.insert({id, y});
  }
  std::sort(top_lane_ids_.begin(), top_lane_ids_.end(),
            [&](uint32_t a, uint32_t b) { return lane_id_dis_[a] > lane_id_dis_[b]; });

  // 寻找双黄线
  bool has_double_yellow_solid{false};
  uint32_t double_yellow_solid_id{0};
  double double_yellow_solid_y{0.0};
  for (auto id : lanemark_ids) {
    if(data_manager_ == nullptr ||data_manager_->GetLaneMarkerById(id) == nullptr){
      continue;
    }
    const auto* lanemarker = data_manager_->GetLaneMarkerById(id);
    if(lanemarker->line_points.empty()){
      continue;
    }
    if (lanemarker->color == 1 && (lanemarker->type == 4 || lanemarker->type == 6)) {
      has_double_yellow_solid = true;
      double_yellow_solid_id = id;
      double_yellow_solid_y = GetPointsY(lanemarker->line_points, top_lane_dis, average_heading);
    }
  }

  // 有双黄线就线过滤下车道
  // if (has_double_yellow_solid) {
  //   for (auto it = top_lane_ids_.begin(); it != top_lane_ids_.end();) {
  //     if (lane_id_dis_[*it] > double_yellow_solid_y) {
  //       XLOG << "Erase toplane_ids " << *it;
  //       it = top_lane_ids_.erase(it);
  //     } else {
  //       XLOG << "KEEP toplane_ids " << *it;
  //       it++;
  //     }
  //   }
  // }

  // 求取edge的y
  std::map<uint32_t, double> road_edge_id_dis;
  for (const auto &edge : data_manager_->bev_map_raw()->edges) {
    for (auto id : top_edge_ids_) {
      if (edge.id == id) {
        const auto &line_points = edge.line_points;
        // FLOG_CROSS << " road_edge_id: " << id << " ,top_lane_dis: " << top_lane_dis << " ,average_heading: " << average_heading;
        // for(const auto& point:line_points){
        //   FLOG_CROSS << " road_edge_point x: " << point.x << " ,y: " << point.y;
        // }
        double y = GetPointsY(line_points, top_lane_dis, average_heading);
        // FLOG_CROSS << " road_edge_y: " << y;
        road_edge_id_dis.insert({id, y});
      }
    }
  }
  std::sort(top_edge_ids_.begin(), top_edge_ids_.end(),
            [&](uint32_t a, uint32_t b) { return road_edge_id_dis[a] > road_edge_id_dis[b]; });

  double opening_left_y{100.0};
  double opening_right_y{-100.0};
  bool has_opending{true};
  // 根据路口后的sd点寻找左右两边的路沿
  if (has_double_yellow_solid) {
    opening_left_y = double_yellow_solid_y;
    opening_.has_left_edge = true;
    auto it_edge = std::find_if(top_edge_ids_.begin(), top_edge_ids_.end(),
                                [&](uint32_t a) { return road_edge_id_dis[a] + 2.0 < opening_left_y; });
    if (it_edge != top_edge_ids_.end()) {
      opening_right_y = road_edge_id_dis[*it_edge];
      opening_.has_right_edge = true;
    } else {
      double lane_right_y = top_lane_ids_.empty() ? opening_left_y : lane_id_dis_[top_lane_ids_.back()] - 1.875;
      opening_right_y = std::min(lane_right_y, sd_cross_top_point.y() - 1.75);
      if (opening_right_y > opening_left_y - 2.0) {
        has_opending = false;
      }
    }
  }else{
    // 分四种情况 1.左右边界都有 2.有左边界无右边界 3.无右边界有左边界 4.左右边界都无
    bool has_left_edge{false};
    bool has_right_edge{false};
    uint32_t left_edge_id{0};
    uint32_t right_edge_id{0};
    for (int i = 0; i < top_edge_ids_.size(); i++) {
      if (road_edge_id_dis[top_edge_ids_[i]] < sd_cross_top_point.y()) {
        has_right_edge = true;
        right_edge_id = top_edge_ids_[i];
        if (i > 0 && road_edge_id_dis[top_edge_ids_[i - 1]] < sd_cross_top_point.y()) {
          has_left_edge = true;
          right_edge_id = top_edge_ids_[i - 1];
        }
        break;
      }
    }
    if (!has_left_edge && !top_edge_ids_.empty() && road_edge_id_dis[top_edge_ids_.back()] > sd_cross_top_point.y()) {
      has_left_edge = true;
      right_edge_id = top_edge_ids_.back();
    }
    if(has_left_edge){
      opening_left_y = road_edge_id_dis[left_edge_id];
      opening_.has_left_edge = true;
    }
    if(has_right_edge){
      opening_right_y = road_edge_id_dis[right_edge_id];
      opening_.has_right_edge = true;
    }
    if(!has_left_edge){
      double lane_left_y = top_lane_ids_.empty() ? opening_right_y : lane_id_dis_[top_lane_ids_.front()] + 1.875;
      opening_left_y = std::max(lane_left_y, sd_cross_top_point.y() + 1.75);
      if (opening_left_y < opening_right_y + 2.0) {
        has_opending = false;
      }
    }
    if(!has_right_edge){
      double lane_right_y = top_lane_ids_.empty() ? opening_left_y : lane_id_dis_[top_lane_ids_.back()] - 1.875;
      opening_right_y = std::min(lane_right_y, sd_cross_top_point.y() - 1.75);
      if (opening_right_y > opening_left_y - 2.0) {
        has_opending = false;
      }
    }
  }
  // 使用左右边界再过滤一遍lane
  // for (auto it = top_lane_ids_.begin(); it != top_lane_ids_.end();) {
  //   if (lane_id_dis_[*it] > opening_left_y || lane_id_dis_[*it] < opening_right_y) {
  //     XLOG << "Erase toplane_ids " << *it;
  //     it = top_lane_ids_.erase(it);
  //   } else {
  //     it++;
  //   }
  // }
  // 增加路口过滤
  if(data_manager_->GetSdSections().has_intersection_zone){
    double delta_angle = std::abs(average_heading - data_manager_->GetSdSections().top_angle());
    if(delta_angle > 0.8){
      has_opending = false;
    }
  }
  opening_.is_valid = has_opending;
  opening_.dis = top_lane_dis;
  opening_.heading = average_heading;
  opening_.lane_ids = top_lane_ids_;
  opening_.lane_size = top_lane_ids_.size();
  opening_.left_y = opening_left_y;
  opening_.right_y = opening_right_y;
  opening_.sd_cross_top_point = sd_cross_top_point;

  data_manager_->SetOpening(opening_);
  // log
  XLOG << " has_opending: " << has_opending << " ,top_lane_dis: " << top_lane_dis << " ,opening_left_y: " << opening_left_y
             << " ,opening_right_y: " << opening_right_y;
  for (auto id : top_lane_ids_) {
    if(data_manager_ == nullptr ||data_manager_->GetLaneById(id) == nullptr){
        continue;
    }
    auto *lane = data_manager_->GetLaneById(id);
    FLOG_CROSS << " ID: " << id;
  }
  for (auto id : top_lane_ids_) {

    XLOG << " top_lane_ids_: " << id;
  }
}

void CrossToplaneProcessor::CalculateRoadHeading() {
  if (data_manager_->GetSdSections().points.size() < 2 ||
      data_manager_->bev_map_raw()->crosswalks.empty()) {
    return;
  }

  size_t point_size = data_manager_->GetSdSections().points.size();
  Eigen::Vector2d tail_point_1st(data_manager_->GetSdSections().points.at(point_size - 1).x(),
                                 data_manager_->GetSdSections().points.at(point_size - 1).y());
  Eigen::Vector2d tail_point_2nd(data_manager_->GetSdSections().points.at(point_size - 2).x(),
                                 data_manager_->GetSdSections().points.at(point_size - 2).y());
  Eigen::Vector2d tail_vector = tail_point_1st - tail_point_2nd;

  for (const auto& crosswalk : data_manager_->bev_map_raw()->crosswalks) {
    if (crosswalk.line_points.size() < 4) {
      continue;
    }
    if (crosswalk.line_points.front().x < 0) {
      continue;
    }
    std::vector<Eigen::Vector2d> ego_crosswalks;
    for (const auto &point : crosswalk.line_points)
    {
        Eigen::Vector2d ego_point(point.x, point.y);
        ego_crosswalks.push_back(ego_point);
    }
    std::sort(ego_crosswalks.begin(), ego_crosswalks.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b)
    {
        return a.x() < b.x();
    });
    Eigen::Vector2d tail_to_crosswalk_pt1 = ego_crosswalks.front() - tail_point_1st;
    Eigen::Vector2d tail_to_crosswalk_pt2 = ego_crosswalks.back() - tail_point_1st;
    double cross_1st = tail_vector.x() * tail_to_crosswalk_pt1.y() - tail_vector.y() * tail_to_crosswalk_pt1.x();
    double cross_2nd = tail_vector.x() * tail_to_crosswalk_pt2.y() - tail_vector.y() * tail_to_crosswalk_pt2.x();

    if (cross_1st * cross_2nd < 0) {
      // double his_average_heading_ = std::atan2(tail_vector.y(), tail_vector.x());
      // AINFO << "his_average_heading_: " << his_average_heading_ << "  crosswalk.id:  " << crosswalk.id;
      break;
    }
  }
}

double CrossToplaneProcessor::GetPointsY(const std::vector<cem::fusion::Point2DF> &points, double input_x, double heading) {
  double output_y{0.0};
  if (points.front().x > input_x) {
    output_y = points.front().y - std::tan(heading) * (points.front().x - input_x);
  } else if (points.back().x < input_x) {
    output_y = points.back().y + std::tan(heading) * (input_x - points.back().x);
  } else {
    for (int i = 1; i < points.size(); i++) {
      if (std::abs(points[i].x - input_x) < 0.5) {
        output_y = points[i].y;
        break;
      } else if (points[i].x > input_x) {
        output_y = points[i - 1].y + ((points[i].y - points[i - 1].y) / (points[i].x - points[i - 1].x)) * (input_x - points[i - 1].x);
        break;
      }
    }
  }
  return output_y;
}

bool CrossToplaneProcessor::GetPointsHeading(const std::vector<cem::fusion::Point2DF>& points, double& line_heading,
                                             double& average_point_err, double& interior_points_percent) {
  if (points.size() < 4 || std::abs(points.front().x - points.back().x) < 20.0) {
    return false;
  }
  constexpr double min_delta_x = 0.2;
  const auto it_end = std::find_if(points.begin(), points.end(), [&points](const cem::fusion::Point2DF& point) {
    return point.x - 30.0 > points.begin()->x;
  });

  cem::fusion::Point2DF end_point = it_end == points.end() ? points.back() : *it_end;
  int point_size = it_end == points.end() ? points.size() : (it_end - points.begin() + 1);

  const auto it_mid =
      std::find_if(points.begin(), points.end(), [&points, &end_point](const cem::fusion::Point2DF& point) {
        return point.x > (points.begin()->x + end_point.x) / 2;
      });
  // start-->end
  {
    if (Cross_Compare(points.front().x, end_point.x) == 0) {
      FLOG_CROSS << " Get no points headings!!! front.x: " << points.front().x << " , end x: " << end_point.x;
      return false;
    }
    double c1 = (end_point.y - points.front().y) / (end_point.x - points.front().x);
    double c0 = points.front().y - c1 * points.front().x;
    int count{0};
    double sum_dis{0.0};
    for (int i = 0; i < point_size; i++) {
      double dis = std::abs(points[i].y - (c1 * points[i].x + c0)) / std::sqrt(c1 * c1 + 1);
      if (dis < min_delta_x) {
        count++;
      }
      sum_dis += dis;
    }
    double percent = count / (point_size * 1.0);
    line_heading = std::atan(c1);
    average_point_err = sum_dis / point_size;
    interior_points_percent = percent;
  }

  if (it_mid == points.end()) {
    FLOG_CROSS << " has no it mid ";
    return true;
  }

  // start-->mid
  {
    cem::fusion::Point2DF start_point = points.front();
    if (Cross_Compare(start_point.x, it_mid->x) == 0) {
      FLOG_CROSS << " start-->mid 1 err";
      return true;
    }
    double c1 = (it_mid->y - start_point.y) / (it_mid->x - start_point.x);
    double c0 = start_point.y - c1 * start_point.x;
    int count{0};
    double sum_dis{0.0};
    for (int i = 0; i < point_size; i++) {
      double dis = std::abs(points[i].y - (c1 * points[i].x + c0)) / std::sqrt(c1 * c1 + 1);
      if (dis < min_delta_x) {
        count++;
      }
      sum_dis += dis;
    }
    double percent = count / (point_size * 1.0);
    if (percent > interior_points_percent) {
      line_heading = std::atan(c1);
      average_point_err = sum_dis / point_size;
      interior_points_percent = percent;
    }
  }

  // mid-->end
  {
    if (Cross_Compare(it_mid->x, end_point.x) == 0) {
      FLOG_CROSS << " mid-->end err";
      return true;
    }
    double c1 = (end_point.y - it_mid->y) / (end_point.x - it_mid->x);
    double c0 = it_mid->y - c1 * it_mid->x;
    int count{0};
    double sum_dis{0.0};
    for (int i = 0; i < point_size; i++) {
      double dis = std::abs(points[i].y - (c1 * points[i].x + c0)) / std::sqrt(c1 * c1 + 1);
      if (dis < min_delta_x) {
        count++;
      }
      sum_dis += dis;
    }
    double percent = count / (point_size * 1.0);
    if (percent > interior_points_percent) {
      line_heading = std::atan(c1);
      average_point_err = sum_dis / point_size;
      interior_points_percent = percent;
    }
  }
  const auto it_mid_1 = std::find_if(points.begin(), it_mid, [&points, &it_mid](const cem::fusion::Point2DF& point) {
    return point.x > (points.begin()->x + it_mid->x) / 2;
  });
  if (it_mid_1 == points.end()) {
    FLOG_CROSS << " has no it_mid_1";
    return true;
  }
  const auto it_mid_2 = std::find_if(it_mid, it_end, [&points, &it_mid](const cem::fusion::Point2DF& point) {
    return point.x > (points.back().x + it_mid->x) / 2;
  });
  if (it_mid_2 == points.end()) {
    FLOG_CROSS << " has no it_mid_2";
    return true;
  }
  // 1/4mid-->3/4mid
  {
    if (Cross_Compare(it_mid_1->x, it_mid_2->x) == 0) {
      FLOG_CROSS << " 1/4mid-->3/4mid err";
      return true;
    }
    double c1 = (it_mid_2->y - it_mid_1->y) / (it_mid_2->x - it_mid_1->x);
    double c0 = it_mid_1->y - c1 * it_mid_1->x;
    int count{0};
    double sum_dis{0.0};
    for (int i = 0; i < point_size; i++) {
      double dis = std::abs(points[i].y - (c1 * points[i].x + c0)) / std::sqrt(c1 * c1 + 1);
      if (dis < min_delta_x) {
        count++;
      }
      sum_dis += dis;
    }
    double percent = count / (point_size * 1.0);
    if (percent > interior_points_percent) {
      line_heading = std::atan(c1);
      average_point_err = sum_dis / point_size;
      interior_points_percent = percent;
    }
  }
  return true;
}

bool CrossToplaneProcessor::FindClosetLane(const BevLaneInfo &virtual_lane) {
  // 默认优先使用topo，topo不处理转向；LatErr在40m才开始起作用
  
  bool is_using_topo = is_enable_mnoa ? (data_manager_->GetVirtualLineSource() != VirtualLineSource::LD &&
                                         data_manager_->GetVirtualLineSource() != VirtualLineSource::EXP)
                                      : true;
  XLOG << "is_enable_mnoa  " << is_enable_mnoa << " data_manager_->GetVirtualLineSource() " << static_cast<int>(data_manager_->GetVirtualLineSource()) << " is_using_topo " << is_using_topo ;

  if (KEnableFindClosetLaneByTopo && is_using_topo) {
    data_manager_->SetIsConnectByTopo(FindClosetLaneByTopo(virtual_lane));
  }
  if (data_manager_->is_find_closetlane_bytopo_success()) {
    auto &top_lane = data_manager_->GetTopLane();
    if (top_lane.points.empty()) {
      data_manager_->SetIsConnectByTopo(false);
      XLOG << "top_lane  !!! ";
      return false;
    }
    his_connect_top_lane_ = top_lane;
    his_connect_top_lane_.points.clear();
    his_connect_top_lane_.points.emplace_back(TransformPointVec(top_lane.points.front(), data_manager_->Twb()));
    FLOG_CROSS << "top_lane id: " << top_lane.lane_id;
    return true;//如果找到toplane就不使用最近逻辑
  }
  return FindClosetLaneByLatErr(virtual_lane.line_points);
}

bool CrossToplaneProcessor::FindClosetLaneByTopo(const BevLaneInfo &lane_input) {
  bool is_turn_type = (data_manager_->turn_type() == BevAction::LEFT_TURN || data_manager_->turn_type() == BevAction::RIGHT_TURN ||
                       data_manager_->turn_type() == BevAction::STRAIGHT);
  XLOG << "  FindClosetLaneByTopo  is_turn_type: " << is_turn_type;
  // 左右转使用新的逻辑
  if (!is_turn_type) {
    XLOG << "is_turn_type: " << is_turn_type;
    return false;
  }
  auto &      top_lanes      = data_manager_->GetTopLanes();
  const auto &ego_road_lanes = data_manager_->GetEgoRoadLanes();
  int         top_lanes_size = top_lanes.size();
  XLOG << "top_lanes_size: " << top_lanes.size();
  if (top_lanes.empty()) {
    return false;
  }
  // 只有一个top lane，直接连接
  // if (top_lanes_size == 1) {
  //   if(top_lanes.front().points.empty()){
  //     return false;
  //   }
  //   auto &top_lane              = data_manager_->GetTopLane();
  //   top_lane                    = top_lanes.front();
  //   his_connect_top_lane_index_ = 0;
  //   return true;
  // }
  // 计算横向偏差
  std::vector<Eigen::Vector2f> connect_line_geos;
  // connect_line_geos.reserve(lane_input.line_points.size());
  for (const auto &point : lane_input.line_points) {
    const auto &pt = TransformPoint(point, data_manager_->Tbw());
    if (pt.x < 0.0) {
      continue;
    }
    // FLOG_CROSS << "connect_line_geos_x: " << pt.x << " ,pt.y: " << pt.y;
    connect_line_geos.emplace_back(Eigen::Vector2f(pt.x, pt.y));
  }

  if (connect_line_geos.size() < 2) {
    XLOG << "connect_line_geos";
    return false;
  }

  std::vector<double> top_lat_errs;
  for (int i = 0; i < top_lanes_size; i++) {
    const auto &                 top_lane = top_lanes[i];
    std::vector<Eigen::Vector2f> top_lane_points;
    for (const auto &point : top_lane.points) {
      XLOG << "top_lane_points_x: " << point.x() << " ,pt.y: " << point.y();
      top_lane_points.emplace_back(Eigen::Vector2f(point.x(), point.y()));
    }
    if (top_lane_points.size() < 2) {
      top_lat_errs.emplace_back(100.0);
      XLOG << " top_lane_points.empty!!!";
      continue;
    }
    double      err         = GetDistBetweenTwoLaneOverlap(connect_line_geos, top_lane_points);
    const auto &con_point_1 = connect_line_geos[connect_line_geos.size() - 2];
    const auto &con_point_2 = connect_line_geos[connect_line_geos.size() - 1];
    if (err > 99.0 &&
        (data_manager_->GetCrossState() == CrossState::CONNECTION || data_manager_->GetCrossState() == CrossState::PREDICTION) &&
        std::hypot(top_lane_points.front().x() - con_point_2.x(), top_lane_points.front().y() - con_point_2.y()) < 10.0) {
      err = GetDistPointLane(top_lane_points.front(), con_point_1, con_point_2);
    }
    XLOG << " top_lane_index: " << i << " ,err: " << err << " ,is_connect_lane: " << top_lane.is_connect_lane
               << " ,lane_id: " << top_lane.lane_id;
    top_lat_errs.emplace_back(err);
  }
  XLOG << " find top_lane_index: ";
  // 当自车道只有一个时，选择横向偏差最小的车道
  // 暂时选择最近的lane
  if (ego_road_lanes.is_one_lane || data_manager_->GetVirtualLineSource() == VirtualLineSource::LD ||
      data_manager_->GetVirtualLineSource() == VirtualLineSource::EXP) {
    int    index{0};
    double min_err = std::numeric_limits<double>::max();
    for (int i = 0; i < top_lat_errs.size(); i++) {
      if (top_lat_errs[i] < min_err) {
        min_err = top_lat_errs[i];
        index   = i;
      }
    }
    double max_err =
        (data_manager_->GetVirtualLineSource() == VirtualLineSource::LD || data_manager_->GetVirtualLineSource() == VirtualLineSource::EXP)
            ? 1.8
            : 10.0;
    if (min_err > max_err) {
      XLOG << " find top_lane_index: " << index << " ,min_err: " << min_err;
      return false;
    }
    auto &top_lane              = data_manager_->GetTopLane();
    top_lane                    = top_lanes[index];
    his_connect_top_lane_index_ = index;
    XLOG << " find top_lane_index: " << index << " ,is_connect_lane: " << top_lane.is_connect_lane;
    return true;
  }
  XLOG << " find top_lane_index: top_lat_errs: " << top_lat_errs.size();
  int top_lane_index{0};
  if (data_manager_->turn_type() == BevAction::STRAIGHT) {
    int max_index = top_lanes_size - ego_road_lanes.right_same_turn_size - 1;
    int min_index = ego_road_lanes.left_same_turn_size;
    XLOG << " max_index: " << max_index << " ,min_index: " << min_index;
    if (ego_road_lanes.left_same_turn_size == 0) {

      if (max_index <= 0) {
        top_lane_index = 0;
      } else {
        int    index{0};
        double min_err = std::numeric_limits<double>::max();
        for (int i = 0; i <= max_index; i++) {
          if (top_lat_errs[i] < min_err) {
            min_err = top_lat_errs[i];
            index   = i;
          }
        }
        top_lane_index = index;
      }
    } else if (ego_road_lanes.right_same_turn_size == 0) {
      if (min_index >= top_lanes_size - 1) {
        top_lane_index = top_lanes_size - 1;
      } else {
        int    index{0};
        double min_err = std::numeric_limits<double>::max();
        for (int i = min_index; i < top_lanes_size; i++) {
          if (top_lat_errs[i] < min_err) {
            min_err = top_lat_errs[i];
            index   = i;
          }
        }
        top_lane_index = index;
      }
    } else {
      int    index{0};
      double min_err = std::numeric_limits<double>::max();
      if (max_index < 0 || min_index > top_lanes_size - 1 || min_index > max_index) {
        for (int i = 0; i < top_lanes_size; i++) {
          if (top_lat_errs[i] < min_err) {
            min_err = top_lat_errs[i];
            index   = i;
          }
        }
      } else {
        for (int i = min_index; i <= max_index; i++) {
          if (top_lat_errs[i] < min_err) {
            min_err = top_lat_errs[i];
            index   = i;
          }
        }
      }
      top_lane_index = index;
    }
  } else if (data_manager_->turn_type() == BevAction::LEFT_TURN) {
    if (ego_road_lanes.left_same_turn_size == 0) {
      top_lane_index = 0;
    } else if (ego_road_lanes.right_same_turn_size == 0) {
      //左侧预留车道后剩下的选择偏差最小的
      if (top_lanes_size <= ego_road_lanes.left_same_turn_size + 1) {
        top_lane_index = top_lanes_size - 1;
      } else {
        int    index{0};
        double min_err = std::numeric_limits<double>::max();
        for (int i = ego_road_lanes.left_same_turn_size; i < top_lat_errs.size(); i++) {
          if (top_lat_errs[i] < min_err) {
            min_err = top_lat_errs[i];
            index   = i;
          }
        }
        top_lane_index = index;
      }
    } else {
      top_lane_index = ego_road_lanes.left_same_turn_size;
    }
  } else if (data_manager_->turn_type() == BevAction::RIGHT_TURN) {
    if (ego_road_lanes.right_same_turn_size == 0) {
      top_lane_index = top_lanes_size - 1;
    } else if (ego_road_lanes.left_same_turn_size == 0) {
      //右侧预留车道后剩下的选择偏差最小的
      if (top_lanes_size <= ego_road_lanes.right_same_turn_size + 1) {
        top_lane_index = 0;
      } else {
        int    index{0};
        double min_err = std::numeric_limits<double>::max();
        for (int i = 0; i < top_lat_errs.size() - ego_road_lanes.right_same_turn_size; i++) {
          if (top_lat_errs[i] < min_err) {
            min_err = top_lat_errs[i];
            index   = i;
          }
        }
        top_lane_index = index;
      }
    } else {
      top_lane_index = top_lanes_size - ego_road_lanes.right_same_turn_size - 1;
    }
  }
  auto &top_lane = data_manager_->GetTopLane();
  XLOG << " top_lane_index: " << top_lane_index;
  top_lane                    = top_lanes[top_lane_index];
  his_connect_top_lane_index_ = top_lane_index;
  XLOG << "FindClosetLaneByTopo end!";
  return true;
}

bool CrossToplaneProcessor::FindClosetLaneByLatErr(const std::vector<cem::message::common::Point2DF> &points) {
  const auto &top_lanes = data_manager_->GetTopLanes();
  bool is_lrturn = (data_manager_->turn_type() == BevAction::LEFT_TURN || data_manager_->turn_type() == BevAction::RIGHT_TURN);
  // 左右转使用新的逻辑
  if (is_lrturn) {
    // return false;
  }
  XLOG << "FindClosetLaneByLatErr enter";
  if (!opening_.is_valid || opening_.lane_ids.size() < 1 || !KEnableFindClosetLaneByLatErr) {
    XLOG << "opening_.is_valid: " << opening_.is_valid << " ,lane_ids.size: " << opening_.lane_ids.size();
    XLOG << "FindClosetLaneByLatErr enter1";
    return false;
  }
  bool   is_turn_type = (data_manager_->turn_type() == BevAction::LEFT_TURN || data_manager_->turn_type() == BevAction::RIGHT_TURN ||
                       data_manager_->turn_type() == BevAction::U_TURN);
  double min_dis      = is_turn_type ? 20 : 40;
  auto   top_dis      = opening_.dis / (std::abs(std::cos(opening_.heading)) + 0.0001);
  // if (top_dis > min_dis || (is_turn_type && std::abs(opening_.heading) > 0.2)) {//top_dis 来自j交叉点
  if ( (is_turn_type && std::abs(opening_.heading) > 0.2)) {
    XLOG << "top_dis: " << top_dis << " opening_.dis " << opening_.dis;
    XLOG << "FindClosetLane enter2:" << std::abs(opening_.heading);
    return false;
  }
  std::vector<cem::message::common::Point2DF> line_points;
  line_points.reserve(points.size());
  for (const auto &point : points) {
    line_points.emplace_back(TransformPoint(point, data_manager_->Tbw()));
  }

  if (line_points.empty()) {
    XLOG << "line_points.empty!!!";
    XLOG << "FindClosetLane enter3";
    return false;
  }

  // 只使用自车前方的点进行比较
std::vector<Eigen::Vector2f> connect_line_geos;
// 先转换再筛选
std::vector<Eigen::Vector2f> temp_geos;
std::transform(line_points.begin(), line_points.end(),
               std::back_inserter(temp_geos),
               [](const cem::message::common::Point2DF& point) {
                   return Eigen::Vector2f(point.x, point.y);
               });

// 筛选大于0的点
std::copy_if(temp_geos.begin(), temp_geos.end(),
             std::back_inserter(connect_line_geos),
             [](const Eigen::Vector2f& point) {
                 return point.x() > 0.0;
             });
   // 关键修复：检查connect_line_geos大小是否足够
  if (connect_line_geos.size() < 2) {  // 至少需要2个点
    XLOG << "connect_line_geos size too small: " << connect_line_geos.size();
    return false;
  }
  std::set<uint32_t> top_lane_ids_set;
  for (auto &top_lane_id : top_lane_ids_) {
    top_lane_ids_set.insert(top_lane_id);
  }
  //
  double min_dist = std::numeric_limits<double>::max();
  for (auto &lane : data_manager_->bev_map_raw()->lane_infos) {
    if (!lane.id)
      continue;
    if (top_lane_ids_set.find(lane.id) != top_lane_ids_set.end() && lane.geos && !lane.geos->empty()) {
      // double dis = LaneGeometry::GetDistanceBetweenLines(connect_line_geos, *lane.geos);
      float dis = GetDistBetweenTwoLaneOverlap(connect_line_geos, *lane.geos);
      const auto& con_point_1 = connect_line_geos[connect_line_geos.size() - 2];
      const auto& con_point_2 = connect_line_geos[connect_line_geos.size() - 1];
      if (dis > 99.0 && data_manager_->GetCrossState() == CrossState::CONNECTION &&
          std::hypot(lane.geos->front().x() - con_point_2.x(), lane.geos->front().y() - con_point_2.y()) < 10.0) {
        dis = GetDistPointLane(lane.geos->front(), con_point_1, con_point_2);
      }
      XLOG << " lane.id: " << lane.id << " ,dis_overlap: " << dis;
      if (dis < min_dist) {
        min_dist         = dis;
        connect_lane_id_ = lane.id;
      }
    }
  }
  XLOG << " connect_lane_id: " << connect_lane_id_ << " ,min_dist: " << min_dist;

  // if (connect_lane_id_ && min_dist < 1.8 && data_manager_->GetLaneById(connect_lane_id_) != nullptr ) {  //最近的距离小于2m
  if (connect_lane_id_  && data_manager_->GetLaneById(connect_lane_id_) != nullptr ) {  //CNOAC2-105566  非对称直行去掉1.8限制
    auto *lane = data_manager_->GetLaneById(connect_lane_id_);
    XLOG << " CONNECT lane id:" << lane->id << " ,front_point_x: " << lane->line_points.front().x
               << " ,y: " << lane->line_points.front().y;
    byd::common::math::Vec2d top_point;
    top_point.set_x(lane->line_points.front().x);
    top_point.set_y(lane->line_points.front().y);
    double half_width = std::max(std::min(lane->width / 2, 2.0f), 1.5f);
    GenerateTopLanePoints(top_point, opening_.heading, half_width, true, lane->id);
  } else {
    XLOG << "has no find ClosetLane!";
    return false;
  }
  XLOG << "FindClosetLane end";
  return true;
}

bool CrossToplaneProcessor::FindClosetLane() {
  AINFO << "FindClosetLane top_lane_ids_size: " << top_lane_ids_.size();
  if (!opening_.is_valid || opening_.lane_ids.size() < 1) {
    AINFO << "opening_.is_valid " << opening_.is_valid;
    return false;
  }
  auto& main_lane = data_manager_->GetMainLane();
  auto& top_lane = data_manager_->GetTopLane();
  // 遍历边界找到角度最小的值
  // if ((opening_.left_y - opening_.right_y) < 4.0 && !top_lane.is_connect_lane) {
  //   GenerateTopLanePoints(opening_.sd_cross_top_point, opening_.heading, 1.75, false);
  // } else 
  {
    double min_heading_err{100.0};
    double optimal_y{0.0};
    double start_y = opening_.left_y - 1.75;
    // 角度偏差过小使用
    if(std::abs(opening_.heading - main_lane.angle) < 0.1){
      optimal_y = main_lane.top_points.y() +
                  (opening_.sd_cross_top_point.x() - main_lane.top_points.x()) * std::sin(main_lane.angle);
    }else if(opening_.has_left_edge || opening_.has_right_edge){
      int index = 0;
      FLOG_CROSS << "main_lane.top_points.x: " << main_lane.top_points.x() << " ,y:" << main_lane.top_points.y();
      while (start_y > opening_.right_y + 1.75 && index < 100) {
        double heading = byd::common::math::Vec2d(opening_.sd_cross_top_point.x() - main_lane.top_points.x(),
                                                  start_y - main_lane.top_points.y())
                             .Angle();
        double err = std::abs(heading - main_lane.angle) + std::abs(opening_.heading - heading);
        // AINFO << " #####start_y: " << start_y << " , main_lane.angle: " << main_lane.angle << " ,heading: " << heading
        //            << " ,opening_.heading: " << opening_.heading;
        if (err < min_heading_err) {
          min_heading_err = err;
          optimal_y = start_y;
        }
        start_y -= 1.0;
        index++;
      }
    }else{
      return false;
    }

    FLOG_CROSS << "close lane x: " << opening_.sd_cross_top_point.x() << " optimal_y:" << optimal_y;
    // 判断optimal_y附近是否有lane，有的话就连接，没有就虚拟
    auto it = std::min_element(top_lane_ids_.begin(), top_lane_ids_.end(), [=](uint64_t a, uint64_t b) {
      return std::abs(lane_id_dis_[a] - optimal_y) < std::abs(lane_id_dis_[b] - optimal_y);
    });
    if (it != top_lane_ids_.end() && std::abs(data_manager_->GetLaneById(*it)->line_points.front().y - optimal_y) < 1.75 && fabs(data_manager_->GetLaneById(*it)->line_points.front().y)<1.75) {
      auto *lane = data_manager_->GetLaneById(*it);
      AINFO << " !!!!CONNECT lane id:" << lane->id << " ,front_point_x: " << lane->line_points.front().x
                 << " ,y: " << lane->line_points.front().y;
      byd::common::math::Vec2d top_point;
      top_point.set_x(lane->line_points.front().x);
      top_point.set_y(lane->line_points.front().y);
      double half_width = std::max(std::min(lane->width / 2, 2.0f), 1.5f);
      GenerateTopLanePoints(top_point, opening_.heading, half_width, true, lane->id);
    } else if (!top_lane.is_connect_lane) {
      //GenerateTopLanePoints(byd::common::math::Vec2d(opening_.sd_cross_top_point.x(), optimal_y), opening_.heading, 1.75, false);
      AINFO << "top_lane.is_connect_lane " << top_lane.is_connect_lane;
      return false;
    } else {
      return false;
    }
  }
  AINFO << "FindClosetLane end";
  return true;
}

void CrossToplaneProcessor::GenerateTopLanePoints(const byd::common::math::Vec2d& point, double heading,
                                                  double half_width, bool is_connect_lane, uint64_t lane_id) {
  auto& top_lane = data_manager_->GetTopLane();
  top_lane.Reset();
  top_lane.lane_id = lane_id;
  top_lane.is_connect_lane = is_connect_lane;
  top_lane.valid = true;
  double x = point.x() - point.x() * 0.33;
  double y = point.y() - point.x() * 0.33 * std::tan(heading);
  top_lane.points.emplace_back(byd::common::math::Vec2d(x, y));
  top_lane.points.emplace_back(point);
  // 生成左边点
  byd::common::math::Vec2d axis = byd::common::math::Vec2d::CreateUnitVec2d(heading);
  axis.SelfRotate(pi / 2);
  top_lane.left_points = TranslateSegmentAlongAxisStart(axis, top_lane.points, half_width);
  // 生成右边点
  top_lane.right_points = TranslateSegmentAlongAxisStart(axis, top_lane.points, -half_width);
  FLOG_CROSS << "GenerateTopLanePoints heading:" << heading << " half_width: " << half_width
             << " ,top_lane point_1_x: " << top_lane.points.front().x() << " ,y: " << top_lane.points.front().y()
             << " top_lane point_2_x: " << top_lane.points.back().x() << " ,y: " << top_lane.points.back().y();
}

bool CrossToplaneProcessor::UpdateClosetLane() {
  AINFO << "UpdateClosetLane ";
  if (!opening_.is_valid) {
    AINFO << "opening_.is_valid " << opening_.is_valid ;
    return false;
  }

  FLOG_CROSS << "ReFindClosetLane!";
  return FindClosetLane();
}

bool CrossToplaneProcessor::UpdateClosetLane(const BevLaneInfo &connect_lane) {
  XLOG << "UpdateClosetLane is_find_closetlane_bytopo_success: " << data_manager_->is_find_closetlane_bytopo_success()
             << " ,is_connect_lane: " << his_connect_top_lane_.is_connect_lane;
  // 第一次进入连接，如果是根据匹配进来的，后面就一直走匹配，如果不是就走原来的不再走匹配
  if (data_manager_->is_find_closetlane_bytopo_success()) {
    auto &top_lanes = data_manager_->GetTopLanes();
    auto &top_lane  = data_manager_->GetTopLane();
    if (top_lanes.empty()) {
      return false;
    }
    if (!data_manager_->GetIsInCrossroad() || data_manager_->GetVirtualLineSource() == VirtualLineSource::LD ||
        data_manager_->GetVirtualLineSource() == VirtualLineSource::EXP) {
      return FindClosetLaneByTopo(connect_lane);
    }

    // 按照index找
    if (his_connect_top_lane_index_ > top_lanes.size() - 1) {
      auto his_points = TransformPointVec(his_connect_top_lane_.points.front(), data_manager_->Tbw());
      // is_connect_lane找不到，再次寻找最近点
      if (his_connect_top_lane_.is_connect_lane) {
        for (const auto &top : top_lanes) {
          if (top.lane_id == his_connect_top_lane_.lane_id && !top_lane.points.empty()) {
            const auto &points = top.points;
            if (points.size() > 1) {
              auto dis = GetDistPointLane(his_points, points[0], points[1]);
              if (dis < 3.0) {
                top_lane              = top;
                his_connect_top_lane_ = top_lane;
                his_connect_top_lane_.points.clear();
                his_connect_top_lane_.points.emplace_back(TransformPointVec(top_lane.points.front(), data_manager_->Twb()));
                return true;
              }
            }
          }
        }
      }

      int    top_index{0};
      double min_dis = std::numeric_limits<double>::max();
      for (int i = 0; i < top_lanes.size(); i++) {
        const auto &points = top_lanes[i].points;
        if (points.size() < 2) {
          continue;
        }
        auto dis = GetDistPointLane(his_points, points[0], points[1]);
        FLOG_CROSS << " index: " << i << " ,dis: " << dis << " ,is_connect_lane: " << top_lane.is_connect_lane
                   << " ,lane_id: " << top_lane.lane_id;
        if (dis < min_dis) {
          min_dis   = dis;
          top_index = i;
        }
      }
      if (min_dis < 3.0) {
        top_lane              = top_lanes[top_index];
        his_connect_top_lane_ = top_lane;
        his_connect_top_lane_.points.clear();
        his_connect_top_lane_.points.emplace_back(TransformPointVec(top_lane.points.front(), data_manager_->Twb()));
        his_connect_top_lane_index_ = top_index;
        FLOG_CROSS << " select index: " << top_index << " ,min_dis: " << min_dis << " ,is_connect_lane: " << top_lane.is_connect_lane
                   << " ,id: " << top_lane.lane_id;
        return true;
      }
    } else {
      FLOG_CROSS << " IsInCrossroad  his_connect_top_lane_index: " << his_connect_top_lane_index_;
      top_lane = top_lanes[his_connect_top_lane_index_];
      return true;
    }

    return false;
  }

  if (!opening_.is_valid) {
    FLOG_CROSS << "opening_.is_valid ";
    return false;
  }
  //   如果未连接到lane上，就重新找
  // 如果已经连接到lane上，就使用相应的lane进行更新
  bool  has_connect_lane{false};
  auto &top_lane = data_manager_->GetTopLane();
  if (top_lane.is_connect_lane) {
    auto  lane_id = top_lane.lane_id;
    auto *lane    = data_manager_->GetLaneById(lane_id);
    if (lane != nullptr && !lane->line_points.empty()) {
      FLOG_CROSS << "lane_points.empty ";
      has_connect_lane = true;
    }

    if (has_connect_lane) {
      double heading    = opening_.heading;
      double half_width = std::max(std::min(lane->width / 2, 2.0f), 1.5f);
      FLOG_CROSS << " CONNECT lane id:" << lane_id << " ,front_point_x: " << lane->line_points.front().x
                 << " ,y: " << lane->line_points.front().y;
      GenerateTopLanePoints(byd::common::math::Vec2d(lane->line_points.front().x, lane->line_points.front().y), opening_.heading,
                            half_width, true, lane_id);
      return true;
    }
  }
  FLOG_CROSS << "ReFindClosetLane!";
  return FindClosetLaneByLatErr(connect_lane.line_points);
}

bool CrossToplaneProcessor::GetAverageHeading(const std::vector<std::tuple<double, double, double>> &heading_infos,
                                              double &                                               average_heading) {
  double heading_sum{0.0};
  int    heading_num{0};
  bool   has_intersection_zone       = data_manager_->GetSdSections().has_intersection_zone;
  double max_interior_points_percent = has_intersection_zone ? 0.9 : 0.7;
  for (const auto &heading_info : heading_infos) {
    // FLOG_CROSS << "heading_info has average heading: " << std::get<0>(heading_info) << ",err: " << std::get<1>(heading_info)
    //            << " ,percep: " << std::get<2>(heading_info);
    if (std::get<2>(heading_info) > max_interior_points_percent && std::get<1>(heading_info) < 0.2) {
      heading_sum += std::get<0>(heading_info);
      heading_num++;
    }
  }

  bool   has_average_heading{false};
  double heading{0.0};
  if (heading_num > 0) {
    has_average_heading = true;
    heading             = heading_sum / heading_num;
  }
  // auto heading_it = std::min_element(heading_infos.begin(), heading_infos.end(),
  //                                    [](const std::tuple<double, double, double> &a, const std::tuple<double, double, double> &b) {
  //                                      return std::get<1>(a) < std::get<1>(b);
  //                                    });
  // if (heading_it != heading_infos.end()) {
  //   FLOG_CROSS << " average heading: " << std::get<0>(*heading_it) << " ,average_point_err: " << std::get<1>(*heading_it)
  //              << " ,interior_points_percent: " << std::get<2>(*heading_it);
  // } else {
  //   FLOG_CROSS << " has no average heading!!!";
  // }
  FLOG_CROSS << " ,has_intersection_zone: " << data_manager_->GetSdSections().has_intersection_zone
             << " ,his_average_heading: " << his_average_heading_ << " ,has_average_heading: " << has_average_heading
             << " ,average_heading: " << heading;
  // bool has_average_heading =
  //     heading_it != heading_infos.end() && std::get<2>(*heading_it) > 0.9 && std::abs(std::get<0>(*heading_it)) < 1.0;
  bool has_his_average_heading   = !IsZero(his_average_heading_);
  float delta_heading  = data_manager_->GetLocDeltaHeading();
  if (has_his_average_heading) {
    his_average_heading_ = average_heading_filter_.Last() - delta_heading;
  }
  FLOG_CROSS << " his_average_heading: " << his_average_heading_  << " ,top_angle: " << data_manager_->GetSdSections().top_angle();
  if (has_average_heading && has_intersection_zone && has_his_average_heading) {
    // double heading = std::get<0>(*heading_it);
    double angle   = data_manager_->GetSdSections().top_angle();
    if (std::abs(heading - angle) < 0.5) {
      average_heading = heading;
    } else {
      average_heading = his_average_heading_;
    }
  } else if (has_average_heading && has_intersection_zone) {
    // double heading = std::get<0>(*heading_it);
    double angle   = data_manager_->GetSdSections().top_angle();
    if (std::abs(heading - angle) < 0.5) {
      average_heading = heading;
    } else {
      return false;
    }
  } else if (has_average_heading && has_his_average_heading) {
    // double heading = std::get<0>(*heading_it);
    if (std::abs(heading - his_average_heading_) < 0.2) {
      average_heading = heading;
    } else {
      average_heading = his_average_heading_;
    }
  } else if (has_average_heading) {
    // double heading = std::get<0>(*heading_it);
    if (heading < 0.8) {
      average_heading = heading;
    } else {
      return false;
    }
  } else if (has_his_average_heading && has_intersection_zone) {
    double angle    = data_manager_->GetSdSections().top_angle();
    average_heading = 0.5 * his_average_heading_ + 0.5 * angle;
  } else if (has_intersection_zone) {
    average_heading = data_manager_->GetSdSections().top_angle();
  } else if (has_his_average_heading) {
    average_heading = his_average_heading_;
  } else {
    return false;
  }
  FLOG_CROSS << " before filter angle: " << average_heading << " ,delta_heading: " << delta_heading;
  average_heading = average_heading_filter_.Filter(average_heading, delta_heading);
  FLOG_CROSS << " after filter angle: " << average_heading;
  return true;
  // if (heading_it != heading_infos.end() && std::get<2>(*heading_it) > 0.8 && std::abs(std::get<0>(*heading_it)) < 1.0) {
  //   // 如果有历史值或者sd的angle，需要对比后再使用
  //   if(data_manager_->GetSdSections().has_intersection_zone){

  //   }
  //   average_heading = std::get<0>(*heading_it);
  // } else if (data_manager_->GetSdSections().has_intersection_zone) {
  //   average_heading = data_manager_->GetSdSections().top_angle();
  // } else if (!IsZero(his_average_heading_)) {
  //   average_heading = his_average_heading_ - data_manager_->GetLocDeltaHeading();
  // } else {
  //   his_average_heading_ = 0.0;
  //   FLOG_CROSS << " has no top heading!!! ";
  //   return;
  // }
}

byd::common::math::Vec2d CrossToplaneProcessor::TransformPointVec(const byd::common::math::Vec2d &point,
                                                                  const Eigen::Isometry3d &       rotate_translate_matrix) {
  byd::common::math::Vec2d point_out;
  Eigen::Vector3d          point_before_dr(point.x(), point.y(), 0);
  Eigen::Vector3d          point_after_dr = rotate_translate_matrix * point_before_dr;
  point_out.set_x(point_after_dr[0]);
  point_out.set_y(point_after_dr[1]);
  return point_out;
}

}  // namespace fusion
}  // namespace cem