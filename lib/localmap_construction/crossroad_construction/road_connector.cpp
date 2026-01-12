#include "road_connector.h"

#include "lib/common/fitter/bezier_points.h"

namespace cem {
namespace fusion {

RoadConnector::RoadConnector() {}

void RoadConnector::Init(const std::shared_ptr<CrossDataManager>& data_manager) { data_manager_ = data_manager; }
void RoadConnector::Reset() {
  dr_ego_lane_ = {};
  connect_lane_ = {};
  connect_left_lanemarker_ = {};
  connect_right_lanemarker_ = {};
}
bool RoadConnector::GenerateConnectLane() {
  AINFO<< "### TRY GenerateConnectLane ";
  auto& main_lane = data_manager_->GetMainLane();
  auto& top_lane = data_manager_->GetTopLane();
  BevLaneInfo connect_lane;
  BevLaneMarker connect_left_lanemarker;
  BevLaneMarker connect_right_lanemarker;
  // 连接中心线
  bool is_lane_connect_valid{false};
  int bot_lane_index = -1;
  std::vector<byd::common::math::Vec2d> control_points = {};
  int top_index = -1;
  std::vector<byd::common::math::Vec2d> mian_points;
  mian_points.emplace_back(main_lane.top_points);
  double x = main_lane.top_points.x() + top_lane.points.back().x()*0.36;
  double y = main_lane.top_points.y() + top_lane.points.back().x()*0.36 * std::tan(main_lane.angle);
  mian_points.emplace_back(byd::common::math::Vec2d(x, y));
  if(main_lane.points.back().DistanceTo(top_lane.points.back()) < 3.0){
    return false;
  }
  FLOG_CROSS << " main_lane.pointsback_x: " << main_lane.points.back().x() << " ,y: " << main_lane.points.back().y();
  FLOG_CROSS << " top_lane.pointsback_x: " << top_lane.points.back().x() << " ,y: " << top_lane.points.back().y();
  if (InsertBezierPoints(mian_points, top_lane.points, control_points, bot_lane_index, top_index)) {
    // FLOG_CROSS << " GenerateConnectLane bot_lane_index: " << bot_lane_index << " , top_index: " << top_index
    //            << " ,control_points.size: " << control_points.size();
    if (bot_lane_index >= 0 && top_index >= 0 && control_points.size() > 0) {
      // 需要判断main_lane用于连接的线是原始车道线还是virtual线，判断是否需要使用
      FLOG_CROSS << " control_pointsfront_x: " << control_points.front().x() << " ,y: " <<control_points.front().y();
      if (main_lane.is_virtual) {
        control_points.insert(control_points.begin(), main_lane.points.begin(), main_lane.points.end() - 1);
      }
      connect_lane.id = kConnectLaneId;
      // connect_lane.id                   = data_manager_->GetNewLandId();
      connect_lane.left_lane_marker_id  = kConnectLeftLanemarkerId;
      connect_lane.right_lane_marker_id = kConnectRightLanemarkerId;
      connect_lane.previous_lane_ids.emplace_back(main_lane.lane_id);
      connect_lane.is_virtual  = true;
      connect_lane.navi_action = main_lane.navi_action;
      bool has_ego_pos         = false;
      for (const auto &lane : data_manager_->bev_map()->lane_infos) {
        if (lane.position == 0) {
          has_ego_pos = true;
          break;
        }
      }
      if (has_ego_pos) {
        connect_lane.position = 7;
      }
      if (top_lane.is_connect_lane) {
        connect_lane.next_lane_ids.emplace_back(top_lane.lane_id);
      }
      connect_lane.line_points.clear();
      connect_lane.line_points.reserve(control_points.size());
      for (auto &pt : control_points) {
        cem::message::common::Point2DF point;
        point.x   = pt.x();
        point.y   = pt.y();
        point.mse = NAN;
        // FLOG_CROSS << " GenerateConnectLane x: " << point.x << " , y: " << point.y;
        point.point_source = cem::message::common::PointSource::TOPO_INSETRT;
        connect_lane.line_points.emplace_back(TransformPoint(point, data_manager_->Twb()));
      }
      //
      is_lane_connect_valid = true;
    }
  }

  // 连接左边车道线
  bool is_left_connect_valid{false};
  bot_lane_index = -1;
  top_index = -1;
  control_points.clear();
  if (InsertBezierPoints(main_lane.left_points, top_lane.left_points, control_points, bot_lane_index, top_index)) {
    if (bot_lane_index >= 0 && top_index >= 0 && control_points.size() > 0) {
      // 需要判断main_lane用于连接的线是原始车道线还是virtual线，判断是否需要使用
      if (main_lane.is_virtual) {
        control_points.insert(control_points.begin(), main_lane.left_points.begin(), main_lane.left_points.end());
      }
      connect_left_lanemarker.id = kConnectLeftLanemarkerId;
      connect_left_lanemarker.is_virtual = true;

      connect_left_lanemarker.line_points.clear();
      connect_left_lanemarker.line_points.reserve(control_points.size());
      for (auto& pt : control_points) {
        cem::message::common::Point2DF point;
        point.x = pt.x();
        point.y = pt.y();
        point.mse = NAN;
        point.point_source = cem::message::common::PointSource::TOPO_INSETRT;
        connect_left_lanemarker.line_points.emplace_back(TransformPoint(point,data_manager_->Twb()));
      }
      //
      is_left_connect_valid = true;
    }
  }
  //连接右边车道线
  bool is_right_connect_valid{false};
  bot_lane_index = -1;
  top_index = -1;
  control_points.clear();
  if (InsertBezierPoints(main_lane.right_points, top_lane.right_points, control_points, bot_lane_index, top_index)) {
    if (bot_lane_index >= 0 && top_index >= 0 && control_points.size() > 0) {
      // 需要判断main_lane用于连接的线是原始车道线还是virtual线，判断是否需要使用
      if (main_lane.is_virtual) {
        control_points.insert(control_points.begin(), main_lane.right_points.begin(), main_lane.right_points.end());
      }
      connect_right_lanemarker.id = kConnectRightLanemarkerId;
      connect_right_lanemarker.is_virtual = true;

      connect_right_lanemarker.line_points.clear();
      connect_right_lanemarker.line_points.reserve(control_points.size());
      for (auto& pt : control_points) {
        cem::message::common::Point2DF point;
        point.x = pt.x();
        point.y = pt.y();
        point.mse = NAN;
        point.point_source = cem::message::common::PointSource::TOPO_INSETRT;

        connect_right_lanemarker.line_points.emplace_back(TransformPoint(point,data_manager_->Twb()));
      }
      //
      is_right_connect_valid = true;
    }
  }

  if (is_lane_connect_valid) {
    connect_lane_ = connect_lane;
    // connect_left_lanemarker_ = connect_left_lanemarker;
    // connect_right_lanemarker_ = connect_right_lanemarker;
    FLOG_CROSS << "GenerateConnectLane success lane size: " << connect_lane_.line_points.size();
    return true;
  }
  AINFO << "!!!!!GenerateConnectLane faild is_lane_connect_valid: " << is_lane_connect_valid
             << " ,is_left_connect_valid: " << is_left_connect_valid
             << " ,is_right_connect_valid: " << is_right_connect_valid;
  return false;
}

bool RoadConnector::GenerateConnectLane(const BevLaneInfo &his_lane){
  FLOG_CROSS << "update GenerateConnectLane Opening().is_valid: " << data_manager_->GetOpening().is_valid
             << " ,his_lane.line_points.size: " << his_lane.line_points.size();
  auto &main_lane = data_manager_->GetMainLane();
  auto &top_lane  = data_manager_->GetTopLane();
  if (top_lane.points.size() < 2 || his_lane.line_points.size() < 2) {
    FLOG_CROSS << "return by top_lane.points.size() < 2";
    return false;
  }
  if (his_lane.line_points.size() > 2) {
    if (main_lane.points.back().DistanceTo(top_lane.points.back()) < 3.0 || main_lane.points.back().x() > top_lane.points.back().x()) {
      if (top_lane.is_connect_lane) {
        if (connect_lane_.next_lane_ids.empty()) {
          connect_lane_.next_lane_ids.emplace_back(top_lane.lane_id);
        } else {
          connect_lane_.next_lane_ids[0] = top_lane.lane_id;
        }
        FLOG_CROSS <<" next_lane_id new_id: " << top_lane.lane_id;
      }
      FLOG_CROSS << "return by Distance";
      return false;
    }
    FLOG_CROSS << "start generate!!!";
     
    // 处理mianlane，到自车后方10个点，不够需要延长补充
    std::vector<Point2DF> history_lane_points;
    history_lane_points.reserve(his_lane.line_points.size());
    for (auto it = his_lane.line_points.begin(); it != his_lane.line_points.end(); it++) {
      history_lane_points.emplace_back(TransformPoint(*it, data_manager_->Tbw()));
    }
    int mainlane_index_end   = 0;
    int mainlane_index_start = 0;
    int using_history_end_index{0};
    for (int i = history_lane_points.size() - 1; i >= 0; i--) {
      if (history_lane_points[i].x < top_lane.points.front().x()) {
        using_history_end_index = i;
        break;
      }
    }
    double min_dis{1000.0};
    for (int i = history_lane_points.size() - 1; i >= 0; i--) {
      double dis = std::hypot(history_lane_points[i].x,history_lane_points[i].y);
      if (dis < min_dis) {
        min_dis = dis;
        mainlane_index_start = i;
      }
    }
    mainlane_index_end = std::min(mainlane_index_start + 20, static_cast<int>(history_lane_points.size() - 1));
    FLOG_CROSS << "mainlane_index_start: " << mainlane_index_start << " ,mainlane_index_end: " << mainlane_index_end
               << " ,main_lane.is_virtual: " << main_lane.is_virtual;
    // for (int i = 0; i < history_lane_points.size();i++) {
    //   FLOG_CROSS << " oringnal history_lane_points[ "<< i  << " ]_x: " << history_lane_points[i].x << " ,y: " << history_lane_points[i].y;
    // }
    std::vector<Point2DF> fit_main_points;
    int using_history_start_index{0};
    if(!main_lane.is_virtual){
      for (int i = 20; i > 0; i--) {
        double x = main_lane.top_points.x() - i;
        double y = main_lane.top_points.y() - i * std::tan(main_lane.angle);
        fit_main_points.emplace_back(x, y);
      }
      fit_main_points.insert(fit_main_points.end(), history_lane_points.begin(), history_lane_points.begin() + mainlane_index_end);
    }else{
      int err_index = mainlane_index_end - mainlane_index_start;
      if(err_index >= 20 || (err_index < 20 && mainlane_index_end - 20 >= 0)){
        using_history_start_index = mainlane_index_start;
        fit_main_points.insert(fit_main_points.end(), history_lane_points.begin()+mainlane_index_start, history_lane_points.begin() + mainlane_index_end);
      }else{
        int index = err_index - 20;
        using_history_start_index = mainlane_index_start;
        cem::message::common::Point2DF start = history_lane_points[mainlane_index_start];
        std::vector<cem::message::common::Point2DF> add_points;
        while (index <= 0) {
          double x = start.x - 1.0;
          double y = start.y - std::tan(main_lane.angle);
          add_points.emplace_back(x, y);
          start = add_points.back();
          index++;
        }
        FLOG_CROSS << " index: " << index << " ,start.x: " << start.x << " ,y: " << start.y;
        for (int i = add_points.size() - 1; i >= 0; i--) {
          fit_main_points.emplace_back(add_points[i]);
        }
        fit_main_points.insert(fit_main_points.end(), history_lane_points.begin(), history_lane_points.begin() + mainlane_index_end);
      }
    }
    FLOG_CROSS << " fit_main_points_size: " << fit_main_points.size();
    // for(const auto& point:fit_main_points){
    //   FLOG_CROSS << " fit_main_points_x: " << point.x << " ,y: " << point.y;
    // }
    
    // 处理toplane
    // 判断toplane在历史连接线的左侧还是右侧以及距离
    Point2DF his_point_one,his_point_two;
    int index = history_lane_points.size() - 1;
    for(int i = 0;i< history_lane_points.size();i++){
      if(history_lane_points[i].x > top_lane.points.front().x()){
        index = i;
        break;
      }
    }
    if(index < history_lane_points.size() - 1){
      his_point_one = history_lane_points[index];
      his_point_two = history_lane_points[index + 1];
    }else{
      his_point_one = history_lane_points[history_lane_points.size() - 2];
      his_point_two = history_lane_points.back();
    }
    auto dis = GetDistPointLane(top_lane.points.front(),his_point_one,his_point_two);
    bool need_move_toplane = false;
    if(dis > 1.0){
      need_move_toplane = true;
      if(PointInVectorSide(his_point_one,his_point_two,top_lane.points.front()) <= 0){ // 左侧，需要往右平移
        dis = -(dis - 0.5);
      }else{
        dis = dis - 0.5;
      }
    }
    // 暂时屏蔽
    need_move_toplane = false;
    std::vector<Point2DF> fit_top_points;
    constexpr int kTopLanePointMaxSize = 20;
    // 先把linepoint 拿出来做下坐标转换
    std::vector<cem::message::common::Point2DF> bev_points;
    auto *bev_top_lane = FindBevLaneById(top_lane.lane_id);
    if (top_lane.is_connect_lane) {
      if (bev_top_lane == nullptr || bev_top_lane->line_points.empty()) {
        FLOG_CROSS << "bev_top_lane == nullptr,and top_id: " << top_lane.lane_id;
        return false;
      }

      bev_points.reserve(bev_top_lane->line_points.size());
      for (auto it = bev_top_lane->line_points.begin(); it != bev_top_lane->line_points.end(); it++) {
        bev_points.emplace_back(TransformPoint(*it, data_manager_->Tbw()));
      }
      FLOG_CROSS << "bev_top_lane_points_size: " << bev_points.size();
      // 增加平移
      if (need_move_toplane) {
        Vec2d axis = top_lane.points.back() - top_lane.points.front();
        axis.Normalize();
        axis.SelfRotate(pi / 2);
        TranslateSegmentAlongAxisStart(axis, bev_points, dis);
      }
      if (bev_points.size() < kTopLanePointMaxSize) {
        fit_top_points = bev_points;
        for (int i = 0; i < (kTopLanePointMaxSize - bev_points.size()); i++) {
          double x = fit_top_points.back().x + 1.0;
          double y = fit_top_points.back().y + std::tan(data_manager_->GetOpening().heading);
          fit_top_points.emplace_back(x, y);
        }
      } else {
        fit_top_points.insert(fit_top_points.begin(), bev_points.begin(), bev_points.begin() + kTopLanePointMaxSize);
      }
    } else {
      // for (const auto &point : top_lane.points) {
      //   FLOG_CROSS << " connect top_lane_points x: " << point.x() << " ,y: " << point.y();
      // }
      std::vector<Vec2d> toplane_points = top_lane.points;
      if (need_move_toplane) {
        Vec2d axis = top_lane.points.back() - top_lane.points.front();
        axis.Normalize();
        axis.SelfRotate(pi / 2);
        toplane_points = TranslateSegmentAlongAxisStart(axis, top_lane.points, dis);
      }
      double heading = (toplane_points[1] - toplane_points[0]).Angle();
      for (const auto &point : toplane_points) {
        fit_top_points.emplace_back(point.x(), point.y());
      }
      for (int i = 0; i < (kTopLanePointMaxSize - toplane_points.size()); i++) {
        double x = fit_top_points.back().x + 1.0;
        double y = fit_top_points.back().y + std::tan(heading);
        fit_top_points.emplace_back(x, y);
      }
    }
    FLOG_CROSS << " fit_top_points_size: " << fit_top_points.size();
    // for (const auto &point : fit_top_points) {
    //   FLOG_CROSS << " fit_top_points_x: " << point.x << " ,y: " << point.y;
    // }
    fit_main_points.insert(fit_main_points.end(), fit_top_points.begin(), fit_top_points.end());
    FLOG_CROSS << " fit_main_points_size: " << fit_main_points.size();
    for (auto it = fit_main_points.begin() + 1; it != fit_main_points.end();) {
      if (it->x < (it - 1)->x) {
        it = fit_main_points.erase(it);
      } else {
        it++;
      }
    }
    FLOG_CROSS << " cut fit_main_points_size: " << fit_main_points.size();
    cem::message::env_model::Curve curve;
    std::vector<cem::message::common::Point2DF> connect_line_point;
    if (connect_lane_.line_points.empty()){
      FLOG_CROSS << " FitterLeastSquareMethod faild,and using bazel";
      return GenerateConnectLane();
    }else if(FitterLeastSquareMethod(fit_main_points, 3, curve)) {
      FLOG_CROSS << " FitterLeastSquareMethod success using_history_start_index: " << using_history_start_index
      << " ,using_history_end_index: " << using_history_end_index;
      // 判断历史线是否需要裁剪掉
      if (history_lane_points.size() >= using_history_end_index) {
        history_lane_points.resize(using_history_end_index + 1);
      }
      // 生成新的连接线
      double x{0.0};
      double x2{0.0};
      double x3{0.0};
      double new_y{0.0};
      double y{0.0};
      double ori_k = 1.0 / std::max(using_history_end_index - using_history_start_index, 1);
      for(int i = 0;i <= using_history_end_index;i++){
        if(i <= using_history_start_index){
          continue;
        }
        x = history_lane_points[i].x;
        x2 = x * x;
        x3 = x * x2;
        new_y = curve.c0 + curve.c1 * x + curve.c2 * x2 + curve.c3 * x3;
        double k = std::min(ori_k * ( i - using_history_start_index),1.0);
        double y = new_y * k +  history_lane_points[i].y * (1 - k);
        // FLOG_CROSS << " i: " << i << " ,k: " << k;
        history_lane_points[i].y = y;
        // history_lane_points[i].y = new_y;
      }

      // 修改原始路口后车道线
      if (top_lane.is_connect_lane) {
        int bev_index = std::min(static_cast<int>(bev_points.size()), kTopLanePointMaxSize);
        ori_k         = std::min(1.0 / bev_index,0.1);
        for (int i = 0; i < bev_index; i++) {
          x        = bev_points[i].x;
          x2       = x * x;
          x3       = x * x2;
          new_y    = curve.c0 + curve.c1 * x + curve.c2 * x2 + curve.c3 * x3;
          double k = (bev_index - i) * ori_k;
          double y = new_y * k + bev_points[i].y * (1 - k);
          // FLOG_CROSS << " out bev_points index: " << bev_index << " ,i:" << i << " ,old_y: " << bev_points[i].y << " ,end_y: " << y
          //            << ",new_y: " << new_y << " ,orin_k:" << ori_k;
          bev_points[i].y = y;
          // bev_points[i].y = new_y;
        }
      } else {
        // 非chedaolane连接线最后面延长8m
        int size = history_lane_points.size();
        if (size > 1) {
          double dx           = history_lane_points[size - 1].x - history_lane_points[size - 2].x;
          double dy           = history_lane_points[size - 1].y - history_lane_points[size - 2].y;
          double delta_length = std::hypot(dx, dy);
          if (delta_length > 0.1 && delta_length < 0.5) {
            double k     = std::abs(1 / delta_length);
            dx           = dx * k;
            dy           = dy * k;
            delta_length = std::hypot(dx, dy);
          }
          double length = 0.0;
          int    break_index{0};
          while (length < 6.0 && break_index < 20) {
            Point2DF point;
            point.x = history_lane_points.back().x + dx;
            point.y = history_lane_points.back().y + dy;
            history_lane_points.emplace_back(point);
            length += delta_length;
            break_index++;
          }
        }
      }

      // 检查连接线如果小于toplane的x大于3m就增加点
      int index{0};
      double top_lane_first_x = top_lane.is_connect_lane ? bev_points.front().x : top_lane.points.front().x();
      while (history_lane_points.back().x + 3.0 < top_lane_first_x && index < 100) {
        double x = history_lane_points.back().x + 1.0;
        x2       = x * x;
        x3       = x * x2;
        y        = curve.c0 + curve.c1 * x + curve.c2 * x2 + curve.c3 * x3;
        history_lane_points.emplace_back(x, y);
        index++;
      }

      // for (const auto &point : history_lane_points) {
      //   FLOG_CROSS << " out history_lane_points: " << point.x << " ,y: " << point.y;
      // }
      // FLOG_CROSS << " ------------------out bev_points_size: " << bev_points.size();
      // for (const auto &point : bev_points) {
      //   FLOG_CROSS << " out bev_points: " << point.x << " ,y: " << point.y;
      // }

      // 坐标转换
      if (top_lane.is_connect_lane) {
        bev_top_lane->line_points.clear();
        bev_top_lane->line_points.reserve(bev_points.size());
        for (const auto &pt : bev_points) {
          bev_top_lane->line_points.emplace_back(TransformPoint(pt, data_manager_->Twb()));
        }
      }

    } else if(connect_lane_.line_points.empty()) {
      FLOG_CROSS << " FitterLeastSquareMethod faild,and using bazel";
      return GenerateConnectLane();
    }else{
      FLOG_CROSS << " FitterLeastSquareMethod faild!!!";
      return false;
    }
    cem::message::sensor::BevLaneInfo connect_lane;
    connect_lane.id = kConnectLaneId;
    // connect_lane.id                   = data_manager_->GetNewLandId();
    connect_lane.left_lane_marker_id  = kConnectLeftLanemarkerId;
    connect_lane.right_lane_marker_id = kConnectRightLanemarkerId;
    connect_lane.previous_lane_ids.emplace_back(main_lane.lane_id);
    connect_lane.is_virtual  = true;
    connect_lane.navi_action = main_lane.navi_action;
    bool has_ego_pos         = false;
    for (const auto &lane : data_manager_->bev_map()->lane_infos) {
      if (lane.position == 0) {
        has_ego_pos = true;
        break;
      }
    }
    if (has_ego_pos) {
      connect_lane.position = 7;
    }
    if (top_lane.is_connect_lane) {
      connect_lane.next_lane_ids.emplace_back(top_lane.lane_id);
    }
    connect_lane.line_points.clear();
    connect_lane.line_points.reserve(history_lane_points.size());
    for (const auto &pt : history_lane_points) {
      connect_lane.line_points.emplace_back(TransformPoint(pt, data_manager_->Twb()));
    }
    connect_lane_ = connect_lane;
    FLOG_CROSS << " success";
    return true;
  }
  FLOG_CROSS << " bad";
  return false;
}

bool RoadConnector::UpdateConnectLane() {

  FLOG_CROSS << "UpdateConnectLane";
  return GenerateConnectLane(connect_lane_);
}

bool RoadConnector::UpdateMainLane() {
  // 1. 拟合ego_lane
  Curve curve;
  bool  fitter_egolane_success = FitterEgelane(curve);
  FLOG_CROSS << " fitter_egolane_success_: " << fitter_egolane_success;
  const auto *ego_lane = data_manager_->EgoLane();
  if (fitter_egolane_success) {
    FLOG_CROSS << " NewMainLane ";
    NewMainLane(curve.c0, curve.c1, ego_lane->line_points, ego_lane->width / 2, ego_lane->id);
    // 更新dr_ego_lane
    dr_ego_lane_ = *data_manager_->EgoLane();
    for (auto &point : dr_ego_lane_.line_points) {
      TransformPoint(&point, data_manager_->Twb());
      // FLOG_CROSS << " dr point x: " << point.x << " ,y: " << point.y;
    }
  } else {
    FLOG_CROSS << " UpdateConnectMainLane ";
    UpdateConnectMainLane(connect_lane_);
  }
  return true;
}

bool RoadConnector::FitterEgelane(Curve &ego_curve) {
  FLOG_CROSS << " FitterEgelane";
  // 车辆过了停止线或者未过之前没有了egolane都不进行拟合
  if (data_manager_->GetIsInCrossroad() || (!data_manager_->GetIsInCrossroad() && !data_manager_->has_ego_lane())) {
    return false;
  }
  const auto& points = data_manager_->EgoLane()->line_points;
  auto size = points.size();
  if (size < 2) {
    return false;
  }
  // 选取egolane的最后20m进行直线拟合求heading
  double end_x = data_manager_->EgoLane()->line_points.back().x;
  std::vector<Point2DF> fit_points;
  // std::vector<Point2DF> fit_points(
  //     ((size > 25) ? data_manager_->EgoLane()->line_points.begin() + (size - 25) : data_manager_->EgoLane()->line_points.begin()),
  //     data_manager_->EgoLane()->line_points.end());
  for (int i = 1; i < size; ++i) {
    if (end_x - points[i].x < 20.0) {
      // FLOG_CROSS << " fit x: " << points[i].x << " ,y: " << points[i].y;
      fit_points.emplace_back(points[i]);
    }
  }
  return FitterLeastSquareMethod(fit_points, 1, ego_curve);
}

void RoadConnector::NewMainLane(double c0, double c1, const std::vector<cem::message::common::Point2DF> &ego_points, double half_width,
                                uint64_t lane_id, double control_dis) {
  auto &main_lane = data_manager_->GetMainLane();
  main_lane.Reset();
  // 第一次生成main_lane
  main_lane.lane_id     = lane_id;
  main_lane.navi_action = data_manager_->turn_type();
  main_lane.angle       = std::atan(c1);
  main_lane.top_points.set_x(ego_points.back().x);
  main_lane.top_points.set_y(ego_points.back().y);
  main_lane.valid      = true;
  main_lane.is_virtual = false;
  main_lane.points.emplace_back(main_lane.top_points);
  // 使用系数求取为了防止原始车道中心线末端两个点是斜的
  double x = ego_points.back().x + control_dis;
  double y = c0 + c1 * x;
  Vec2d  next_point(x, y);
  main_lane.points.emplace_back(next_point);
  // 求左侧main lane
  Vec2d axis = next_point - main_lane.top_points;
  axis.Normalize();
  axis.SelfRotate(pi / 2);
  main_lane.left_points     = TranslateSegmentAlongAxisStart(axis, main_lane.points, half_width);
  main_lane.left_top_points = main_lane.left_points.front();
  // 求右侧main lane
  main_lane.right_points     = TranslateSegmentAlongAxisStart(axis, main_lane.points, -half_width);
  main_lane.right_top_points = main_lane.right_points.front();
  FLOG_CROSS << " generate NewMainLane end!";
}

void RoadConnector::UpdateConnectMainLane(const cem::message::sensor::BevLaneInfo& lane) {
  auto& main_lane = data_manager_->GetMainLane();
  main_lane.Reset();
  main_lane.lane_id = dr_ego_lane_.id;
  main_lane.navi_action = data_manager_->turn_type();
  main_lane.valid = true;
  main_lane.is_virtual = true;
  // 中心线
  auto dr_points_size = lane.line_points.size();
  if (dr_points_size <= 2) {
    main_lane.valid = false;
    return;
  }
  std::vector<Vec2d> fit_dr_ego_points;
  fit_dr_ego_points.reserve(dr_points_size);
  for (auto it = lane.line_points.begin(); it != lane.line_points.end(); it++) {
    auto point = TransformPoint(*it, data_manager_->Tbw());
    fit_dr_ego_points.emplace_back(point.x, point.y);
  }

  // 将车后的点放入到main_lane中，最少放两个
  int index = 0;
  for (int i = fit_dr_ego_points.size() - 1; i >= 0; i--) {
    if (fit_dr_ego_points[i].x() < 15.0) {
      index = i;
      break;
    }
  }
  fit_dr_ego_points.resize(std::max(2, index + 1));
  main_lane.points = fit_dr_ego_points;
  main_lane.angle  = (fit_dr_ego_points[fit_dr_ego_points.size() - 1] - fit_dr_ego_points[fit_dr_ego_points.size() - 2]).Angle();

  if (!main_lane.points.empty()) {
    main_lane.top_points = main_lane.points.back();
    FLOG_CROSS << "UpdateVirtualMainLane heading:" << main_lane.angle << " ,main_lane backpoint_x: " << main_lane.points.back().x()
               << " ,y: " << main_lane.points.back().y() << " ,angle: " << main_lane.angle;
  }

  // 左侧
  auto dr_left_points_size = connect_left_lanemarker_.line_points.size();
  std::vector<cem::message::common::Point2DF> fit_dr_left_points;
  fit_dr_left_points.reserve(dr_left_points_size);
  for (auto it = connect_left_lanemarker_.line_points.begin(); it != connect_left_lanemarker_.line_points.end(); it++) {
    fit_dr_left_points.emplace_back(TransformPoint(*it, data_manager_->Tbw()));
  }
  for (int i = 0; i < fit_dr_left_points.size(); i++) {
    main_lane.left_points.emplace_back(fit_dr_left_points[i].x, fit_dr_left_points[i].y);
    if (fit_dr_left_points[i].x > 0.0 && main_lane.left_points.size() > 1) {
      main_lane.angle = byd::common::math::Vec2d(fit_dr_left_points[i].x - fit_dr_left_points[i - 1].x,
                                                 fit_dr_left_points[i].y - fit_dr_left_points[i - 1].y)
                            .Angle();
      break;
    }
  }
  if (!main_lane.left_points.empty()) {
    main_lane.left_top_points = main_lane.left_points.back();
    FLOG_CROSS << " main_left point_1_x: " << main_lane.left_points.front().x() << " ,y: " << main_lane.left_points.front().y();
  }

  // 右侧
  auto dr_right_points_size = connect_right_lanemarker_.line_points.size();
  std::vector<cem::message::common::Point2DF> fit_dr_right_points;
  fit_dr_right_points.reserve(dr_right_points_size);
  for (auto it = connect_right_lanemarker_.line_points.begin(); it != connect_right_lanemarker_.line_points.end();
       it++) {
    fit_dr_right_points.emplace_back(TransformPoint(*it, data_manager_->Tbw()));
  }
  for (int i = 0; i < fit_dr_right_points.size(); i++) {
    main_lane.right_points.emplace_back(fit_dr_right_points[i].x, fit_dr_right_points[i].y);
    if (fit_dr_right_points[i].x > 0.0 && main_lane.right_points.size() > 1) {
      main_lane.angle = byd::common::math::Vec2d(fit_dr_right_points[i].x - fit_dr_right_points[i - 1].x,
                                                 fit_dr_right_points[i].y - fit_dr_right_points[i - 1].y)
                            .Angle();
      break;
    }
  }
  if (!main_lane.right_points.empty()) {
    main_lane.right_top_points = main_lane.right_points.back();
    FLOG_CROSS << "mian_right point_1_x: " << main_lane.right_points.front().x() << " ,y: " << main_lane.right_points.front().y();
  }
  FLOG_CROSS << "UpdateConnectMainLane end";
}

cem::message::sensor::BevLaneInfo *RoadConnector::FindBevLaneById(uint64_t id) {
  if (data_manager_->bev_map() == nullptr) {
    return nullptr;
  }
  for (auto &lane : data_manager_->bev_map()->lane_infos) {
    if (lane.id == id) {
      return &lane;
    }
  }
  return nullptr;
}

}  // namespace fusion
}  // namespace cem