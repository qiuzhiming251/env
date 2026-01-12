/**
 * @file virtual_egoroad_processor.h
 * @author fanminglei (fan.minglei@byd.com)
 * @brief
 * @version 2.1
 * @date 2025-04-08
 *
 * @copyright Copyright (c) 2025 BYD Corporation. All rights reserved.
 * @copyright
 * @copyright BYD Corporation and its licenses retain all intellectual property
 * @copyright and proprietary rights in and to this software, related documentation
 * @copyright and any modifications thereto. Any use, reproduction, disclosure or
 * @copyright distribution of this software and related documentation without an express
 * @copyright license agreement from BYD Corporation is strictly prohibited.
 *
 */
#ifndef VIRTUALEGOROAD_PROCESSOR_H
#define VIRTUALEGOROAD_PROCESSOR_H

#include "cross_data_manager.h"
#include "crossroad_lane_selector.h"

namespace cem {
namespace fusion {
using cem::message::sensor::BevArrowType;
///
class VirtualEgoRoadProcessor {
 public:
  struct VirtualLaneDegeDid {
    int    index{-1};
    double dis{std::numeric_limits<double>::max()};
    bool   val{false};
  };
  VirtualEgoRoadProcessor();
  void Init(const std::shared_ptr<CrossDataManager>& data_manager);
  void ReSet();
  bool GetVirtualLanes();
  void UpdateVirtualLanes();
  const BevLaneInfo& GetDrGeoLane(){return dr_ego_lane_;}
  inline bool has_virtual_lane() { return has_virtual_lane_; }
  BevLaneInfo& ego_lane() { return ego_lane_; }
  BevLaneMarker& ego_left_lanemarker() { return ego_left_lanemarker_; }
  BevLaneMarker& ego_right_lanemarker() { return ego_right_lanemarker_; }
  BevLaneInfo& virtual_lane() { return virtual_lane_; }
  BevLaneMarker& virtual_left_lanemarker() { return virtual_left_lanemarker_; }
  BevLaneMarker& virtual_right_lanemarker() { return virtual_right_lanemarker_; }

 private:
  bool GenerateStraightLane();

  bool GenerateStraightLaneNew();

  bool GenerateLeftTurnLaneNew();
  bool GenerateRightTurnLaneNew();
  bool GenerateLRTurnLaneNew();
  bool GenerateLeftTurnLane();

  bool GenerateRightTurnLane();

  bool GenerateAroundTurnLane();

  void GenerateVirtualLaneAndLaneMarker(const std::vector<Point2DF>& points);

  bool GenerateTurnLaneAndLaneMarker();

  bool GenerateAroundTurnLaneAndLaneMarker();

  TopLane GenerateTopLanePoints(const byd::common::math::Vec2d& point, double heading, double half_width);
  TopLane GenerateAroundTurnTopLanePoints(const byd::common::math::Vec2d& point, double heading, double half_width);

  void UpdateStraightMainLane();
  void NewMainLane(double c0, double c1, const std::vector<Point2DF>& ego_points,
                   double half_width,uint64_t lane_id,double control_dis = 20.0);
  void UpdateVirtualMainLane();
  void UpdateTurnMainLane();

  bool GenerateVirtualStraightLinePoints(std::vector<Point2DF> &virtual_line_points, Curve curve);
  bool GenerateVirtualStraightLinePointsByLd(std::vector<Point2DF> &virtual_line_points, Curve ego_curve);
  bool GenerateVirtualStraightLinePointsBySd(std::vector<Point2DF> &virtual_line_points, Curve ego_curve);
  bool GenerateVirtualStraightLinePointsByObs(std::vector<Point2DF> &virtual_line_points, Curve ego_curve);
  bool GenerateVirtualStraightLinePointsByHeading(std::vector<Point2DF> &virtual_line_points, Curve ego_curve);
  bool GenerateVirtualStraightLinePointsByExpTraj(std::vector<Point2DF> &virtual_line_points, Curve ego_curve);

  bool GenerateVirtualLRTurnLinePoints(std::vector<Point2DF> &virtual_line_points, Curve curve);
  bool GenerateVirtualLRTurnLinePointsByLd(std::vector<Point2DF> &virtual_line_points, Curve ego_curve);
  bool GenerateVirtualLRTurnLinePointsBySd(std::vector<Point2DF> &virtual_line_points, Curve ego_curve);
  bool GenerateVirtualLRTurnLinePointsByObs(std::vector<Point2DF> &virtual_line_points, Curve ego_curve);
  bool GenerateVirtualLRTurnLinePointsByHeading(std::vector<Point2DF> &virtual_line_points, Curve ego_curve);
  bool GenerateVirtualLRTurnLinePointsByExpTraj(std::vector<Point2DF> &virtual_line_points, Curve ego_curve);

  bool GenerateVirtualAroundTurnLinePoints(std::vector<Point2DF> &virtual_line_points, Curve curve);
  bool GenerateVirtualAroundTurnLinePointsByPer(std::vector<Point2DF> &virtual_line_points, Curve ego_curve);

  // int  DecideStraightVirtualLineSource();
  bool GenerateVirtualLinePointsByLd(std::vector<Point2DF> &virtual_line_points, Curve ego_curve);
  bool GenerateVirtualLinePointsByExpTraj(std::vector<Point2DF> &virtual_line_points, Curve ego_curve);

  // int  DecideLRTurnVirtualLineSource();

  VirtualLineSource  DecideAroundTurnVirtualLineSource();

  bool FitterEgelane(Curve&  ego_curve);

  void GenerateMainLane(Curve&  ego_curve);

  bool GenerateVirtualStraightPoints(std::vector<Point2DF> &virtual_line_points, Point2DF start_point, Curve ego_curve);

  bool CaculateMoveDis(double& move_dis);
  bool CaculateMoveDisOcc(double& move_dis);

  std::shared_ptr<CrossDataManager> data_manager_{nullptr};
  BevLaneInfo ego_lane_;
  BevLaneInfo dr_ego_lane_;
  BevLaneMarker ego_left_lanemarker_;
  BevLaneMarker ego_right_lanemarker_;
  BevLaneInfo virtual_lane_;
  BevLaneMarker virtual_left_lanemarker_;
  BevLaneMarker virtual_right_lanemarker_;

  int straight_exceed_heading_count_{0};
  BevAction his_turn_type_{BevAction::STRAIGHT};
  std::shared_ptr<CrossRoadSelector> cross_road_selector_{nullptr};
  double left_right_move_dis_{0.0};
  bool has_virtual_lane_{false};
  bool is_first_generate_{true};
  bool is_generate_success_{false};
  bool fitter_egolane_success_{false};
  bool is_straight_exceed_heading_{false};
};
}  // namespace fusion
}  // namespace cem
#endif  // VIRTUALEGOROAD_PROCESSOR_H