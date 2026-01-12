#ifndef MESSENGER_H_
#define MESSENGER_H_

#include "middleware/middleware_util.h"
#include "message/internal_message.h"
#include "base/sensor_data_manager.h"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "vehicle_msgs/msg/ap_ipd100ms_pdu12.hpp"
#include "lib/message/env_model/stop_line/stop_line.h"
#include "message_adapter/cyberRT_adapter.h"
#include "lib/perception_and_ld_map_fusion/data_fusion/geometry_match_info.h"
#include "lib/common/log_custom.h"
#include "draw_point.h"
#include "base/params_manager/params_manager.h"
#include  "lib/perception_and_ld_map_fusion/data_fusion/match_maker.h"


namespace cem {
namespace fusion {
class Messenger
{  
public:
    using MsgRoutingMapPtr = std::shared_ptr<byd::msg::orin::routing_map::RoutingMap>;
    using MsgRoutingMap = byd::msg::orin::routing_map::RoutingMap;
    using Points = google::protobuf::RepeatedPtrField<byd::msg::orin::routing_map::Point>;
    using Point = byd::msg::orin::routing_map::Point;
private:
    std::shared_ptr<byd::msg::basic::ModuleStatus> diagno_info_ = std::make_shared<byd::msg::basic::ModuleStatus>();
    std::shared_ptr<byd::msg::orin::routing_map::RoutingMap> routing_map_ = std::make_shared<byd::msg::orin::routing_map::RoutingMap>();
    std::shared_ptr<byd::msg::orin::routing_map::MapEvent> map_event_ = std::make_shared<byd::msg::orin::routing_map::MapEvent>();
    std::shared_ptr<byd::msg::orin::routing_map::TrafficLightsE2EInfo> traffic_light_e2e_ = std::make_shared<byd::msg::orin::routing_map::TrafficLightsE2EInfo>();
    diagnostic_msgs::msg::DiagnosticStatus diagnostic_msgs_;
    std::shared_ptr<byd::msg::env_model::ScenarioRealityElement> traffic_light_info_ = std::make_shared<byd::msg::env_model::ScenarioRealityElement>();
    std::shared_ptr<cem::message::env_model::PLanningResultData> planning_result_ = std::make_shared<cem::message::env_model::PLanningResultData>();
    std::shared_ptr<cem::message::env_model::PLanFuncState> plan_func_ = std::make_shared<cem::message::env_model::PLanFuncState>();
    std::shared_ptr<cem::message::env_model::CAN1> can1_func_ = std::make_shared<cem::message::env_model::CAN1>();
    std::shared_ptr<byd::msg::perception::OCCInfo> occ_func_ = std::make_shared<byd::msg::perception::OCCInfo>();
    GeometryMatchInfo* geometry_match_info_{nullptr};
    std::vector<MatchMaker::LaneMatchPair> match_pairs_;
    bool current_bev_has_top_{false};
    bool is_local_map_ = false;
    uint64_t routing_map_count_ = 0;
    Eigen::Isometry3d T_ego_local_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_local_ego_ = Eigen::Isometry3d::Identity();
#if MAP_SMOOTH_DEBUG
    std::shared_ptr<ppm::PPMImage>  ppm_image_ = nullptr;
#endif  
public:
    Messenger();
    ~Messenger() = default;

    std::shared_ptr<byd::msg::basic::ModuleStatus> GetDiagnoInfo() { return diagno_info_; };
    std::shared_ptr<byd::msg::env_model::ScenarioRealityElement> GetTrafficLightInfo() { return traffic_light_info_; };
    std::shared_ptr<byd::msg::orin::routing_map::RoutingMap> GetFusionMapInfo() { return routing_map_; };
    std::shared_ptr<byd::msg::orin::routing_map::RoutingMap> GetRoutingMapInfo() { return routing_map_; };
    std::shared_ptr<byd::msg::orin::routing_map::TrafficLightsE2EInfo> GetTrafficLightsE2EInfo() { return traffic_light_e2e_; };
    std::shared_ptr<byd::msg::orin::routing_map::MapEvent> GetMapEventInfo() { return map_event_; };
    std::shared_ptr<cem::message::env_model::PLanningResultData> GetPlanningResult() { return planning_result_; };
    std::shared_ptr<cem::message::env_model::PLanFuncState> GetPlanFuncState() { return plan_func_; };
    std::shared_ptr<cem::message::env_model::CAN1> GetDNPState() { return can1_func_; };
    void  SetGeometryMatchInfo(const std::vector<MatchMaker::LaneMatchPair>& match_pairs,const bool& has_top){
        match_pairs_ = match_pairs;
        current_bev_has_top_ = has_top;

    }

    void Publish(RoutingMapPtr routing, cem::fusion::navigation::AccLaneInfo  sd_acc_lane_info);

    void PulishDiagno();

    void PulishFusionMap(bool is_local_map, const BevMapInfoPtr lane_markers_ptr, const RoutingMapPtr routing_map,
                         const std::vector<cem::message::env_model::StopLine> stop_lines_ptr, bool is_on_highway_flag,
                         cem::fusion::navigation::AccLaneInfo  sd_acc_lane_info, const std::string &info = "");


    void PulishTrafficLightE2E(const TrafficLightsE2EInfoPtr traffic_lights_e2e);
    void MapSourceSmoother(bool is_local_map, const MsgRoutingMapPtr routing_map, MsgRoutingMapPtr output_routing_map,BevMapInfoConstPtr bev_map_info);
    void FixLaneConnections(MsgRoutingMapPtr routing_map, 
                                  const  std::vector<std::pair<uint64_t,std::pair<Eigen::Vector3d,Eigen::Vector3d>>>& split_points);

public:
    template <typename InputMsgType, typename InternalMsgType>
    void Callback(const std::shared_ptr<InputMsgType> &input_msg);

    template <typename InputMsgType, typename InternalMsgType1,
              typename InternalMsgType2>
    void Callback(const std::shared_ptr<InputMsgType> &input_msg);
private:
    struct SmoothTransitionState {
        bool last_is_local_map = false;
        bool is_transitioning = false;
        int transition_frames = 0;
        MsgRoutingMapPtr last_map_lane;
        MsgRoutingMapPtr last_perception_lane;
    };

    // 平滑过渡相关方法
    void PerformSmoothTransition(bool is_local_map, const MsgRoutingMapPtr current_map, 
                                MsgRoutingMapPtr output_routing_map,BevMapInfoConstPtr bev_map_info);
    void SmoothLaneTransition(const MsgRoutingMapPtr source_lane, const MsgRoutingMapPtr target_lane,
                            double weight, MsgRoutingMapPtr output_routing_map,BevMapInfoConstPtr bev_map_info);
    
    // 点集处理相关方法
    bool SmoothLanePoints(const Points& source_points,
                         const Points& target_points,
                         double weight, std::deque<Point> &output_points);
    void ResampleAndMatchPoints(const Points& source_points,
                               const Points& target_points,
                               Points& resampled_source,
                               Points& resampled_target);
    
    Point InterpolatePoint(const Point& p1, const Point& p2, double t);
    
    // 边界处理
    void SmoothBoundaryPoints(const std::vector<LaneBoundaryInfo>& source_boundaries,
                             const std::vector<LaneBoundaryInfo>& target_boundaries,
                             double weight, std::vector<LaneBoundaryInfo>& output_boundaries);
    bool HasSharpTurn(const Point& p1, const Point& p2, const Point& p3, 
                           double max_angle_deg = 90.f);

    // 工具函数
    template<typename T>
    T Lerp(const T& a, const T& b, double t) {
        return a + (b - a) * t;
    }

    SmoothTransitionState smooth_state_;
    static const int TRANSITION_DURATION = 5;
    void ResampleAndMatchPointsByArcLengthCompletion(
        const Points& source_points,
        const Points& target_points,
        Points& resampled_source,
        Points& resampled_target);
    void ResampleAndMatchPointsByShortCutoff(
        const Points& source_points,
        const Points& target_points,
        Points& resampled_source,
        Points& resampled_target);
    
    Point SamplePointWithCompletion(
        const Points& short_curve,
        const Points& long_curve,
        double target_arc_length, double short_length, double long_length);
    
    double CalculatePolylineLength(
        const Points& points);
    
    Point SamplePointByArcLength(
        const Points& points, 
        double target_length);
};

template <typename InputMsgType, typename InternalMsgType>
void Messenger::Callback(const std::shared_ptr<InputMsgType> &input_msg)
{
    auto internal_msg = std::make_shared<InternalMsgType>();
    AdaptInput2Internal(input_msg, internal_msg);
    SensorDataManager::Instance()->AddSensorMeasurements(internal_msg);
}

template <typename InputMsgType, typename InternalMsgType1,
          typename InternalMsgType2>
void Messenger::Callback(const std::shared_ptr<InputMsgType> &input_msg)
{
    Callback<InputMsgType, InternalMsgType1>(input_msg);
    Callback<InputMsgType, InternalMsgType2>(input_msg);
}

} // namespace fusion
} // namespace cem


#endif
