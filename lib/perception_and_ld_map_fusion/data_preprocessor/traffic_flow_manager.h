#ifndef _TRAFFIC_FLOW_MANAGER_H
#define _TRAFFIC_FLOW_MANAGER_H

#include "common/utility.h"
#include "message/internal_message.h"
#include "message/common/geometry.h"
#include "base/sensor_data_manager.h"

namespace cem {
namespace fusion {

typedef struct FusionObjs_s
{
    uint32_t id{0};
    message::common::Point3DD position;
    message::common::Point3DD velocity;
    uint16_t source{0};
    double speed{0.0};
    double heading{0.0};
    double filter_heading{0.0};
    double length{0.0};
    double width{0.0};
    double heading_rate{0.0};
    double curvature{0.0};
    double start_offset{0.0};
    double end_offset{0.0};
    cem::message::env_model::Curve curve;
    // 0:other 1:left front 2: front 3:right_front
    // 4:left 5:right 6:left rear 7:rear 8:right rear
    int lane_position{0};

}FusionObjs;
typedef struct MotionObject2Lane_s
{
    uint32_t lane_id{0};
    std::vector<FusionObjs> motionObjects;
}MotionObject2Lane;

class TrafficFlowManager
{
public:
    TrafficFlowManager();
    ~TrafficFlowManager();

    void ParseTrafficFlowRawData(std::unordered_map<uint32_t, FusionObjs> & objs);

    void BindObstaclesToBevLane(const BevMapInfoPtr& localmap,std::vector<MotionObject2Lane> & object2Lanes);

    void BindObstaclesToLdMapLane(const RoutingMapPtr& ldmap,std::vector<MotionObject2Lane> & object2Lanes);

private:

    float GetDistPointLane(
        const Eigen::Vector2f &point_a,
        const Eigen::Vector2f &point_b,
        const Eigen::Vector2f &point_c);

    float Point2LineDistanece(
        const Eigen::Vector2f &point,
        const std::vector<Eigen::Vector2f> & line);

    std::unordered_map<uint32_t, FusionObjs> fusion_objs_;


};

}
}
#endif
