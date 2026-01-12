#ifndef RADA_VERSION_OBSTACLES_FUN_H_
#define RADA_VERSION_OBSTACLES_FUN_H_

#include <cstdint>
#include <vector>

#include "message/common/header.h"
#include "message/common/geometry.h"
namespace cem {
namespace message {
namespace sensor {

//参考通信接口：Orin-N通信接口V2.52.
//https://doc.weixin.qq.com/sheet/e3_AW4ADQZBAK8JZvpk5owTAWVuasx5f?scode=AFIAoQc6AAg9oJp7y2AQcAvAYNAOc&version=4.1.41.6006&platform=win
enum class ObjectTypeClass
{
    UNKNOWN = 0,                  // 未知
    CAR = 1,                      // 轿车
    TRUCK = 2,                    // 卡车
    TRAM = 3,                     // 公交
    PEDESTRIAN = 4,               // 行人
    CYCLIST = 5,                  // 骑车人
    TRICYCLE = 6,                 // 三轮车
    TRAFFIC_CONE = 7,             // 锥桶
    CRASH_BARRELS = 8,            // 防撞筒
    WATER_HORSE = 9,              // 水马
    FIRE_HYDRANT = 10,            // 消防栓
    TRAFFIC_WARNING_POLE = 11,    // 交通警示柱
    TRAILER_HEAD = 12,            // 挂车车头
    TRAILER_REAR = 13,            // 挂车车身
    SPEED_RESTRICTION_BOARD = 14, // 限速牌
    ANIMAL = 15,                  // 动物
    TRIANGLE_BOARD = 16,          // 三角牌
    TRAFFIC_LIGHT = 17            // 红绿灯
};

struct FusionObj
{
    double timestamp = 0;
    uint32_t id = 0;
    uint8_t obstacle_valid = 0;
    uint16_t fusion_source = 0;
    uint32_t type = 0;

    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;

    // 自车坐标系下障碍物相对+x（正前方）的朝向，弧度制
    float heading_angle = 0.0;
    float heading_rate = 0.0;
    // 障碍物长宽高
    float length = 0.0;
    float width = 0.0;
    float height = 0.0;

    // 障碍物标量速度
    float speed = 0.0;
    // 障碍物标量加速度
    float normal_acceleration = 0.0;
    // 曲率
    float curvature = 0.0;

    // 速度、加速度是否可信，1为可信，0为不可信
    bool is_velocity_valid = false;
    bool is_acceleration_valid = false;

    // 该障碍物可信度分数，取值范围为0-1，1为可信
    float exist_score = 0.0;
    // track age，这里一次检出的间隔是0.1s
    uint32_t track_age = 0;

    // 追踪状态等信息，均需要明确语义并改为enum
    uint8_t track_status = 0;
    uint32_t cipv = 0;
    bool aeb_flag = false;
    uint64_t object_lane_id = 0;
};

struct FusObjInfo
{
    cem::message::common::Header header;
    uint32_t num_tracks = 0;
    std::vector<FusionObj> fused_obstacles;
};

} // namespace sensor
} // namespace message
} // namespace cem

#endif
