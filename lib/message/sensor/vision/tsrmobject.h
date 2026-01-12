#ifndef TSRMOBJECT_H_
#define TSRMOBJECT_H_
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "message/common/header.h"
#include "message/common/geometry.h"
namespace cem {
namespace message {
namespace sensor {

struct Tsrmobject
{
    cem::message::common::Header header;
    int32_t id = 0;             // track id
    int32_t tracked_age;        // number of frames tracked continuously
    int8_t classid;             // detection class id
    float confidence;           // detection confidence
    std::vector<float> bbox;    // bbox = {x,y,w,h}, (x,y) is top-left point
    std::vector<float> polygon; // reserved
    int8_t occlusion;           // 0: no occlusion, 1: has occlusion
    int8_t truncation;          // 0: no truncation, 1: has truncation
    int8_t direction;           // reserved
    int8_t color;               // reserved
    int8_t shape;               // reserved
    int32_t status;        // 0: default value, 1: need to classification, other
                           // number: reserved
    int8_t speedid;        // speed limit classification id
    int8_t speed_validity; // speed limit validity, 0: invalid, 1: valid
    std::string details;   // reserved
    std::vector<float> position_3d; // center point in vehicle coordinate system
    float position_3d_confidence;   // position_3d status. 0.5: track without
                                    // odom,  1: track with odom
};

enum ObjectType {
  UNKNOWN = 0, // 未知
  CAR = 1, // 轿车
  TRUCK = 2, // 卡车
  TRAM = 3, // 公交
  PEDESTRIAN = 4, // 行人
  CYCLIST = 5, // 骑车人
  TRICYCLE = 6, // 三轮车
  TRAFFIC_CONE = 7, // 锥桶
  CRASH_BARRELS = 8, // 防撞筒
  WATER_HORSE = 9, // 水马
  FIRE_HYDRANT = 10, // 消防栓
  TRAFFIC_WARNING_POLE = 11, // 交通警示柱
  TRAILER_HEAD = 12, // 挂车车头
  TRAILER_REAR = 13, // 挂车车身
  SPEED_RESTRICTION_BOARD = 14, // 限速牌
  ANIMAL = 15, // 动物
  TRIANGLE_BOARD = 16, // 三角牌
  TRAFFIC_LIGHT = 17 // 红绿灯
};

enum CameraSource {
  NONE = 0, // BEV结果中使用
  FRONT_NARROW = 1,
  FRONT_WIDE = 2,
  LEFT_BACK = 3,
  LEFT_FRONT = 4,
  RIGHT_BACK = 5,
  RIGHT_FRONT = 6,
  REAR = 7
};

enum TrackStatus {
  INVALID = 0,
  NEW = 1,
  UPDATED = 2,
  COASTED = 3
};

enum SpeedRestrictionType { //限速牌属性
  SP_UNKNOWN = 0, // 未知
  SP_MAX_SPEED = 1, //最高限速
  SP_MIN_SPEED = 2, //最低限速
  SP_CANCEL_RESTRICTION = 3 //取消限速
};

enum TrafficLightDirectionType { //红绿灯朝向
  TLD_UNKNOWN = 0, // 未知
  TLD_RIGHT = 1, 
  TLD_RIGHT_BACK = 2,
  TLD_BACK = 3,
  TLD_LEFT_BACK = 4,
  TLD_LEFT = 5,
  TLD_OTHER = 6
};

enum TrafficLightColorType {  //红绿灯颜色
  TLC_UNKNOWN = 0,
  TLC_RED     = 1,
  TLC_GREEN   = 2,
  TLC_YELLOW  = 3,
  TLC_BLACK   = 4,
  TLC_OTHER   = 5,

  TLC_GREEN_FLASHING  = 6,
  TLC_YELLOW_FLASHING = 7,
  TLC_RED_FLASHING    = 8,
  TLC_BLURRING_MODE   = 9,
  TLC_BLOCK_FAILED    = 10,
  TLC_NONE_LIGHT      = 11,
  TLC_CLOUD_NOMATCH   = 12,
  TLC_NOT_MATCH       = 13,
};

enum TrafficLightShapeType { //红绿灯形状
  TLS_UNKNOWN = 0, // 未知
  TLS_CIRCULAR = 1, //圆形
  TLS_LEFT_ARROW = 2,//向左箭头
  TLS_RIGHT_ARROW = 3,//向右箭头
  TLS_UP_ARROW = 4,//向上箭头
  TLS_DOWN_ARROW = 5,//向下箭头
  TLS_BICYCLE = 6, //自行车
  TLS_PEDESTRIAN = 7, //行人
  TLS_CLOSE_TO_TRAFFIC = 8, //禁止通行
  TLS_UP_LEFT_ARROW = 9, //上与左箭头
  TLS_TURN_ARROUND_ARROW = 10, //掉头箭头
  TLS_UP_RIGHT_ARROW = 11, //上与右箭头
  TLS_LEFT_TURN_ARROUND_ARROW = 12, //左与转弯箭头
  TLS_SLOW_DOWN = 13, //慢
  TLS_HEART_SHAPE = 14, // 心型图案
  TLS_NUMBER = 15, //倒计时数字
  TLS_PROCESS_BAR = 16, //进度条红绿灯
  TLS_BUS_ONLY_SHAPE = 17, //公交专用
  TLS_RECTANGLE = 18, //方块形 
  TLS_OTHER_SHAPE = 19 //其它形状
};

struct TrfAttributesInfo
{
    //限速牌属性
    SpeedRestrictionType speed_restriction_type = SP_UNKNOWN;
    uint32_t speed_restriction_num = 0;
    //红绿灯属性
    TrafficLightDirectionType traffic_light_direction = TLD_UNKNOWN;
    TrafficLightColorType traffic_light_color = TLC_UNKNOWN;
    TrafficLightShapeType traffic_light_shape = TLS_UNKNOWN;
    uint32_t traffic_light_num = 0;
    bool traffic_light_flashing = false;
};

enum class LightTurn {
  LEFT     = 0,
  RIGHT    = 1,
  STRAIGHT = 2,
  UTURN    = 3,
  UNKNOWN  = 4,
};

enum DetectionSource {
  SOURCE_UNKNOWN          = 8,   // 未知检测源，默认值
  MONO_FRONT_NARROW       = 1,   // Mono 前窄
  MONO_FRONT_WIDE         = 2,   // Mono 前宽
  MONO_LEFT_BACK          = 3,   // Mono 左后
  MONO_LEFT_FRONT         = 4,   // Mono 左前
  MONO_RIGHT_BACK         = 5,   // Mono 右后
  MONO_RIGHT_FRONT        = 6,   // Mono 右前
  MONO_REAR               = 7,   // Mono 后视
  BEV_VISION              = 0,   // BEV 纯视觉
  BEV_LIDAR_VISION        = 9,   // BEV 视觉 LiDAR 前融合
  BEV_LIDAR               = 10,  // BEV 纯 LiDAR
  BEV_LIDAR_VISION_RADAR  = 11,  // BEV 视觉 LiDAR Radar 前融合
  MONO_FRONT_NARROW_LIDAR = 12,  // Mono 前窄 LiDAR融合
  MONO_FRONT_WIDE_LIDAR   = 13,  // Mono 前宽 LiDAR融合
};

struct TrfObjectInfo {
    uint32_t          id     = 0;        // id
    ObjectType        type   = UNKNOWN;  // 类别，目前只支持SPEED_RESTRICTION_BOARD和TRAFFIC_LIGHT
    CameraSource      source = NONE;     // 来源
    TrfAttributesInfo attributes;        // 属性

    cem::message::common::Point3DD position     = {0.0, 0.0, 0.0};  // 自车坐标系下的位置 x, y, z
    cem::message::common::Point3DD position_std = {0.0, 0.0, 0.0};  // 位置标准差

    float       length       = 0.0;
    float       width        = 0.0;
    float       height       = 0.0;
    float       exist_score  = 0.0;      // 可信度分数，取值范围为0-1，1为可信
    double      direction_conf = 0.0;
    double      shape_conf     = 0.0;
    uint32_t    track_age    = 0;        // track age，这里一次检出的间隔是0.1s
    TrackStatus track_status = INVALID;  // 追踪状态信息
    double      publish_time = 0.0;
    std::vector<LightTurn> turn;

    uint32_t traffic_light_box_id = 0;
    bool     is_occluded          = false;
    double   yellow_flashing_start_time = 0.0;
    DetectionSource det_source{DetectionSource::SOURCE_UNKNOWN};
};

struct PercepTrfInfo
{
    cem::message::common::Header header;
    uint32_t num_objects = 0;
    std::vector<TrfObjectInfo> objects = {};
};

struct VisionTrfInfo
{
    cem::message::common::Header header;
    uint32_t num_objects = 0;
    std::vector<TrfObjectInfo> objects = {};
};

} // namespace sensor
} // namespace message
} // namespace cem

#endif
