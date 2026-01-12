#ifndef LANE_H_
#define LANE_H_

#include <vector>

#include "geometry_line.h"

namespace cem {
namespace message {
namespace env_model {

enum class TrfLaneDirection {
   UNKNOWN = 0,
   BOTH = 1,
   ALONG_DRIVING_DIRECTION = 2,
   AGAINST_DRIVING_DIRECTION = 3
};

enum class TrfLaneTransition {
     None = 0,
     RIGHTEXITBEGIN = 1,
     RIGHTEXITEND = 2,
     LEFTEXITBEGIN = 3,
     LEFTEXITEND = 4,
     RIGHTENTERBEGIN = 5,
     RIGHTENTEREND = 6,
     LEFTENTERBEGIN = 7,
     LEFTENTEREND = 8,
     SPLIT = 9,
     MERGE = 10,
     LANEEND = 11,
     LANEADDRIGHT = 12,
     LANEADDLEFT = 13,
};
enum TrfLanePosition
{
    TRF_LP__UNKNOWN = 0,
    TRF_LP__EGO = 1,
    TRF_LP__FIRST_LEFT = 2,
    TRF_LP__SECOND_LEFT = 3,
    TRF_LP__THIRD_LEFT = 4,
    TRF_LP__FOURTH_LEFT = 5,
    TRF_LP__FIFTH_LEFT = 6,
    TRF_LP__SIXTH_LEFT = 7,
    TRF_LP__FIRST_RIGHT = 8,
    TRF_LP__SECOND_RIGHT = 9,
    TRF_LP__THIRD_RIGHT = 10,
    TRF_LP__FOURTH_RIGHT = 11,
    TRF_LP__FIFTH_RIGHT = 12,
    TRF_LP__SIXTH_RIGHT = 13,
    TRF_LP__OTHER = 14
};

enum class TrfLaneType {
      NORMAL = 0,  // 普通
      ENTRY = 1,   // 入口
      EXIT = 2,    // 出口
      EMERGENCY_LANE = 3,  // 应急车道
      ON_RAMP = 4,     // 进入匝道
      OFF_RAMP = 5,   // 退出匝道
      CONNECTING_RAMP = 6,   // 连接匝道
      ACCELERATION_LANE = 7, // 加速车道
      DECELERATION_LANE = 8, // 减速车道
      SPECIAL_USAGE_LANE = 9,// 紧急停车带
      CLIMBING = 10, // 爬坡车道
      ESCAPE = 11, // 避险车道
      DEDICATED_CUSTOMS = 12, // 海关专用车道
      VIEWING_PLATFROM = 13, // 观景台车道
      PARALLEL_LANE = 14, // 平行车道
      DIVERSION = 17, // 导流区车道
      PARKING = 19, // 停车车道
      LEFT_TURN_AREA = 20, // 左转等待车道
      VARIABLE_LANE = 21, // 可变车道
      NON_MOTOR_LANE = 22, // 非机动车道
      BUS_STOP = 23, // 公交停靠站
      NO_ENTRY_LANE = 24, // 不可行驶车道
      U_TURN_LANE = 25, // 掉头车道
      RIGHT_TURN_LANE = 26, // 右转专用车道
      ETC_LANE = 28, // ETC车道
      MANUAL_TOLL_LANE = 29, // 人工收费车道
      PLAZA_LANE = 30, // 收费站外广场车道
      HARBOR_STOP = 31, //港湾停靠站
      RIGHT_TURN_AREA = 32, //右转等待车道
      U_TURN_AREA = 33, //掉头等待车道
      NO_TURN_AREA = 34, //直行等待车道
      VIRTUAL_CONNECTED_LANE = 35 //虚拟连接车道
};

struct LaneAttrSeg
{
    float start_offset = 0.0f;
    float end_offset = 0.0f;
    uint32_t type = 0;
    float max_speed_limit = 0.0f;
    float min_speed_limit = 0.0f;
    float recommend_speed_limit = 0.0f;
    uint32_t transit = 0;
};

struct LaneGeoSeg
{
    uint64_t id = 0;
    float lane_width = 0.0f;
    float start_offset = 0.0f;
    float end_offset = 0.0f;
    uint32_t seg_lane_idx = 0;
    uint32_t center_line_type = 0;
    uint32_t contributing_sensors = 0;
    GeometryLine geometry;
    float reserved_1 = 0.0f;
    float reserved_2 = 0.0f;
};

struct LaneTopological
{
    uint32_t start_lane_id = 0;
    uint64_t start_seg_lane_id = 0;
    uint32_t end_lane_id = 0;
    uint64_t end_seg_lane_id = 0;
    bool start_valid = false;  ///start_lane_id 有效标志位
    bool end_valid = false;    ///end_lane_id 有效标志位
};

struct Lane
{
    uint32_t id = 0;
    uint32_t type = 0;
    uint32_t direction = 0;
    uint32_t position = 0;
    float dist_2_ego = 0.0f;
    float start_offset = 0.0f;
    float end_offset = 0.0f;
    bool is_start_out_of_range = true;
    bool is_end_out_of_range = true;
    LaneTopological lane_topological;
    uint32_t lane_attr_segs_num = 0;
    std::vector<LaneAttrSeg> lane_attr_segs = {};
    uint32_t front_lane_geo_segs_num = 0;
    std::vector<LaneGeoSeg> front_lane_geo_segs = {};
    uint32_t rear_lane_geo_segs_num = 0;
    std::vector<LaneGeoSeg> rear_lane_geo_segs = {};
    uint32_t left_lane_marker_segs_num = 0;
    std::vector<uint32_t> left_lane_marker_segs_ids = {};
    uint32_t right_lane_marker_segs_num = 0;
    std::vector<uint32_t> right_lane_marker_segs_ids = {};

    uint64_t road_id = 0;
    std::vector<uint64_t> previous_lane_ids;
    std::vector<uint64_t> next_lane_ids;
    uint8_t connect_type=0;
    bool is_lane_connect = false;


    void ClearSamplingPoints()
    {
        for (auto &geo_seg : front_lane_geo_segs)
            geo_seg.geometry.ClearSamplingPoints();

        for (auto &geo_seg : rear_lane_geo_segs)
            geo_seg.geometry.ClearSamplingPoints();
    }
};

} // namespace env_model
} // namespace message
} // namespace cem

#endif
