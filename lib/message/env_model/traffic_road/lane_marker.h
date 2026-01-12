#ifndef LANE_MARKER_H_
#define LANE_MARKER_H_

#include <vector>
#include <list>

#include "geometry_line.h"

namespace cem {
namespace message {
namespace env_model {

enum BaiduLaneMarkerColor
{
    BD_LMC__NONE = 0,
    BD_LMC__OTHER = 1,
    BD_LMC__WHITE = 2,
    BD_LMC__YELLOW = 3,
    BD_LMC__ORANGE = 4,
    BD_LMC__RED = 5,
    BD_LMC__BLUE = 6
};

enum BaiduLaneMarkerType
{
    BD_LMT__UNKNOWN = 0,
    BD_LMT__NONE = 1, // compensation line
    BD_LMT__SOLID = 2,
    BD_LMT__DASHED = 3,
    BD_LMT__DOUBLE_SOLID = 4,
    BD_LMT__DOUBLE_DASHED = 5,
    BD_LMT__LEFT_SOLID_RIGHT_DASHED = 6,
    BD_LMT__RIGHT_SOLID_LEFT_DASHED = 7,
    // BD_LMT__VIRTUAL = 8,     // virtual line
    BD_LMT__VIRTUAL = 11,     // virtual line 0321
    BD_LMT__NEW_NONE = 15,   // new compensation line
    BD_LMT__NEW_VIRTUAL = 16 // new virtual line
};

enum BaiduLaneMarkerSubType
{
    BD_LMST__NONE = 0,
    BD_LMST__SHORT_COLUMN = 0x01,
    BD_LMST__FENCE = 0x02,
    BD_LMST__CONE = 0x04,
    BD_LMST__JUMA = 0x08,
    BD_LMST__SLOW_DOWN_SIGN = 0x10,
    BD_LMST__DIVERSION = 0x20,
    BD_LMST__SPECIAL = 0x40
};

enum TrfLaneMarkerColor
{
    TRF_LMC__UNKNOWN = 0,
    TRF_LMC__WHITE = 1,
    TRF_LMC__YELLOW = 2,
    TRF_LMC__BLUE = 3,
    TRF_LMC__RED = 4,
    TRF_LMC__OTHERS = 5
};

enum TrfLaneMarkerType
{
    TRF_LMT__UNKNOWN = 0,
    TRF_LMT__SOLID = 1,
    TRF_LMT__DASHED = 2,
    TRF_LMT__DOUBLE_SOLID_DASHED = 3,
    TRF_LMT__DOUBLE_DASHED_SOLID = 4,
    TRF_LMT__DOUBLE_SOLID_SOLID = 5,
    TRF_LMT__DOUBLE_DASHED_DASHED = 6,
    TRF_LMT__BOTTS = 7,
    TRF_LMT__DECELERATION = 8,
    TRF_LMT__OTHERS = 9,
    TRF_LMT__COMP = 10,
    TRF_LMT__VIRTUAL = 11,
    TRF_RMT__CROSSWALK = 12,           // 人行道
    TRF_RMT__STRAIGHT = 13,            // 直行
    TRF_RMT__LEFT = 14,                // 左转
    TRF_RMT__RIGHT = 15,               // 右转
    TRF_RMT__TURNING = 16,             // 掉头
    TRF_RMT__STRAIGHT_LEFT = 17,       // 直行左转
    TRF_RMT__STRAIGHT_RIGHT = 18,      // 直行右转
    TRF_RMT__STRAIGHT_LEFT_RIGHT = 19, // 直行左转右转
    TRF_RMT__LEFT_RIGHT = 20,          // 左转右转
    TRF_RMT__STRAIGHT_TURNING = 21,    // 直行掉头
    TRF_RMT__LEFT_TURNING = 22,        // 左转掉头
    TRF_RMT__IMPORT = 23,              // 汇入
    TRF_RMT__EXPORT = 24,              // 汇出
    TRF_RMT__STOPLINE = 25             // 停止线
};

enum TrfLaneMarkerSubType
{
    TRF_LMST__NONE = 0,
    TRF_LMST__SHORT_COLUMN = 1,
    TRF_LMST__FENCE = 2,
    TRF_LMST__CONE = 3,
    TRF_LMST__JUMA = 4,
    TRF_LMST__SLOW_DOWN_SIGN = 5,
    TRF_LMST__DIVERSION = 6,
    TRF_LMST__SPECIAL = 7
};

enum TrfLaneMarkerPosition
{
    TRF_LMP__UNKNOWN = 0,
    TRF_LMP__FIRST_LEFT = 1,
    TRF_LMP__SECOND_LEFT = 2,
    TRF_LMP__THIRD_LEFT = 3,
    TRF_LMP__FOURTH_LEFT = 4,
    TRF_LMP__FIFTH_LEFT = 5,
    TRF_LMP__SIXTH_LEFT = 6,
    TRF_LMP__SEVENTH_LEFT = 7,
    TRF_LMP__FIRST_RIGHT = 8,
    TRF_LMP__SECOND_RIGHT = 9,
    TRF_LMP__THIRD_RIGHT = 10,
    TRF_LMP__FOURTH_RIGHT = 11,
    TRF_LMP__FIFTH_RIGHT = 12,
    TRF_LMP__SIXTH_RIGHT = 13,
    TRF_LMP__SEVENTH_RIGHT = 14,
    TRF_LMP__OTHER = 15,
    TRF_LMP_FIRST_LEFT_RIGHT = 16,
    TRF_LMP_FIRST_RIGHT_LEFT = 17,
    TRF_LMP_NEW_EGO_LANE_ADD_LANE_LEFT = 18,
    TRF_LMP_NEW_EGO_LANE_ADD_LANE_RIGHT = 19
};

struct LaneMarkerAttrSeg
{
    uint32_t type = 0;
    uint32_t sub_type = 0;
    uint32_t color = 0;
    float start_offset = 0.0f;
    float end_offset = 0.0f;
    float type_conf = 0.0f;
    bool is_deceleration = false;
    bool is_waiting_area = false;
    bool is_far_end_new_line = false;
    bool is_split = false;
    bool is_merge = false;
    uint32_t split_merge_gid = 0;
};

struct LaneMarkerGeoSeg
{
    uint32_t id = 0;
    uint32_t contributing_sensors = 0;
    float width = 0.0f;
    float start_offset = 0.0f;
    float end_offset = 0.0f;
    GeometryLine geometry;
    uint32_t reserved_1 = 0;
    uint32_t reserved_2 = 0;
    uint32_t reserved_3 = 0;
    uint32_t reserved_4 = 0;
};

struct LaneMarker
{
    uint32_t id = 0;
    uint32_t position = 0;
    bool is_start_out_of_range = false;
    bool is_end_out_of_range = false;
    float end_offset = 0.0f;
    bool is_constrained_by_roadedge = false;
    uint32_t additional_location = 0;
    bool is_associated_by_f100c_in_vision_pre_fusion = false;
    bool is_completed_new_lane_marker = false;
    uint32_t associated_bev_id = 0;
    std::vector<uint8_t> associated_sensors_lane_markers_ids = {};
    uint32_t lane_marker_attr_segs_num = 0;
    std::list<LaneMarkerAttrSeg> lane_marker_attr_segs = {};
    uint32_t front_lane_marker_geo_segs_num = 0;
    std::vector<LaneMarkerGeoSeg> front_lane_marker_geo_segs = {};
    uint32_t rear_lane_marker_geo_segs_num = 0;
    std::vector<LaneMarkerGeoSeg> rear_lane_marker_geo_segs = {};
    bool is_core_position = true;
    bool is_lanemarker_connect = false;
    void ClearSamplingPoints()
    {
        for (auto &geo_seg : front_lane_marker_geo_segs)
            geo_seg.geometry.ClearSamplingPoints();

        for (auto &geo_seg : rear_lane_marker_geo_segs)
            geo_seg.geometry.ClearSamplingPoints();
    }

    void SetId(size_t new_id) { id = new_id; }
};

} // namespace env_model
} // namespace message
} // namespace cem

#endif
