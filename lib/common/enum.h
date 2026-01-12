#ifndef ENUM_H_
#define ENUM_H_

namespace cem {
namespace fusion {

enum class ContributingSensors
{
    FVCM = 1 << 0,
    LIDAR = 1 << 1,
    MONOCULAR_CAMERA = 1 << 2,
    FISH_EYE_CAMERA = 1 << 3,
    LIDAR_AND_MONOCULAR_FUSION = 1 << 4,
    LIDAR_AND_FISH_EYE_FUSION = 1 << 5,
    HDMAP = 1 << 6,
    TRAFFICFLOW = 1 << 7,
    OLA = 1 << 8,
    ROAD_EDGE = 1 << 9,
    FLE = 1 << 10,
    BEV = 1 << 11,
    VISION_PRE_FUSION = 1 << 12,
    OTHERS = 1 << 13
};

enum class MeasurementState
{
    DELETED = 0,
    NEW = 1,
    MEASURED = 2,
    PREDICTED = 3,
    INACTIVE = 4,
    VIRTUAL = 5,
    OTHERS
};

enum class HdmapZhtLineColor
{
    BLACK = 1,
    WHITE = 2,
    YELLOW = 3,
    ORANGE = 4,
    GREEN = 5,
    GRAY = 6,
    RED = 7,
    BLUE = 8
};

enum class HdmapZhtLineType
{
    UNKNOWN = 1,
    SINGLE_DASHED = 2,
    SINGLE_SOLID = 3,
    DOUBLE_DASHED = 4,
    DOUBLE_SOLID = 5,
    SOLID_DASHED = 6,
    DASHED_SOLID = 7,
    DIVERSION = 8,
    DUMMY_CROSS = 9,
    CHAIN_OBSTACLE = 10,
    GUIDE = 11
};

enum LANE_CHANGE
{
    LANE_CHANGE_UNKNOWN = 0,
    KEEP_LANE = 1,
    LEFT_CHANGE = 2,
    RIGHT_CHANGE = 3,
    LEFT_LEFT_CHANGE = 4,
    RIGHT_RIGHT_CHANGE = 5
};

///融合输入源切换原因
 enum class FusionInputChangeReason{
     default_value = 0,           ///默认数值
     mainroad_map2bev = 1,   ///主路 地图切感知
     ramp_bev2map = 2,       ///匝道 感知切地图
     ramp_map2bev = 3       ///匝道 地图与感知相差太大 地图切感知
 };

 enum class LastFusionInputType {
     unknown = 0,
     map  = 1,
     bev = 2,
     icc = 3
 };

} // namespace fusion
} // namespace cem

#endif
