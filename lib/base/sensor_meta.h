#ifndef SENSOR_META_H_
#define SENSOR_META_H_

#include <string>

namespace cem {
namespace fusion {

enum class SensorType
{
    UNKNOWN_SENSOR_TYPE = -1,
    FVCM_LANE_MARKER = 0,
    FVCM_ROAD_EDGE = 1,
    LOC_DAISCH = 2,
    LOC_ASENSING_IPS = 3,
    LOC_SAIC_POSITION = 4,
    EGO_VEHICLE_STATUS = 5,
    HD_MAP_ZHT_POSE = 6,
    HD_MAP_ZHT_INFO = 7,
    HD_MAP_ZHT_MODEL = 8,
    PANORAMIC = 9,
    RADAR = 10,
    LUMINAR = 11,
    FUSED_OBSTACLES = 12,
    HD_MAP_BD_SCENE = 13,
    HD_MAP_BD_LANE = 14,
    GTR = 15,
    OLA = 16,
    ROAD_EDGE = 17,
    SENSOR_TYPE_NUM
};

enum class SensorOrientation
{
    FRONT = 0,
    LEFT_FORWARD = 1,
    LEFT = 2,
    LEFT_BACKWARD = 3,
    REAR = 4,
    RIGHT_BACKWARD = 5,
    RIGHT = 6,
    RIGHT_FORWARD = 7,
    PANORAMIC = 8
};

struct SensorInfo
{
    std::string name = "UNKNONW_SENSOR";
    SensorType type = SensorType::UNKNOWN_SENSOR_TYPE;
    SensorOrientation orientation = SensorOrientation::FRONT;
    std::string frame_id = "UNKNOWN_FRAME_ID";

    SensorInfo(const std::string &sensor_name, const SensorType &sensor_type,
               const SensorOrientation &sensor_orientation,
               const std::string &sensor_frame_id)
        : name(sensor_name),
          type(sensor_type),
          orientation(sensor_orientation),
          frame_id(sensor_frame_id)
    {
    }

    void Reset()
    {
        name = "UNKNONW_SENSOR";
        type = SensorType::UNKNOWN_SENSOR_TYPE;
        orientation = SensorOrientation::FRONT;
        frame_id = "UNKNOWN_FRAME_ID";
    }
};

} // namespace fusion
} // namespace cem

#endif
