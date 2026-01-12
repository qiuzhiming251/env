#ifndef COMMUNICATION_PARAMS_H_
#define COMMUNICATION_PARAMS_H_

#include <string>

namespace cem {
namespace fusion {

struct CommunicationParams
{
    std::string traffic_road_hmi_topic = "/cem/fusion/traffic_road_hdm";
    std::string rviz_traffic_road_hmi_topic = "/viz/fusion/traffic_road_hdm";
    bool enable_traffic_road_hmi_output = false;
    bool enable_traffic_road_hmi_points_output = false;
    bool enable_reset_subs_on_parking = false;
};

} // namespace fusion
} // namespace cem

#endif
