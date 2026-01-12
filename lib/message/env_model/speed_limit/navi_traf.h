#ifndef NAVI_TRAF_H_
#define NAVI_TRAF_H_

#include "message/common/header.h"

namespace cem {
namespace message {
namespace env_model {

struct NaviTrafficInfo
{
    cem::message::common::Header header;
    uint32_t spd_lmt_speed_value = 0; //当前道路基础限速值 单位:km/h
    uint32_t road_class = 0; //当前自车所在道路等级 0x0=高速路 0x1=国道 0x2=省道 0x3=郊区道路 0x4=乡村道路 0x5=县级道路 0x6=城市快速路 0x7=主路 0x8=次干路 0x9=支路 0xA=非导航道路 0xFF=无信息
};

} // namespace env_model
} // namespace message
} // namespace cem

#endif
