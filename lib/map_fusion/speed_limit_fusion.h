#ifndef SPEED_LIMIT_FUSION_H_
#define SPEED_LIMIT_FUSION_H_

#include "common/CommonDataType.h"
#include "lib/common/utility.h"

namespace cem {
namespace fusion {


class CnoaSpeedLimit {
 public:
  CnoaSpeedLimit() = default;

  void Process(DetectBevMap &bev_map, const RoutingMapPtr routing_map_ptr);

  int CountPreGroupLaneNum(const SDLaneGroupInfo* lane_group_ptr, 
                           const std::unordered_map<uint64_t, const SDLaneGroupInfo*> &route_lane_groups,
                           int &round);

 private:
  const double kMaxOffset = 600.0;  //计算下一道路限速的最大搜索距离
};


enum VisionSpeedLimitType {
  sp_unknown = 0,           //未知
  sp_max_speed = 1,         //最高限速
  sp_min_speed = 2,         //最低限速
  sp_cancel_restriction = 3 //取消限速
};

enum SpeedLimitStatus {
  normal = 0x0,     //正常限速
  unknown = 0x1,    //没有输入
  unlimited = 0x2,  //不限速
  cancelled = 0x3,  //取消限速
  no_display = 0x4  //不显示
};

enum SpeedLimitMode {
  passive_mode = 0x0, //地图、视觉、导航限速无效
  fusion_mode = 0x1,  //只有视觉和导航限速
  map_mode = 0x2,     //只有导航限速
  vision_mode = 0x3,  //只有视觉限速
  hdmap_mode = 0x4    //有地图限速，直接采用地图限速
};

enum RoadClass {
  freeway = 0x0,              //高速路 
  national_highway = 0x1,     //国道 
  provincial_highway = 0x2,   //省道 
  suburban_roadway = 0x3,     //郊区道路 
  country_roadway = 0x4,      //乡村道路 
  country_highway = 0x5,      //县道 
  city_freeway = 0x6,         //城市快速路 
  primary_road = 0x7,         //主路 
  arterial_road = 0x8,        //次干路 
  access_road = 0x9,          //支路 
  non_navigation_road = 0xA,  //非导航道路 
  no_information = 0xFF,      //无信息
};

class SpeedLimitFusion {
 public:
  SpeedLimitFusion() = default;

  void Process(DetectBevMap &bev_map, const RoutingMapPtr routing_map_ptr);

  void UpdateHoldingDistance();

  void CalcAccumulatedDistance();

  void SetLaneSpeedLimit(DetectBevMap &bev_map);

 private:
  // input
  uint32_t EHR_speedlimit = 0;                                //LD地图输入限速(C平台)，LDlite地图输入限速(B平台)
  uint32_t vision_speedlimit_id = 0;                          //视觉输入限速牌id
  uint32_t vision_speedlimit_value = 0;                       //视觉输入限速值
  VisionSpeedLimitType vision_speedlimit_type = sp_unknown;   //视觉输入限速类型
  Eigen::Vector3d vision_speedlimit_position = {0,0,0};       //视觉输入限速牌位置(自车坐标系)
  uint32_t Navi_speedlimit = 0;                               //导航地图输入限速
  uint64_t function_support_status = 0;                       //TSR 故障输入，bit42=1表示相机故障(0x40000000000)
  double current_vehicle_speed = 0.0;                         //车速信息(m/s)
  RoadClass road_class = no_information;                      //导航地图道路等级

  // process
  uint32_t vision_filter_value = 0;         //滤波后的视觉限速
  bool holding_distance_update = false;     //是否更新保持阈值并且从零累计距离
  double holding_distance = 0.0;            //限速保持阈值(m)
  double accumulated_distance = 0.0;        //限速累积距离(m)
  uint32_t pre_EHR_speedlimit = 0;          //上一帧的地图限速值
  uint32_t pre_Navi_speedlimit = 0;         //上一帧的导航限速值
  uint32_t pre_vision_speedlimit = 0;       //上一帧的视觉限速
  uint32_t prepre_vision_speedlimit = 0;    //上上帧的视觉限速
  double cur_measurement_timestamp = 0.0;   //当前帧时间戳
  double pre_measurement_timestamp = 0.0;   //前一帧时间戳

  // output
  uint32_t speed_limit_value = 0;                   //ENV输出限速值(km/h)
  SpeedLimitStatus speed_limit_status = unknown;    //ENV输出限速类型
  SpeedLimitMode speed_limit_mode = passive_mode;   //ENV输出限速融合模式
  double lane_speed_limit = 0.0;                    //ENV输出车道中心线限速值(m/s)
};

}  // namespace fusion
}  // namespace cem

#endif  // SPEED_LIMIT_FUSION_H_
