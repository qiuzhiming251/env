#ifndef MAP_MATCH_H_
#define MAP_MATCH_H_

#include "message/common/header.h"

namespace cem {
namespace message {
namespace sensor {

enum struct LocationLevel {
  Nonelevel = 0,     ///<	无法定位
  Roadlevel = 1,     ///<	道路级定位
  Lanelevel = 2,     ///<	车道级定位
  Invaildlevel = 3,  ///<	位置信息不可使用（暂不使用）
  Roughlevel = 4,    ///<	粗略定位，用于第三方定位
};

struct MapMatchResult {
  bool valid;
  cem::message::common::Header header;
  LocationLevel location_level;
};

struct MapMatchResultBaidu {
  cem::message::common::Header header;
  LocationLevel location_level;
};

} // namespace sensor
} // namespace message
} // namespace cem

#endif
