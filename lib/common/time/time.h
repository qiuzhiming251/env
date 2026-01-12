#ifndef TIME_H_
#define TIME_H_

#include "middleware/middleware_util.h"
#include "TimeCostTool.h"

namespace cem {
namespace fusion {

#ifdef MW_TYPE_DDS
inline MW_TIME GetMwTimeNow() { return MW_TIME_NOW; }
inline double GetMwTimeNowSec() { return MW_TIME_NOW_SEC; }
#elif MW_TYPE_CyberRT
inline MW_TIME GetMwTimeNow() { return MW_TIME_NOW; }
inline double GetMwTimeNowSec() { return MW_TIME_NOW_SEC; }


#endif


} // namespace fusion
} // namespace cem

#endif
