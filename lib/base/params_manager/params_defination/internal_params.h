#ifndef INTERNAL_PARAMS_H_
#define INTERNAL_PARAMS_H_

#include <unordered_map>
#include <utility>
#include "common/enum.h"
#include "datacontainer_params.h"
#include "message/internal_message.h"

namespace cem {
namespace fusion {

struct InternalParams {
  BevDataContainer        raw_bev_data;
  SDMapDataContainer      sd_map_data;
  LDMapDataContainer      ld_map_data;
  NavigationInfoContainer navigation_info_data;
  BevLDMatchInfoContainer ld_match_info_data;
  GeosMatchMergeInfoContainer geos_match_merge_data;
};

}  // namespace fusion
}  // namespace cem

#endif
