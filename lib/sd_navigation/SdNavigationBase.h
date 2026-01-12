#ifndef SDNAVIGATIONBASE_H
#define SDNAVIGATIONBASE_H

#include <map>
#include <base/params_manager/params_defination/internal_params.h>
#include <base/params_manager/params_manager.h>
#include "lib/perception_and_ld_map_fusion/fusion_manager.h"

namespace cem {
namespace fusion {
namespace navigation {

class SdNavigationBase {

  public:
      virtual void Proc(const std::shared_ptr<RoutingMap> &raw_routing_map, BevMapInfoConstPtr &raw_bev_map, BevMapInfoPtr &GlobalBevMapOutPut,
                std::vector<std::pair<uint64_t, std::pair<int, int>>>& guide_lane_result, std::vector<uint64_t>& sd_guide_lane_ids, AccLaneInfo  &sd_acc_lane_info){
          return;
      }

};
}
}
}


#endif //SDNAVIGATIONBASE_H
