#ifndef DEBUG_INFOS_H_
#define BASE_FILTER_H_
#include "message/internal_message.h"

namespace cem {
namespace fusion {
    class DebugInfos {
    public:
        DebugInfos() {};
        ~DebugInfos() {};
        void SetDebugInfo(const bool& used_ld_map, const BevMapInfoPtr& bev_map_info, Eigen::Isometry3d T_local_ego_, 
                          const RoutingMapPtr& ld_map_info);
        void SetBEVMapDebugInfo(const BevMapInfoPtr GlobalBevMapOutPut, Eigen::Isometry3d T_local_ego_);
        void SetLDMapDebugInfo(const RoutingMapPtr GlobalLDMapOutPut, const BevMapInfoPtr GlobalBevMapOutPut);
    }; 

}
}

#endif