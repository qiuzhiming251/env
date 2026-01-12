#pragma once

#include "common/utility.h"
#include "lib/common/log_custom.h"
#include "lib/message/sensor/camera/bev_lane/bev_lane.h"
#include "lib/perception_and_ld_map_fusion/data_preprocessor/bev_map_preprocessor.h"

using namespace cem::message::sensor;

namespace cem {
namespace fusion {
struct BevHarborTopology{
    uint64_t split_main_id;             // split拓扑主路
    uint64_t merge_main_id;             // merge拓扑主路
    std::set<uint64_t> topo_sub_ids;    // merge拓扑和split拓扑共同的子路
    uint64_t sub_left_id;               // 左侧子路车道
    uint64_t sub_right_id;              // 右侧子路车道
    uint64_t harbor_lane_id;            // 港湾车道id
    std::pair<double, double> range;    // 感知港湾车道范围[start, end]
    std::string harbor_position;        // harbor车道位置 "none" "most_left" "most_right"
};
struct SplitTopo{
    int      count          = 1;
    uint64_t main_lane      = 0;
    uint64_t normal_lane    = 0;
    uint64_t split_lane     = 0;

    std::set<uint64_t> split_lane_ids;

    SplitTopoExtendType     main_lane_type      = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
    SplitTopoExtendType     normal_lane_type    = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
    SplitTopoExtendType     split_lane_type     = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;

    bool operator==(const SplitTopo& other) const{
        return  main_lane == other.main_lane &&
               normal_lane == other.normal_lane &&
               split_lane == other.split_lane &&
               main_lane_type == other.main_lane_type &&
               normal_lane_type == other.normal_lane_type &&
               split_lane_type == other.split_lane_type;
    }

    void operator()(const SplitTopo& other)
    {
        count           = 1;
        main_lane       = other.main_lane;
        normal_lane     = other.normal_lane;
        split_lane      = other.split_lane;
        main_lane_type  = other.main_lane_type;
        normal_lane_type= other.normal_lane_type;
        split_lane_type = other.split_lane_type;
    }
};
struct MergeTopo{
    int      count          = 1;
    uint64_t main_lane      = 0;
    uint64_t normal_lane    = 0;
    uint64_t merge_lane     = 0;

    std::set<uint64_t> merge_lane_ids;

    MergeTopoExtendType     main_lane_type       = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
    MergeTopoExtendType     normal_lane_type     = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
    MergeTopoExtendType     merge_lane_type      = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;

    bool operator==(const MergeTopo& other) const{
        return  main_lane == other.main_lane &&
               normal_lane == other.normal_lane &&
               merge_lane == other.merge_lane &&
               main_lane_type == other.main_lane_type &&
               normal_lane_type == other.normal_lane_type &&
               merge_lane_type == other.merge_lane_type;
    }

    void operator()(const MergeTopo& other){
        count           = 1;
        main_lane       = other.main_lane;
        normal_lane     = other.normal_lane;
        merge_lane      = other.merge_lane;
        main_lane_type  = other.main_lane_type;
        normal_lane_type= other.normal_lane_type;
        merge_lane_type = other.merge_lane_type;
    }
};
class LaneTypeChecker {
public:

    LaneTypeChecker();
    ~LaneTypeChecker();

    void UpdateBevTopologyLaneType(std::shared_ptr<cem::message::sensor::BevMapInfo>& bev_info,
                                   const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map,
                                   const std::function<std::vector<const BevSplitInfo*>(uint64_t)>& LaneSplitGetter,
                                   const std::function<std::vector<const BevMergeInfo*>(uint64_t, bool)>& LaneMergeGetter);

private:

    void KeeperClear();

    void KeeperRemove();

    void KeeperCounter();

    void KeeperDebugInfo();

    void KeeperGetFrameTopos();

    void CheckSplitLaneType();

    void CheckMergeLaneType();

    bool CheckTopoUseSemanticInfo();

    bool HarborTopologyOperator();

    navigation::HarborStopInfo GetHarborStopInfo();

    bool CheckTopoUseMatchers(const uint64_t topo_lane_id, const int type);

    void UpdateKeeperStatus(const int type);

    bool buildTopoTypeUpdate(const uint64_t main_lane_id, 
                             const uint64_t sub_lane_1_id, 
                             const uint64_t sub_lane_2_id, 
                             const int type);

    uint64_t GetAdjoinLaneUseGeos(const uint64_t query_id,
                                  const uint64_t sub_lane_1_id,
                                  const uint64_t sub_lane_2_id,
                                  std::string& type);

    bool GetKeeperStatus(const uint64_t main_lane_id,
                         const uint64_t normal_lane_id,
                         const uint64_t topo_lane_id,
                         const int type);

    bool CheckTopoUseSpecialLane(const uint64_t sub_lane_1_id,
                                 const uint64_t sub_lane_2_id,
                                 const int type);

    bool CheckTopoUseLeftAdjoinLane(const uint64_t m_lane_id,
                                    const uint64_t sub_lane_1_id,
                                    const uint64_t sub_lane_2_id,
                                    const int type);

    bool CheckTopoUseSlope(const uint64_t main_lane_id,
                           const uint64_t sub_lane_1_id,
                           const uint64_t sub_lane_2_id,
                           const int type);

    void GetBevLaneGeos(uint64_t bev_lane_id,
                        std::vector<Eigen::Vector2f>& bev_lane_pts);

    void GetMapLaneGeos(std::vector<uint64_t> m_group_ids,
                        std::vector<Eigen::Vector2f>& map_lane_pts);

    bool BevTopoIsValid(const uint64_t main_lane_id,
                        const uint64_t sub_lane_1_id,
                        const uint64_t sub_lane_2_id,
                        const int type);

    bool GetVetorialAngle(std::vector<Eigen::Vector2f>& main_lane,
                          std::vector<Eigen::Vector2f>& sub_lane,
                          const int type,
                          std::pair<float, float>& angle);

    bool CheckMatchInfo(std::vector<const BevSplitInfo*>& split_match_infos,
                        std::vector<const BevMergeInfo*>& merge_match_infos,
                        const int type);

    bool CheckHorizontalDisMatchInfo(std::vector<const BevSplitInfo*>& split_match_infos,
                                     std::vector<const BevMergeInfo*>& merge_match_infos,
                                     const int type);

    bool CheckPositionMatchInfo(std::vector<const BevSplitInfo*>& split_match_infos,
                                std::vector<const BevMergeInfo*>& merge_match_infos,
                                const int type);
private:

    std::unordered_map<SplitTopoExtendType, std::string> split_type{
       {SplitTopoExtendType::TOPOLOGY_SPLIT_NONE, "SplitTopoExtendType::TOPOLOGY_SPLIT_NONE"},
       {SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT, "SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT"},
       {SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT, "SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT"},
       {SplitTopoExtendType::TOPOLOGY_SPLIT_UNKNOWN, "SplitTopoExtendType::TOPOLOGY_SPLIT_UNKNOWN"}
    };

    std::unordered_map<MergeTopoExtendType, std::string> merge_type{
       {MergeTopoExtendType::TOPOLOGY_MERGE_NONE, "MergeTopoExtendType::TOPOLOGY_MERGE_NONE"},
       {MergeTopoExtendType::TOPOLOGY_MERGE_LEFT, "MergeTopoExtendType::TOPOLOGY_MERGE_LEFT"},
       {MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT, "MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT"},
       {MergeTopoExtendType::TOPOLOGY_TO_BE_MERGED, "MergeTopoExtendType::TOPOLOGY_TO_BE_MERGED"},
       {MergeTopoExtendType::TOPOLOGY_MERGE_UNKNOWN, "MergeTopoExtendType::TOPOLOGY_MERGE_UNKNOWN"}
    };

    int keeper_type = 0;
    const int split_mode        = 1;
    const int merge_mode        = 2;

    const int threshold         = 5;
    const float max_keep_offset = 80.0;
    std::vector<SplitTopo> split_keeper;
    std::vector<MergeTopo> merge_keeper;

    std::vector<SplitTopo> frame_split_topos;
    std::vector<MergeTopo> frame_merge_topos;

    std::vector<SplitTopo> prev_frame_split_topos;
    std::vector<MergeTopo> prev_frame_merge_topos;

    std::shared_ptr<cem::message::sensor::BevMapInfo> data_;
    std::shared_ptr<cem::message::env_model::RoutingMap> ld_map_;

    std::function<std::vector<const BevSplitInfo*>(uint64_t)> SplitGetter;
    std::function<std::vector<const BevMergeInfo*>(uint64_t, bool)> MergeGetter;

};

}
}
