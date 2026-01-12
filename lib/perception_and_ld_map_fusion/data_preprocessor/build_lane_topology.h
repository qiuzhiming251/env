#ifndef _BUILD_LANE_TOPOLOFY_H_
#define _BUILD_LANE_TOPOLOFY_H_

#include "lib/common/utils/lane_geometry.h"
#include "lib/message/env_model/routing_map/routing_map.h"
#include "lib/perception_and_ld_map_fusion/data_preprocessor/data_preprocessor.h"
#include "lib/perception_and_ld_map_fusion/data_preprocessor/ld_map_preprocessor.h"
#include "lib/perception_and_ld_map_fusion/data_preprocessor/bev_map_preprocessor.h"

#include "fitter/bspline_segment.h"
#include "traffic_flow_manager.h"

namespace cem {
namespace fusion {

typedef struct bev_map_ids_info_s
{
    std::map<uint64_t, std::vector<uint64_t>> local_map_matched_ids;
    std::unordered_map<uint64_t, uint32_t> lane_id_to_group_id_;
    const cem::message::env_model::LaneInfo* bev_eog_id_map_lane_info = nullptr;
    //根据感知bev id查找到地图split和merge属性
    std::vector<struct BevSplitInfo> ldmap_split_info;
    std::vector<struct BevMergeInfo> ldmap_merge_info;
}bev_map_ids_info;

typedef struct map_id_point_s
{
    uint64_t map_id = 0;
    std::vector<Point> line_points;
}map_id_point;

typedef struct bev_important_line_point_s
{
    uint64_t bev_id = 0;
    size_t bev_index = 0; //使用的感知bev index
    std::vector<Point2DF> line_points;
    float avg_width = FLT_MAX;
    float max_width = FLT_MAX;
}bev_important_line_point;

const static uint64_t MAP_EGO_LANE_NEW_INTERNAL_TYPE = 1;
const static uint64_t MAP_MAIN_LANE_NEW_INTERNAL_TYPE = 2;
const static uint64_t MAP_SPLIT_LANE_NEW_INTERNAL_TYPE = 3;

typedef struct segment_map_as_bev_info_s
{
    uint64_t bev_split_type = 0;
    uint64_t new_bev_id = 0;//根据地图新生成bev id
    uint64_t origin_bev_id = 0; //使用的感知bev原有的id
    size_t origin_bev_index = 0; //使用的感知bev原有的index
    std::vector<Point> all_line_points;
    std::vector<Point> seg_line_points;
    std::vector<uint64_t> map_ld_list;
    uint32_t pts_idx = 0;
    Point split_point;
    bool use_map_bev = false; //default: bev
    bool ego_lane_split_topo_exist = false;
}segment_map_as_bev_info;

class BuildLaneTopology
{
private:
    const static int BUILD_TOPO_MERGE = 1;
    const static int BUILD_TOPO_SPLIT = 2;
    const static int BUILD_TOPO_NOA_MODE = 2;
    const static int BUILD_TOPO_ICC_MODE = 1;

    typedef struct index2value_s
    {
        int idx = -1;
        float val = 0.0f;
    }index2value;
    typedef struct Lane_ID_Storage_s
    {
        uint32_t source_lane_id = 0;
        uint32_t source_left_lanemarker_id = 0;
        uint32_t source_right_lanemarker_id = 0;
        uint32_t source_position = 0;
        uint32_t target_lane_id = 0;
        uint32_t target_position = 0;
        uint32_t left_lane_id = 0;
        uint32_t right_lane_id = 0;
        Eigen::Vector2f P0;
        double T0 = 0.0;
        Eigen::Isometry3d Twb0_r;
        bool first_remove = false;
        bool is_blocked = false;
        bool has_narrow_lane = false;
        bool has_origin_merge_topo = false;
        bool has_new_merge_topo = false;
        bool need_to_build_topo = false;
    }Lane_ID_Storage;
    typedef struct Ego_Lane_Crop_Info_s
    {
        int less_2dot5_both_lm_idx = -1;
        int less_2dot5_edge_left_lm_idx = -1;
        int less_2dot5_edge_right_lm_idx = -1;
        int dist_prev_cut_lm_idx = -1;
        //自车道太窄开始裁剪的点
        Eigen::Vector2f remove_start_point;
        //不能生成merge拓扑情况：最右边没有车道或导流线或实线。
        bool right_build_merge = false;
        //不能生成merge拓扑情况：最左边没有车道或导流线或实线。
        bool left_build_merge = false;
        bool has_narrow_lane = false;
        bool has_left_edge = false;
        bool has_right_edge = false;
    }Ego_Lane_Crop_Info;
    typedef struct Ego_Lane_Info_To_Build_Topo
    {
        uint32_t bev_ego_lane_id;
        uint32_t bev_ego_lane_left_lanemarker_id;
        uint32_t bev_ego_lane_right_lanemarker_id;
        uint32_t map_ego_lane_id;
        uint32_t map_main_lane_id;
        uint32_t map_sub_lane_id;
        std::vector<Eigen::Vector2f> bev_geos;
        std::vector<Eigen::Vector2f> bev_adjoin_lane_geos;
        std::vector<Eigen::Vector2f> map_main_lane_geos;
        std::vector<Eigen::Vector2f> map_sub_lane_geos;
        bool bev_ego_lane_no_topo = false;
        bool map_ego_lane_has_topo = false;
        bool map_split_point_valid = false;
        Eigen::Vector2f map_split_point;
        float bev_ego_lane_length;
        bool bev_ego_lane_is_on_wrong_way = false;
        std::string bev_adjoin_type = "";
        uint32_t bev_ego_adjoin_lane_id = 0;
        uint64_t new_lane_id = 0;
        Eigen::Vector2f segment_point = {999.0, 999.0};
        bool ready_to_build_split_topo = false;
    }To_Build_Split_Topo_Lane_Info;

public:

    BuildLaneTopology();
    ~BuildLaneTopology();

    void BuildNewMergeSplitTopo(
        BevMapInfoPtr local_map,
        std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
        const bev_map_ids_info & bev_map_ids);

    //自车道逐渐变窄消失构建merge拓扑到相邻车道
    void BuildMergeTopo(
        BevMapInfoPtr local_map,
        std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
        const bev_map_ids_info & bev_map_ids);
    // 创建SPLIT拓扑
    void BuildSplitTopo(BevMapInfoPtr local_map, std::shared_ptr<cem::message::env_model::RoutingMap> ld_map);

    //由于车流感知缺少导航主路车道车道线，根据车流构建split拓扑
    void BuildSplitTopoWithLdMap(
        BevMapInfoPtr local_map,
        std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
        const bev_map_ids_info & bev_map_ids);

    //校验感知Merge拓扑
    void CheckMergeTopo(BevMapInfoPtr &bevMapPtr,RoutingMapPtr &routingMapPtr,std::unordered_map<uint64_t, std::vector<uint64_t>>& bev_ld_match);

    //校验感知Split拓扑
    void CheckSplitTopo(BevMapInfoPtr &bevMapPtr,RoutingMapPtr &routingMapPtr, const bev_map_ids_info &bev_map_ids);

    //校验感知、地图匹配是否正确
    bool CheckMatchRes(BevMapInfoPtr &bevMapPtr,RoutingMapPtr &routingMapPtr,std::unordered_map<uint64_t, std::vector<uint64_t>>& bev_ld_match);

    //判断LD地图是否处于低精区域
    bool isLDMapLowPrecision();

    //根据map_event判断是否快到高快收费站
    bool isNearToToll();

private:
    //限定构建拓扑在高快和城市道路，但乡村小道和地库等不构建。
    bool CheckValidRoadClass();

    bool isICCmode();

    void TransPoint(Eigen::Vector3d & P0, const Eigen::Isometry3d &rotate_translate_matrix);
    Eigen::Isometry3d DR2BodyTrans(const double &timestamp);
    /*根据第一个时间戳T0和开始点P0，当前时间戳Tn，转换获得当前Pn点。*/
    Eigen::Vector2f getStartPointInDR(const double &Tn, const Lane_ID_Storage & storage);

    uint32_t generateNewLaneID(BevMapInfoPtr local_map);

    uint32_t generateNewLaneMarkerID(BevMapInfoPtr local_map);

    static bool cmp(const index2value & a, const index2value & b)
    {
        return a.val < b.val;
    }

    float Point2LineDistanece(const Eigen::Vector2f &point,const std::vector<Eigen::Vector2f> & line);

    float GetCurrentSpeed();

    void OtherLineWidthSorting(std::vector<bev_important_line_point> & bilp);

    //车道线切割为一分为二,front
    void SegmentLaneTwoParts(
        BevMapInfoPtr local_map,
        const int & segment_lane_id,
        const Eigen::Vector2f & segment_point,
        const bool & is_on_topo,
        int & new_lane_id);

    bool RemoveInvalidLaneSeg(
        BevMapInfoPtr local_map,
        const int & ego_lane_id,
        const Eigen::Vector2f & removed_start_point,
        const std::set<int> &target_lanemarker_ids,
        Eigen::Vector2f & end_point);

    bool JudgeTargetLaneIfAvail(
        BevMapInfoPtr local_map,
        const float &current_speed,
        const std::vector<Eigen::Vector2f> & left_edge_line,
        const std::vector<Eigen::Vector2f> & right_edge_line,
        const int & target_lane_id);

    bool CheckBevTopoAndTargetLaneConsistency(
        BevMapInfoPtr local_map,
        const int & target_lane_id);

    bool CheckLDMapIfTopoExist(
        BevMapInfoPtr local_map,
        std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
        const bev_map_ids_info & bev_map_ids);

    bool JudgeIfBuildMergeTopo(
        BevMapInfoPtr local_map,
        const std::vector<Eigen::Vector2f> &source_line,
        const std::vector<Eigen::Vector3f> & src_left_lm_line,
        const std::vector<Eigen::Vector3f> & src_right_lm_line,
        const std::vector<Eigen::Vector2f> & left_edge_line,
        const std::vector<Eigen::Vector2f> & right_edge_line,
        const float &current_speed,
        float & lane_width_limit,
        Lane_ID_Storage &store_merge_topo,
        Ego_Lane_Crop_Info & result);

    float GetDistPointLane(
        const Eigen::Vector2f &point_a,
        const Eigen::Vector2f &point_b,
        const Eigen::Vector2f &point_c);

    bool ComputeEgoToAdjacentLaneWidth(
        BevMapInfoPtr local_map,
        Lane_ID_Storage &store_merge_topo,
        const int & valid_count,
        Ego_Lane_Crop_Info & result,
        int & target_lane_id,
        bool &has_narrow_lane,
        bool &is_selected_adjacent_left_lane,
        bool &is_selected_adjacent_right_lane,
        std::vector<Eigen::Vector2f> & widths);

    bool ComputeTwoLanesWidth(
        BevMapInfoPtr local_map,
        const int & src_lane_id,
        const float &current_speed,
        const int & src_left_lanemarker_id,
        const int & src_right_lanemarker_id,
        std::vector<Eigen::Vector2f> & widths);

    bool Computelane2EdgeWidth(
        BevMapInfoPtr local_map,
        const std::vector<Eigen::Vector2f> &source_line,
        const std::vector<Eigen::Vector2f> & left_edge_line);

    void GetBuildTopoLaneInfo(
        BevMapInfoPtr local_map,
        std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
        To_Build_Split_Topo_Lane_Info& to_build_topo_lane_info);

    bool IsNotRunningOnNaviLane(
        std::vector<Eigen::Vector2f> bev_geos,
        std::vector<Eigen::Vector2f> map_main_lane_geos,
        std::vector<Eigen::Vector2f> map_sub_lane_geos,
        Eigen::Vector2f map_split_point);

    void GetBevEgoAdjoinLane(
        BevMapInfoPtr local_map,
        std::vector<Eigen::Vector2f> bev_geos,
        Eigen::Vector2f map_split_point,
        std::string bev_adjoin_type,
        uint32_t& bev_ego_adjoin_lane_id,
        Eigen::Vector2f segment_point,
        std::vector<Eigen::Vector2f> adjoin_lane_geos);

    bool SegmentBevEgoLane(
        BevMapInfoPtr local_map,
        const uint64_t target_lane_id,
        const double distance_to_ego,
        uint64_t& new_lane_id,
        std::pair<cem::message::common::Point2DF, cem::message::common::Point2DF> target_bev_line);

    void LaneEdgeChecking(
        BevMapInfoPtr local_map,
        std::vector<map_id_point> &ego_lane_points,
        std::vector<map_id_point> &split_lane_points,
        std::vector<map_id_point> &main_lane_points,
        bool &edge_exist);

    void BevAndMapChecking(
        BevMapInfoPtr local_map,
        std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
        const bev_map_ids_info & bev_map_ids,
        bool &is_need_build);

    bool LineObjectsChecking(
        std::unordered_map<uint32_t, FusionObjs> & traffic_flow,
        std::vector<map_id_point> &lane_points);

    void trafficFlowChecking(
        std::unordered_map<uint32_t, FusionObjs> & traffic_flow,
        std::vector<map_id_point> &ego_lane_points,
        std::vector<map_id_point> &split_lane_points,
        std::vector<map_id_point> &main_lane_points,
        bool &traffic_flow_exist);

    void GetMapTopoLinePoints(
        BevMapInfoPtr local_map,
        std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
        const bev_map_ids_info & bev_map_ids,
        std::vector<map_id_point> &ego_lane_points,
        std::vector<map_id_point> &split_lane_points,
        std::vector<map_id_point> &main_lane_points,
        bool & is_split_left,
        const float & max_len);

    void GetMapNewSegment(
        BevMapInfoPtr local_map,
        std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
        const bev_map_ids_info & bev_map_ids,
        const Eigen::Vector2f &segment_point,
        const float & max_len,
        int & new_lane_id);

    void GetMapSegmentPointForNoTopo(
        BevMapInfoPtr local_map,
        std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
        const bev_map_ids_info & bev_map_ids,
        std::vector<map_id_point> & ego_lane_points,
        std::vector<map_id_point> & split_lane_points,
        std::vector<map_id_point> & main_lane_points,
        std::vector<MotionObject2Lane> & objstacleInfoForLdMap,
        std::vector<segment_map_as_bev_info> & segment_info);

    void GetMapSegmentPointForSplit(
        BevMapInfoPtr local_map,
        std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
        const bev_map_ids_info & bev_map_ids,
        const bool & map_split_has_motion_objs,
        std::vector<map_id_point> & ego_lane_points,
        std::vector<map_id_point> & split_lane_points,
        std::vector<map_id_point> & main_lane_points,
        std::vector<MotionObject2Lane> & objstacleInfoForLdMap,
         std::vector<segment_map_as_bev_info>  & segment_info);

    void GetMapEgoIdByMatch(
        BevMapInfoPtr local_map,
        std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
        const bev_map_ids_info & bev_map_ids,
        uint64_t & map_ego_lane_id);

    std::vector<Point> GetMapPointByBevLaneId(
        uint64_t map_lane_id,
        std::shared_ptr<cem::message::env_model::RoutingMap> ld_map);

    std::vector<Point2D> GetMapPointByBevLanemarkerId(
        uint64_t map_lanemarker_id,
        std::shared_ptr<cem::message::env_model::RoutingMap> ld_map);

    //根據postion找到左右车道，然后找到左车道的右车道线，右车道的左车道线，作为自车道的左右车道线
    void GetLaneMarkerBasedPosition(
        uint32_t bev_ego_id,
        std::vector<cem::message::sensor::BevLaneInfo>::iterator left_side_lane_ptr,
        std::vector<cem::message::sensor::BevLaneInfo>::iterator right_side_lane_ptr,
        BevMapInfoPtr &bevMapPtr,
        std::vector<pair<uint32_t,std::vector<Eigen::Vector2f>>>& lmk_geos);

    //根据到对应的自车道中心线的距离寻找，自车道的左右中心线
    void GetLaneMarkerBasedLd(
        uint32_t bev_id,
        std::shared_ptr<std::vector<Eigen::Vector2f>> &ld_ego_tmp_geos,
        BevMapInfoPtr &bevMapPtr,
        std::vector<pair<uint32_t,
        std::vector<Eigen::Vector2f>>>& lmk_geos);

    //左右车道和中心线其中一根线缺少一段进行补齐
    void AlignLaneMarker(
        BevMapInfoPtr &bevMapPtr,
        const int & target_lane_id,
        const int & bev_left_lanemarker_id,
        const int & bev_right_lanemarker_id,
        const Eigen::Vector2f &target_lane_last_pts,
        const Eigen::Vector2f &target_left_lanemarker_last_pts,
        const Eigen::Vector2f &target_right_lanemarker_last_pts);

    float current_speed = 0.0;
    bool is_ICC_mode = false;
private:
    //车道最小宽度2.5米
    float const LANE_MIN_DIST = 2.5f;
    float const LANEMARKER_2_EDGE_DIST = 2.5f;
    const float MERGE_TOPO_DIST = 20.0f;
    const float DIST_2_END_CUT = 10.0f;
    const float DETECT_LIMIT = 120;
    const float MERGE_TOPO_FINISHED_DISTANCE = 100.0f;
    const float SPLIT_TOPO_DISTANCE = 100.0f;
    const float SPLIT_TOPO_DIST = 30.0f;
    const float FRONT_MAX_RANGE = 150.0f;
    const float PnC_TIME_LIMIT = 1.2f; //规控锁定变道距离：时间1.2秒*车速（米每秒）
    const float NOT_BUILD_TOPO_DISTANCE = 300.0f; //快到收费站距离不构建拓扑（地图在收费站有很多split）

};

}
}
#endif
