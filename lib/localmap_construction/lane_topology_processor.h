#ifndef PROCESSLANESTOPO_H
#define PROCESSLANESTOPO_H
#include <vector>
#include "base/sensor_data_manager.h"
#include <common/utils/lane_geometry.h>
#include <base/params_manager/params_manager.h>
#include <message/sensor/camera/bev_lane/bev_lane.h>
#include "lib/message/env_model/routing_map/routing_map.h"
#include <base/params_manager/params_defination/internal_params.h>

namespace cem {
namespace fusion {

typedef std::shared_ptr<std::vector<Eigen::Vector2f>> Vec2fVector;
typedef cem::message::common::Point2DF Point2DF;

struct traverseCrossWalkLane {
  uint64_t far_lane_id;
  uint64_t near_lane_id;
  uint64_t cross_walk_id;
  bool     traverse = false;
};
class ProcessLanesTopo
{
    struct LineSort {
        LineSort(uint32_t ID, bool IsRoadBoundary) : id(ID), is_road_boundary(IsRoadBoundary) {}

        uint64_t id;
        bool     is_road_boundary;
    };
    struct laneDist {
        uint64_t    f_lane_id;
        uint64_t    n_lane_id;
        float       dist;
    };

	struct SurroundLaneInfo {
		uint64_t 			lane_id;
		std::set<uint64_t>	right_lane_ids;
		std::set<uint64_t>	left_lane_ids;
	};

public:
    static constexpr int     locking_count_                       = 5;
    static constexpr float   break_bev_lane_length_threshold_     = 10.0;
    static constexpr float   break_bev_lane_distance_threshold_   = 80.0;

    ProcessLanesTopo();

    void SetTopoExtendInfo(std::vector<message::sensor::BevLaneInfo> &vct_bev_lane_info);
    void SetBrokenTopoInfo(BevMapInfoPtr &bevMapPtrInput);
    void GetCrossParam(bool flag, double dist, bool turn_right) {
      cross_road_status_   = flag;
      cross_road_distance_ = dist;
      cross_turn_right_    = turn_right;
    };
    void SetAdc2Junction(double adc_to_junction) { adc_to_junction_ = adc_to_junction; }

    void GetTraverseCrossWalkLaneInfo(std::vector<traverseCrossWalkLane> &t_crosswalk_lane) {
      t_crosswalk_lane = traverse_crosswalk_lane_;
    };
    // 输出所有链式路径（DFS）
    void PrintAllPaths() {
      // 找所有非终点作为起点
      std::unordered_set<uint64_t> to_nodes;
      for (const auto &[from, tos] : topo_map_) {
        for (auto to : tos)
          to_nodes.insert(to);
      }

      for (auto node : topo_add_nodes_) {
        if (to_nodes.count(node) == 0) {  // 非被连接节点 = 起点
          std::vector<uint64_t> path;
          DFS(node, path);
        }
      }
    }

private:
    void IsTraverseCrossWalk(std::unordered_map<uint64_t, std::vector<Eigen::Vector2f>> &lane_geos,
                             std::map<uint64_t, std::vector<Eigen::Vector2f>> &cross_walk_geos,
                             const std::vector<BevLaneInfo *> &far_break_bev_lane, const std::vector<BevLaneInfo *> &near_break_bev_lane,
                             std::set<uint64_t> &traver_id);

    void SortBevLaneInfo(std::vector<cem::message::sensor::BevLaneInfo *> &bev_lane_infos_in, std::vector<LineSort> &line_sorts);

    void GetEgoLaneId(const std::vector<cem::message::sensor::BevLaneInfo> &vct_bev_lane_info);

    void CutLaneTail(BevLaneInfo *lane, const float &overlap);

    void AddEdge(uint64_t from, uint64_t to) {
      auto &edges = topo_map_[from];
      if (std::find(edges.begin(), edges.end(), to) == edges.end()) {
        edges.push_back(to);
      }
      topo_add_nodes_.insert(from);
      topo_add_nodes_.insert(to);
    }
    void RemoveEdge(uint64_t from, uint64_t to) {
      auto it = topo_map_.find(from);
      if (it != topo_map_.end()) {
        auto &edges = it->second;
        auto  to_it = std::find(edges.begin(), edges.end(), to);
        if (to_it != edges.end()) {
          edges.erase(to_it);
        }
      }
      // 可选：是否要移除 topo_add_nodes_ 中的节点？取决于你的业务逻辑。
      // topo_add_nodes_.insert(from); // 如果删除边也需要记录节点，可保留
      // topo_add_nodes_.insert(to);
    }
    void DFS(uint64_t node, std::vector<uint64_t> &path) {
      path.push_back(node);

      if (topo_map_.count(node) == 0 || topo_map_[node].empty()) {
        PrintPath(path);  // 到终点了
      } else {
        for (auto next : topo_map_[node]) {
          DFS(next, path);
        }
      }

      path.pop_back();  // 回溯
    }

    void PrintPath(const std::vector<uint64_t> &path) {
      std::stringstream ss;
      for (size_t i = 0; i < path.size(); ++i) {
        ss << path[i];
        if (i + 1 < path.size())
          ss << " -> ";
      }
      AINFO << ss.str();
    }

private:
    std::vector<Eigen::Vector2f> LinePointsVector2f(const std::vector<Point2DF> &lm_points);

    std::unordered_map<uint32_t, message::sensor::BevLaneInfo *> bev_lane_ids_set_;
    std::map<uint32_t, std::vector<uint32_t>> split_topo_list_;
    std::map<uint32_t, std::vector<uint32_t>> merge_topo_list_;

    std::unordered_map<uint64_t, std::vector<uint64_t>> topo_map_;
    std::unordered_set<uint64_t> topo_add_nodes_;

    uint32_t ego_lane_id_        = 0;
    uint32_t last_ego_lane_id_    = 0;
    std::unordered_set<uint32_t> right_next_lane_ids_;
    bool     cross_road_status_  = false;
    double   cross_road_distance_ = -1.0;
    bool cross_turn_right_ = false;
    double   adc_to_junction_ = -1.0;    

    std::vector<traverseCrossWalkLane> traverse_crosswalk_lane_;


    // template <typename TopoListType, typename LaneSetType, typename ConnectType, typename TopoExtendType>
    // void CleanTopoList(TopoListType &topo_list, LaneSetType &lane_set, ConnectType normal_connect_type, TopoExtendType no_topo_extend_type);
};

} // namespace fusion
} // namespace cem
#endif // PROCESSLANESTOPO_H
