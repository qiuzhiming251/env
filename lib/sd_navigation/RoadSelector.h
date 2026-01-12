#ifndef ROADSELECT_H
#define ROADSELECT_H

#include <base/params_manager/params_defination/internal_params.h>
#include <base/params_manager/params_manager.h>

#include <set>
#include <vector>

#include "fmt/format.h"
#include "lib/common/log_custom.h"
#include "lib/message/env_model/routing_map/routing_map.h"

namespace cem {
namespace fusion {
namespace navigation {
class RoadSelector {

 public:
  RoadSelector();

  std::vector<uint64_t> SelectOptRoad(const std::vector<JunctionInfoCity> &junctions_info_city,
                                      const std::set<uint64_t> &bev_ego_lane_related, const std::vector<uint64_t> &history_guide_laneids,
                                      const std::vector<std::vector<uint64_t>> &bev_sections_in, const BevRouteInfo &complete_section_info,
                                      fmt::memory_buffer &info_buffer, bool &isSplitRoadSelectWorked);

  std::vector<uint64_t> HoldOptRoad(const std::vector<std::vector<uint64_t>> &bev_sections_in, const BevRouteInfo &complete_section_info,
                                    const std::vector<uint64_t> &road_selected);

 private:
  size_t SelectSectionWithMostLanes(const std::vector<std::vector<uint64_t>> &bev_sections_in);
  int    SelectMainRoadSectionNoNavi(const std::vector<std::vector<uint64_t>> &bev_sections_in,
                                     const std::vector<SeparatorInfo>         &section_separators);
  /**
     *  //RoadSplit场景中 判断是否是主路到辅路场景
     *  @param junction 城市 junction 信息，包含 junction 类型、后续道路类别等
     *  @return SplitPassType 路口通过类型
     */
  SplitPassType RoadSplitTypeJudge(const JunctionInfoCity &junction);
  /**
     * //分叉口直行 选路
     * @param bev_sections_in
     * @return size_t 选路结果
     */
  std::vector<std::vector<uint64_t>> SelectSectionInStraight(const std::vector<std::vector<uint64_t>> &bev_sections_in,
                                                             const JunctionInfoCity                   &junction);

  /**
     * //辅路车道数
     * @param
     * @return 辅路车道数
     */
  int GetLanenumOfRamp(const JunctionInfoCity &junction, SplitPassType &PassType);

  std::vector<uint64_t>        SelectRoadWithSdPts(const std::vector<std::vector<uint64_t>> &bev_sections_in,
                                                   const std::vector<SeparatorInfo>         &section_separators);
  std::vector<Eigen::Vector2f> SparsePoints(const std::vector<Eigen::Vector2f> &points, size_t n);
  float                        CalcPointSetSimilarity(std::vector<Eigen::Vector2f> &bev_points, std::vector<Eigen::Vector2f> &sd_points);

  // 计算综合section代价的函数，包含形点权重、方向权重等
  double CalculateCombinedSectionCost(int section_index, const std::vector<std::vector<uint64_t>> &bev_sections_in,
                                      const std::vector<int> &ego_section_indexs, const std::vector<uint64_t> &road_selected_sd,
                                      const DirectionSplitMerge &split_merge_dir);

  std::set<uint64_t>              bev_ego_lane_related_       = {};
  std::vector<uint64_t>           history_guide_laneids_      = {};
  bool                            use_select_road_with_sdpts_ = false;
  std::vector<HistSelectRoadInfo> hist_road_infos_            = {};
};
}  //namespace navigation
}  // namespace fusion
}  // namespace cem

#endif  //ROADSELECT_H
