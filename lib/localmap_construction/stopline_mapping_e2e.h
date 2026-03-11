#ifndef STOPLINE_MAPPING_E2E_H_
#define STOPLINE_MAPPING_E2E_H_
#include "base/sensor_data_manager.h"
#include "common/utility.h"
#include "fmt/core.h"
#include "fmt/ranges.h"
#include "lib/common/log_custom.h"
#include "lib/message/env_model/stop_line/stop_line.h"
#include "lib/message/internal_message.h"
namespace cem {
namespace fusion {
namespace e2e {

class StoplineMapping {
    public:
        StoplineMapping();
        ~StoplineMapping();
        void Process(const BevMapInfoPtr& bev_map_info);

        const std::vector<cem::message::env_model::StopLine>& GetStopLines()
            const {
          return stop_line_objs_;
        }

        const std::unordered_map<uint64_t, uint64_t>& GetLaneToStopline()
            const {
          return lane_to_stopline_;
        }
        const std::string& GetStopLineDebug() const {
          return stopline_debug_info_;
        }

        enum StopLineType
        {
            ST_TYPE_UNKNOWN = 0,        // 未知
            ST_TYPE_VERTICAL = 1,       // 与车道垂直
            ST_TYPE_SLOP = 2,           // 与车道倾斜
            ST_TYPE_STEP = 3,           // 阶梯型
            ST_TYPE_DOUBLE = 4,         // 双线
            ST_TYPE_WAIT = 5            // 待转区
        };

    private:
     void Reset();
     bool CalcEgoToLocal(const BevMapInfoPtr& bev_info);
     void LaneAssociation(const BevMapInfoPtr& bev_info);
     void CalcDisToStopline(const BevMapInfoPtr& bev_info);
     void SetStopLinesObjs(const BevMapInfoPtr& bev_info);
     void Tracking(
         std::vector<cem::message::env_model::StopLine>& stop_line_objs);
     double FindNearestPoint(const std::vector<Point2DF>& lane_points,
                             const Point2DF& target_point);
     float Nearest_lane_point_to_stopline(
         const std::vector<Point2DF>& lane_points,
         const std::vector<Point2DF>& stopline_points);
     float Point_to_segment(const Point2DF& p, const Point2DF& a,
                            const Point2DF& b);

     std::vector<BevLaneMarker> SearchNearbyStopLines(
         const BevLaneMarker& known_stopline,
         const std::vector<BevLaneMarker>& stoplines);
     Eigen::Vector2d GetStoplineMidPoint(const BevLaneMarker& stopline);
     double CalcDistance(const Point2DF& p1, const Point2DF& p2);
     void GetDebugInfo(
         const std::vector<cem::message::env_model::StopLine>& stop_line_objs);
     uint64_t prev_bev_timestamp_ = 0;
     int missing_frames_ = 0;
     int maintain_frames_ = std::numeric_limits<int>::max();  // raw 10fps
     Eigen::Isometry3d T_local_ego_;
     Eigen::Isometry3d T_ego_local_;
     bool binding_success_;
     bool is_used_stopline_ = false;
     double distance_to_stopline_{std::numeric_limits<double>::lowest()};
     std::vector<cem::message::env_model::StopLine> stop_line_objs_;
     std::vector<cem::message::env_model::StopLine> last_valid_stop_lines_;
     std::unordered_map<uint64_t, uint64_t> lane_to_stopline_;
     std::unordered_map<uint64_t, double> stopline_to_distance_;
     std::unordered_map<uint64_t, BevLaneMarker> stopline_map_;
     uint64_t prev_best_stop_line_id_ = 0;
     std::string stopline_debug_info_;
     std::vector<uint64_t> filter_stop_lines_;
};
}
}  // namespace fusion
}  // namespace cem

#endif  // STOPLINE_MAPPING_E2E_H_
