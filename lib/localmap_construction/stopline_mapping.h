#ifndef STOPLINE_MAPPING_H_
#define STOPLINE_MAPPING_H_
#include "base/sensor_data_manager.h"
#include "common/utility.h"
#include "lib/common/log_custom.h"
#include "lib/message/env_model/stop_line/stop_line.h"
#include "lib/message/internal_message.h"
namespace cem {
namespace fusion {

class StoplineMapping {
    public:
        StoplineMapping();
        ~StoplineMapping();
        void Process();

        const std::vector<cem::message::env_model::StopLine>& GetStopLines()
            const {
          return stop_line_objs_;
        }

        const std::unordered_map<uint64_t, uint64_t>& GetLaneToStopline()
            const {
          return lane_to_stopline_;
        }

        const double timestamp() const { return timestamp_;};

        enum StopLineType
        {
            ST_TYPE_UNKNOWN = 0,        // 未知
            ST_TYPE_VERTICAL = 1,       // 与车道垂直
            ST_TYPE_SLOP = 2,           // 与车道倾斜
            ST_TYPE_STEP = 3,           // 阶梯型
            ST_TYPE_DOUBLE = 4,         // 双线
            ST_TYPE_WAIT = 5            // 待转区
        };

        static constexpr float MAX_LANE_WIDTH = 5.0f;
        static constexpr float MIN_LANE_WIDTH = 2.3f;
        static constexpr float DEFAULT_LANE_WIDTH = 3.5f;


    private:
     void Reset();
     bool CalcEgoToLocal(cem::fusion::BevMapInfoPtr& bev_info);
     bool FeedLatestFrame(const cem::fusion::BevMapInfoPtr& bev_info);
     void LaneAssociation(cem::fusion::BevMapInfoPtr& bev_info);
     void CalcDisToStopline(cem::fusion::BevMapInfoPtr& bev_info);
     void SetStopLinesObjs(cem::fusion::BevMapInfoPtr& bev_info);
     void Tracking(const cem::fusion::BevMapInfoPtr& bev_info);

     bool AssociateBevAndMap(const cem::message::env_model::StopLine& bev_stopline,
                             const cem::message::env_model::StopLine& noa_stopline);

     void BevProcessor();

     void FusionNOAMap();

     double timestamp_;  // sec
     uint64_t prev_bev_timestamp_ = 0;
     int lost_count_;
     Eigen::Isometry3d T_local_ego_;
     Eigen::Isometry3d T_ego_local_;
     bool binding_success_;
     bool is_used_stopline_ = false;
     double distance_to_stopline_{std::numeric_limits<double>::lowest()};
     std::vector<cem::message::env_model::StopLine> stop_line_objs_;
     std::unordered_map<uint64_t, uint64_t> lane_to_stopline_;
     std::unordered_map<uint64_t, double> stopline_to_distance_;
};

}  // namespace fusion
}  // namespace cem

#endif  // STOPLINE_MAPPING_H_
