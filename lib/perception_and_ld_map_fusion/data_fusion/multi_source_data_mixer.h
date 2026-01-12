#ifndef MULTISOURCEDATAMIXER_
#define MULTISOURCEDATAMIXER_
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "common/utility.h"
#include "lib/perception_and_ld_map_fusion/common/pose_filter.h"
#include "lib/perception_and_ld_map_fusion/data_fusion/bev_filleting_machine.h"
#include "lib/perception_and_ld_map_fusion/data_fusion/match_maker.h"
#include "lib/perception_and_ld_map_fusion/data_preprocessor/bev_map_preprocessor.h"
#include "lib/perception_and_ld_map_fusion/data_preprocessor/ld_map_preprocessor.h"
namespace cem {
namespace fusion {

class MultiSourceDataMixer {
  struct FittedLane {
    uint64_t id;
    double c0;
    double c1;
    double c2;
    double c3;
    double start_x;
    double end_x;
  };

 public:
  MultiSourceDataMixer();
  ~MultiSourceDataMixer();
  void Init(BevMapProcessor* bev_map_processor,
            LdMapProcessor* ld_map_processor, BevFilletingMachine* slicer);
  void LoadBevMap(BevMapProcessor* bev_map_processor);
  void LoadLdMap(LdMapProcessor* ld_map_processor);
  void LoadBevSlicer(BevFilletingMachine* slicer);

  void Process(const std::unordered_map<
                   uint64_t, std::vector<std::unique_ptr<
                                 cem::fusion::MatchMaker::MatchingDetail>>>&
                   match_details,
               const cem::fusion::LocalizationConstPtr pose);

  // const cem::message::env_model::RoutingMap& GetFusedMap() const {
  //   return fused_map_;
  // };

 private:
  BevMapProcessor* bev_map_preprocessor_ = nullptr;
  LdMapProcessor* ld_map_preprocessor_ = nullptr;
  cem::fusion::RoutingMapConstPtr corrected_map_ = nullptr;
  BevFilletingMachine* bev_slicer_ = nullptr;
  std::unique_ptr<PoseFilter> pose_filter_ = nullptr;
  Eigen::MatrixXd process_noise_;
  Eigen::MatrixXd measurement_noise_by_matching_info_;
  Eigen::MatrixXd measurement_noise_by_last_state_;
  double timestamp_ = 0.0;
  // cem::message::env_model::RoutingMap fused_map_;
  int noise_factor_of_matching_info_;
  std::vector<double> predicted_pose_offset_;
  bool is_match_info_believable_ = false;
  bool is_small_size_of_sampling_point = false;

 private:
  std::vector<double> EstimatePoseOffsetByMatchInfo(
      const std::unordered_map<uint64_t,
                               std::vector<std::unique_ptr<
                                   cem::fusion::MatchMaker::MatchingDetail>>>&
          match_details);
  std::vector<double> EstimatePoseOffsetByMatchInfoRANSAC(
      const std::unordered_map<uint64_t,
                               std::vector<std::unique_ptr<
                                   cem::fusion::MatchMaker::MatchingDetail>>>&
          match_details,
      int max_iteration_num = 100, int minimum_sample_size = 4,
      double inliner_distance_threshold = 0.2,
      double inliner_min_sample_prob_threshold = 0.95);
  std::vector<double> EstimatePoseOffsetByLastState(const Eigen::Isometry3d& T);
  std::vector<double> EstimatePoseOffsetByLastStateRANSAC(
      const Eigen::Isometry3d& T, int max_iteration_num = 100,
      int minimum_sample_size = 4, double inliner_distance_threshold = 0.2,
      double inliner_min_sample_prob_threshold = 0.95);
  bool IsPoseEstimateBelievable(
      const std::vector<double>& pose_offset_by_last_state,
      const std::vector<double>& pose_offset_by_vision);
  Eigen::Matrix4d ConstructLdMapCorrectT(const std::vector<double>& params);
  void DataFusion();
  void FitCubicCurve(
      const std::vector<const std::vector<cem::message::env_model::Point>*>&
          points,
      const int point_size, Eigen::Vector4d& coeffs, double& start_pt,
      double& end_pt);
  std::vector<int> GetRandomSamplesIndex(int n, bool fix_seed = false);
  void ExtractHdMapInfoIntoPerceptionMap(
      const std::unordered_map<uint64_t,
                               std::vector<std::unique_ptr<
                                   cem::fusion::MatchMaker::MatchingDetail>>>&
          match_details);
  void AlignHdMapToPerceptionMap(
      const std::unordered_map<uint64_t,
                               std::vector<std::unique_ptr<
                                   cem::fusion::MatchMaker::MatchingDetail>>>&
          match_details,
      const cem::fusion::LocalizationConstPtr pose);
};

}  // namespace fusion
}  // namespace cem

#endif  // MULTISOURCEDATAMIXER_