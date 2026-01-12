#ifndef EXTEND_LANE_H_
#define EXTEND_LANE_H_
#include "common/CommonDataType.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unordered_map>
#include <common/utils/lane_geometry.h>
#include "lib/common/utility.h"
#include "lib/perception_and_ld_map_fusion/fusion_manager.h"
#include "lib/localmap_construction/crosswalk_mapping.h"
#include "lib/localmap_construction/crossroad_construction/cross_common.h"

namespace cem{
namespace fusion{
namespace extendLane{

typedef cem::message::common::Point2DF Point2DF;

class ExtendLane
{
public:
	ExtendLane();
	~ExtendLane();
	void Process(DetectBevMap &bev_map);
private:
	void IsVirtualLine(const BevMapInfo &bev_map);
    void JudgeIsEmergency(BevMapInfo &bev_map);
    void ExtendLaneInfo(BevMapInfo &bev_map);
    void ExtendLaneMarker(BevMapInfo &bev_map);
    void CurveFitting(std::vector<BevLaneMarker> &lane_markers_infos,
			const BevMapInfo &bev_map);
	void CurveFittingLaneInfo(BevMapInfo &bev_map);
    void SmoothLineInfo(BevMapInfo &bev_map);
    void SmoothLineMarkers(BevMapInfo &bev_map);
    void CutLaneInfoOutLaneMarker(BevMapInfo &bev_map);
    void CutLaneInfoOutOtherLane(BevMapInfo &bev_map);
	void CutLaneInfoOutCrossRoad(BevMapInfo &bevMap);
	//void CutLaneInfoOutCrossRoad(BevMapInfo &bevMap);
    double CalculateLineToLaneDist(std::vector<Eigen::Vector3d> &sourcePt, std::vector<Eigen::Vector3d> &targetPoints,bool debug);
    void ExtendLaneInfoToCross(BevMapInfo &bevMap);
    void ExtendLaneMarkerToCross(BevMapInfo &bevMap);
    void FindCrossMinValue(BevMapInfo &BevMap);
    void BoundLaneMarkerWithLane(BevMapInfo &bev_map);
    void FitAndExtendLanePoints(const std::vector<Eigen::Vector3d> &pointsVec, double maxX,
                                             std::vector<Point2DF>   &target_points,
                                             std::unordered_map<uint64_t, Eigen::VectorXd> &coeff_map, uint64_t id,bool extend_to_cross);
    void DeleteDuplicateLaneInfo(BevMapInfo &bev_map);
    void ExtendLaneMarkerwithLane(BevMapInfo &bev_map);

    Eigen::Isometry3d T_local_ego_;
    Eigen::Isometry3d T_ego_local_;
	std::set<int> m_virtualLineIdxs;
    std::unordered_map<uint64_t, Eigen::VectorXd> m_laneMarkerCoeff;
    std::unordered_map<uint64_t, Eigen::VectorXd> m_laneInfoCoeff;
    std::set<uint64_t> m_emergencyLaneIds{};
    std::set<uint32_t> m_emergencyLaneMarkerIds{};
	double m_corssMinValue {std::numeric_limits<double>::max()};
    std::shared_ptr<cem::fusion::CrossWalkTracker> crosswalk_tracker_ptr_;
};
}
}
}
#endif
