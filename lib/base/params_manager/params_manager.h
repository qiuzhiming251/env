/*
 * @Date: 2024-03-04 11:03:32
 * @Description: 
 */
#ifndef PARAMS_MANAGER_H_
#define PARAMS_MANAGER_H_

#include "params_defination/params_headers.h"
#include "common/design_pattern/singleton/singleton.h"
#include "cyber/common/macros.h"
#include "cyber/common/log.h"
// #include "third_party/nlohmann_json/json.hpp"
#include "nlohmann/json.hpp"
#include <fstream>
#include <ostream>
#include <string>
#include <iostream>

namespace cem {
namespace fusion {


class ParamsManager
{
private:
    VehicleParams vehicle_params_;
    NrpParams nrp_params_;
    PreProcessorParams pre_processor_params_;
    MasterParams master_params_;
    BaseParams base_params_;
    CommunicationParams communication_params_;
    DataAssociationParams data_association_params_;
    DataFusionParams data_fusion_params_;
    FusionSystemParams fusion_system_params_;
    LaneBuilderParams lane_builder_params_;
    PostProcessorParams post_processor_params_;
    DebugParams debug_params_;
    FLEParams fle_params_;
    DiagnosticParams diagnostic_params_;
    MapConvertParams map_convert_params_;

   private:
    VersionInfoParams version_info_params_;
    InternalParams internal_params_;

private:
    nlohmann::json eval_params_;

public:
    void ReadJsonParams(const std::string &main_params_json_file_path,
                        const std::string &eval_params_json_file_path);

    void ReadVersionInfoParams(const std::string &version_file_path);

    inline const VehicleParams &GetVehicleParams() { return vehicle_params_; }

    inline const NrpParams &GetNrpParams() { return nrp_params_; }

    inline const PreProcessorParams &GetPreProcessorParams()
    {
        return pre_processor_params_;
    }

    inline const MasterParams &GetMasterParams() { return master_params_; }

    inline const BaseParams &GetBaseParams() { return base_params_; }

    inline const CommunicationParams &GetCommunicationParams()
    {
        return communication_params_;
    }

    inline const DataAssociationParams &GetDataAssociationParams()
    {
        return data_association_params_;
    }

    inline const DataFusionParams &GetDataFusionParams()
    {
        return data_fusion_params_;
    }

    inline const FusionSystemParams &GetFusionSystemParams()
    {
        return fusion_system_params_;
    }

    inline const LaneBuilderParams &GetLaneBuilderParams()
    {
        return lane_builder_params_;
    }

    inline const PostProcessorParams &GetPostProcessorParams()
    {
        return post_processor_params_;
    }

    inline const DebugParams &GetDebugParams() { return debug_params_; }

    inline const FLEParams &GetFLEParams() { return fle_params_; }

    inline const DiagnosticParams &GetDiagnosticParams()
    {
        return diagnostic_params_;
    }

    MapConvertParams GetMapConvertParams() const { return map_convert_params_; }

   public:
    inline VersionInfoParams &GetVersionInfoParams()
    {
        return version_info_params_;
    }

    inline InternalParams &GetInternalParams() { return internal_params_; }

public:
     inline nlohmann::json &GetEvalParams() { return eval_params_; }

    DECLARE_SINGLETON(ParamsManager)
};

#define VEHICLE_PARAMS ParamsManager::Instance()->GetVehicleParams()
#define NRP_PARAMS ParamsManager::Instance()->GetNrpParams()
#define PRE_PROCESSOR_PARAMS ParamsManager::Instance()->GetPreProcessorParams()
#define MASTER_PARAMS ParamsManager::Instance()->GetMasterParams()
#define BASE_PARAMS ParamsManager::Instance()->GetBaseParams()
#define COMMUNICATION_PARAMS ParamsManager::Instance()->GetCommunicationParams()
#define DATA_ASSOCIATION_PARAMS \
    ParamsManager::Instance()->GetDataAssociationParams()
#define DATA_FUSION_PARAMS_ ParamsManager::Instance()->GetDataFusionParams()
#define FUSION_SYSTEM_PARAMS ParamsManager::Instance()->GetFusionSystemParams()
#define LANE_BUILDER_PARAMS ParamsManager::Instance()->GetLaneBuilderParams()
#define POST_PROCESSOR_PARAMS \
    ParamsManager::Instance()->GetPostProcessorParams()
#define DEBUG_PARAMS ParamsManager::Instance()->GetDebugParams()
#define FLE_PARAMS ParamsManager::Instance()->GetFLEParams()
#define VERSION_INFO_PARAMS ParamsManager::Instance()->GetVersionInfoParams()
#define INTERNAL_PARAMS ParamsManager::Instance()->GetInternalParams()
#define EVAL_PARAMS ParamsManager::Instance()->GetEvalParams()
#define DIAGNOSTIC_PARAMS ParamsManager::Instance()->GetDiagnosticParams()


#define FUSION_SENSORS_PARAMS MASTER_PARAMS.fusion_sensor_params_
#define TRACK_PARAMS BASE_PARAMS.track_params_
#define NN_ASSOCIATION_PARAMS DATA_ASSOCIATION_PARAMS.nn_association_params_
#define MOTION_FUSION_PARAMS DATA_FUSION_PARAMS_.motion_fusion_params_
#define TRACK_ATTRIBUTE_FUSION_PARAMS \
    DATA_FUSION_PARAMS_.track_attribute_fusion_params_
#define TRACKER_PARAMS DATA_FUSION_PARAMS_.tracker_params_
#define SEQUENTIAL_FUSION_PARAMS FUSION_SYSTEM_PARAMS.sequential_fusion_params_
#define CENTRAL_FUSION_PARAMS FUSION_SYSTEM_PARAMS.central_fusion_params_
#define SCENE_PARAMS MASTER_PARAMS.scene_params_


#define KALMAN_MOTION_FUSION_PARAMS \
    MOTION_FUSION_PARAMS.kalman_motion_fusion_params_
#define COLOR_FUSION_PARAMS TRACK_ATTRIBUTE_FUSION_PARAMS.color_fusion_params_
#define EXISTENCE_FUSION_PARAMS \
    TRACK_ATTRIBUTE_FUSION_PARAMS.existence_fusion_params_
#define TYPE_FUSION_PARAMS TRACK_ATTRIBUTE_FUSION_PARAMS.type_fusion_params_
#define PBF_TRACKER_PARAMS TRACKER_PARAMS.pbf_tracker_params_

} // namespace fusion
} // namespace cem

#endif
