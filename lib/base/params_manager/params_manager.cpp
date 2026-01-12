#include "params_manager.h"

namespace cem {
namespace fusion {

ParamsManager::ParamsManager() {}

void ParamsManager::ReadJsonParams(
    const std::string &main_params_json_file_path,
    const std::string &eval_params_json_file_path)
{
    try
    {
        std::ifstream main_params_fstream_input(main_params_json_file_path);
        nlohmann::json nlohmann_json;
        main_params_fstream_input >> nlohmann_json;

        std::ifstream eval_params_fstream_input(eval_params_json_file_path);
        eval_params_fstream_input >> eval_params_;

// for vehicle params
#define vehicle_params Instance()->vehicle_params_
        vehicle_params.vehicle_width =
            nlohmann_json["VehicleParams"]["vehicle_width"];
        vehicle_params.vehicle_length =
            nlohmann_json["VehicleParams"]["vehicle_length"];
        vehicle_params.cam_to_rear_axle_center_lon_dist =
            nlohmann_json["VehicleParams"]["cam_to_rear_axle_center_lon_dist"];
        vehicle_params.cam_to_rear_axle_center_lat_dist =
            nlohmann_json["VehicleParams"]["cam_to_rear_axle_center_lat_dist"];

// for nrp params
#define nrp_params Instance()->nrp_params_

        nrp_params.nrp_mode = nlohmann_json["NrpParams"]["nrp_mode"];
        nrp_params.switch_from_vision_to_hdmap_dist_thresh =
            nlohmann_json["NrpParams"]
                         ["switch_from_vision_to_hdmap_dist_thresh"];
        nrp_params.switch_from_hdmap_to_vision_dist_thresh =
            nlohmann_json["NrpParams"]
                         ["switch_from_hdmap_to_vision_dist_thresh"];
        nrp_params.enter_split_merge_dist_thresh =
            nlohmann_json["NrpParams"]["enter_split_merge_dist_thresh"];
        nrp_params.exit_split_merge_dist_thresh =
            nlohmann_json["NrpParams"]["exit_split_merge_dist_thresh"];
        nrp_params.ramp_end_area_dist_thresh =
            nlohmann_json["NrpParams"]["ramp_end_area_dist_thresh"];
        nrp_params.enter_urban_dist_thresh =
            nlohmann_json["NrpParams"]["enter_urban_dist_thresh"];
        nrp_params.exit_urban_dist_thresh =
            nlohmann_json["NrpParams"]["exit_urban_dist_thresh"];
        nrp_params.pre_center_line_opt_ego_lanemarker =
            nlohmann_json["NrpParams"]["pre_center_line_opt_ego_lanemarker"];
        nrp_params.re_enable = nlohmann_json["NrpParams"]["re_enable"];
        nrp_params.re_points_viz = nlohmann_json["NrpParams"]["re_points_viz"];
        nrp_params.re_diff_dis = nlohmann_json["NrpParams"]["re_diff_dis"];
        nrp_params.enable_hdm_loc_check =
            nlohmann_json["NrpParams"]["enable_hdm_loc_check"];
        nrp_params.enable_nrp_switch =
            nlohmann_json["NrpParams"]["enable_nrp_switch"];
        nrp_params.ld_map_activation_dist_thresh =
            nlohmann_json["NrpParams"]["ld_map_activation_dist_thresh"];
        nrp_params.ld_map_deactivation_dist_thresh =
            nlohmann_json["NrpParams"]["ld_map_deactivation_dist_thresh"];

// for pre-processor params
#define pre_processor_params Instance()->pre_processor_params_

        pre_processor_params.enable_fvcm_comp =
            nlohmann_json["PreProcessorParams"]["enable_fvcm_comp"];
        pre_processor_params.enable_fvcm_re_protection =
            nlohmann_json["PreProcessorParams"]["enable_fvcm_re_protection"];
        pre_processor_params.enable_fvcm_re_protection_outside_ramp =
            nlohmann_json["PreProcessorParams"]
                         ["enable_fvcm_re_protection_outside_ramp"];
        pre_processor_params.enable_vision_pre_fusion_re_protection =
            nlohmann_json["PreProcessorParams"]
                         ["enable_vision_pre_fusion_re_protection"];
        pre_processor_params.enable_vision_pre_fusion_in_rainy =
            nlohmann_json["PreProcessorParams"]
                         ["enable_vision_pre_fusion_in_rainy"];
        pre_processor_params.enable_vision_pre_fusion_within_hdmap =
            nlohmann_json["PreProcessorParams"]
                         ["enable_vision_pre_fusion_within_hdmap"];
        pre_processor_params.enable_hdmap_comp =
            nlohmann_json["PreProcessorParams"]["enable_hdmap_comp"];
        pre_processor_params.hdm_virtual_point_weight =
            nlohmann_json["PreProcessorParams"]["hdm_virtual_point_weight"];
        pre_processor_params.enable_extend_virtual_seg =
            nlohmann_json["PreProcessorParams"]["enable_extend_virtual_seg"];
        pre_processor_params.enable_gtr_fix =
            nlohmann_json["PreProcessorParams"]["enable_gtr_fix"];
        pre_processor_params.collision_prediction_time =
            nlohmann_json["PreProcessorParams"]["collision_prediction_time"];
        pre_processor_params.enable_gtr_choose_opposing_lane =
            nlohmann_json["PreProcessorParams"]
                         ["enable_gtr_choose_opposing_lane"];
        pre_processor_params.enable_collision_detection =
            nlohmann_json["PreProcessorParams"]["enable_collision_detection"];
        pre_processor_params.enable_bev_evaluate =
            nlohmann_json["PreProcessorParams"]["enable_bev_evaluate"];
        pre_processor_params.enable_limit_bev_evaluate_speed =
            nlohmann_json["PreProcessorParams"]
                         ["enable_limit_bev_evaluate_speed"];
        pre_processor_params.enable_collision_detection_rviz =
            nlohmann_json["PreProcessorParams"]
                         ["enable_collision_detection_rviz"];
        pre_processor_params.enable_opposite_lanemarker_filter =
            nlohmann_json["PreProcessorParams"]
                         ["enable_opposite_lanemarker_filter"];
        pre_processor_params.enable_re_constraint_correction =
            nlohmann_json["PreProcessorParams"]
                         ["enable_re_constraint_correction"];
        pre_processor_params.enable_reduce_hdmap_weight =
            nlohmann_json["PreProcessorParams"]["enable_reduce_hdmap_weight"];
        pre_processor_params.enable_nrp_downgrade_by_evaluate =
            nlohmann_json["PreProcessorParams"]
                         ["enable_nrp_downgrade_by_evaluate"];
        pre_processor_params.enable_hdmap_high_priority_seg_enable =
            nlohmann_json["PreProcessorParams"]
                         ["enable_hdmap_high_priority_seg_enable"];
        pre_processor_params.enable_hdmap_curvature_diff_cost_enable =
            nlohmann_json["PreProcessorParams"]
                         ["enable_hdmap_curvature_diff_cost_enable"];
        pre_processor_params.enable_hdmap_double_re_constraint =
            nlohmann_json["PreProcessorParams"]
                         ["enable_hdmap_double_re_constraint"];
        pre_processor_params.enable_hdm_lane_discrete_point_smooth =
            nlohmann_json["PreProcessorParams"]
                         ["enable_hdm_lane_discrete_point_smooth"];
        pre_processor_params.enable_hdm_lane_vitrual_erase_and_generator =
            nlohmann_json["PreProcessorParams"]
                         ["enable_hdm_lane_vitrual_erase_and_generator"];
        pre_processor_params.enable_hdm_lane_fit =
            nlohmann_json["PreProcessorParams"]["enable_hdm_lane_fit"];
        pre_processor_params.enable_replace_bad_measurement_with_good =
            nlohmann_json["PreProcessorParams"]
                         ["enable_replace_bad_measurement_with_good"];
        pre_processor_params.enable_remove_bev_faraway_points =
            nlohmann_json["PreProcessorParams"]
                         ["enable_remove_bev_faraway_points"];
        pre_processor_params.enable_join_bev_merge_lanemarkers =
            nlohmann_json["PreProcessorParams"]
                         ["enable_join_bev_merge_lanemarkers"];
        pre_processor_params.enable_double_roadedge_constrain =
            nlohmann_json["PreProcessorParams"]
                         ["enable_double_roadedge_constrain"];
        pre_processor_params.enable_extend_lanemarker_length_for_re =
            nlohmann_json["PreProcessorParams"]
                         ["enable_extend_lanemarker_length_for_re"];
        pre_processor_params.enable_downgrade_on_ramp =
            nlohmann_json["PreProcessorParams"]["enable_downgrade_on_ramp"];
        pre_processor_params.enable_highway_measurement_replacement =
            nlohmann_json["PreProcessorParams"]
                         ["enable_highway_measurement_replacement"];
        pre_processor_params.enable_bev_lanemarker_width =
            nlohmann_json["PreProcessorParams"]["enable_bev_lanemarker_width"];
        pre_processor_params.is_ola_lanemarker_width_available =
            nlohmann_json["PreProcessorParams"]
                         ["is_ola_lanemarker_width_available"];
        pre_processor_params.enable_pole_roadedge_constraint_correction =
            nlohmann_json["PreProcessorParams"]
                         ["enable_pole_roadedge_constraint_correction"];
        pre_processor_params.enable_bev_lane_topology =
            nlohmann_json["PreProcessorParams"]["enable_bev_lane_topology"];
        pre_processor_params.enable_bev_lane_topology_rviz =
            nlohmann_json["PreProcessorParams"]
                         ["enable_bev_lane_topology_rviz"];
        pre_processor_params
            .enable_mean_nrp_downgrade_work_without_re_constraint =
            nlohmann_json
                ["PreProcessorParams"]
                ["enable_mean_nrp_downgrade_work_without_re_constraint"];
        pre_processor_params.enable_hdmap_eval_recovery_optimize_on_ramp =
            nlohmann_json["PreProcessorParams"]
                         ["enable_hdmap_eval_recovery_optimize_on_ramp"];
        pre_processor_params.enable_map_checker_downgrade =
            nlohmann_json["PreProcessorParams"]["enable_map_checker_downgrade"];
        pre_processor_params.enable_large_hdmap_radius_re_constraint =
            nlohmann_json["PreProcessorParams"]
                         ["enable_large_hdmap_radius_re_constraint"];
        pre_processor_params.large_hdmap_radius_thresh =
            nlohmann_json["PreProcessorParams"]["large_hdmap_radius_thresh"];

// for master params
#define master_params Instance()->master_params_

        master_params.fusion_method =
            nlohmann_json["MasterParams"]["fusion_method"];
                master_params.lane_fusion_method =
            nlohmann_json["MasterParams"]["lane_fusion_method"];
        master_params.segments_num =
            nlohmann_json["MasterParams"]["segments_num"];
        master_params.coef_filter_segs_num =
            nlohmann_json["MasterParams"]["coef_filter_segs_num"];
        master_params.used_localization =
            nlohmann_json["MasterParams"]["used_localization"];

#define fusion_sensor_params Instance()->master_params_.fusion_sensor_params_
        fusion_sensor_params.enable_fvcm =
            nlohmann_json["MasterParams"]["FusionSensorsParams"]["enable_fvcm"];
        fusion_sensor_params.enable_around_view =
            nlohmann_json["MasterParams"]["FusionSensorsParams"]
                         ["enable_around_view"];
        fusion_sensor_params.enable_hdmap =
            nlohmann_json["MasterParams"]["FusionSensorsParams"]
                         ["enable_hdmap"];
        fusion_sensor_params.enable_traffic_flow =
            nlohmann_json["MasterParams"]["FusionSensorsParams"]
                         ["enable_traffic_flow"];
        fusion_sensor_params.enable_road_edge =
            nlohmann_json["MasterParams"]["FusionSensorsParams"]
                         ["enable_road_edge"];
        fusion_sensor_params.enable_further_lane_extension =
            nlohmann_json["MasterParams"]["FusionSensorsParams"]
                         ["enable_further_lane_extension"];
        fusion_sensor_params.enable_measurement_evaluate =
            nlohmann_json["MasterParams"]["FusionSensorsParams"]
                         ["enable_measurement_evaluate"];
        fusion_sensor_params.enable_bev =
            nlohmann_json["MasterParams"]["FusionSensorsParams"]["enable_bev"];
        fusion_sensor_params.enable_bev_motion_nonhost =
            nlohmann_json["MasterParams"]["FusionSensorsParams"]
                         ["enable_bev_motion_nonhost"];
        fusion_sensor_params.enable_vision_pre_fusion =
            nlohmann_json["MasterParams"]["FusionSensorsParams"]
                         ["enable_vision_pre_fusion"];
        

#define scene_param Instance()->master_params_.scene_params_
        scene_param.intersection_enable_stopline =
            nlohmann_json["MasterParams"]["SceneParams"]
                         ["intersection_enable_stopline"];
        scene_param.enter_intersection_dx_end_threshold =
            nlohmann_json["MasterParams"]["SceneParams"]
                         ["enter_intersection_dx_end_threshold"];
        scene_param.stopline_compensate_dis_max =
            nlohmann_json["MasterParams"]["SceneParams"]
                         ["stopline_compensate_dis_max"];
        scene_param.thorough_exit_crossroad_dx_start_threshold =
            nlohmann_json["MasterParams"]["SceneParams"]
                         ["thorough_exit_crossroad_dx_start_threshold"];
        scene_param.coming_exit_crossroad_dx_start_threshold =
            nlohmann_json["MasterParams"]["SceneParams"]
                         ["coming_exit_crossroad_dx_start_threshold"];
        scene_param.dx_startHysteresisThreshold =
            nlohmann_json["MasterParams"]["SceneParams"]
                         ["dx_startHysteresisThreshold"];
        scene_param.dx_endHysteresisThreshold =
            nlohmann_json["MasterParams"]["SceneParams"]
                         ["dx_endHysteresisThreshold"];
// for base params
#define track_params Instance()->base_params_.track_params_

        track_params.low_speed_thresh =
            nlohmann_json["BaseParams"]["TrackParams"]["low_speed_thresh"];
        track_params.track_confirmed_thresh =
            nlohmann_json["BaseParams"]["TrackParams"]
                         ["track_confirmed_thresh"];
        track_params.track_normal_speed_lost_thresh =
            nlohmann_json["BaseParams"]["TrackParams"]
                         ["track_normal_speed_lost_thresh"];
        track_params.track_low_speed_lost_thresh =
            nlohmann_json["BaseParams"]["TrackParams"]
                         ["track_low_speed_lost_thresh"];
        track_params.first_seg_len_thresh =
            nlohmann_json["BaseParams"]["TrackParams"]["first_seg_len_thresh"];

// for communication params
#define communication_params Instance()->communication_params_
        communication_params.traffic_road_hmi_topic =
            nlohmann_json["CommunicationParams"]["traffic_road_hmi_topic"];
        communication_params.rviz_traffic_road_hmi_topic =
            nlohmann_json["CommunicationParams"]["rviz_traffic_road_hmi_topic"];
        communication_params.enable_traffic_road_hmi_output =
            nlohmann_json["CommunicationParams"]
                         ["enable_traffic_road_hmi_output"];
        communication_params.enable_traffic_road_hmi_points_output =
            nlohmann_json["CommunicationParams"]
                         ["enable_traffic_road_hmi_points_output"];
        communication_params.enable_reset_subs_on_parking =
            nlohmann_json["CommunicationParams"]
                         ["enable_reset_subs_on_parking"];

// for data association params
#define nn_association_params \
    Instance()->data_association_params_.nn_association_params_

        nn_association_params.basic_weight =
            nlohmann_json["DataAssociationParams"]["NNAssociationParams"]
                         ["basic_weight"];
        nn_association_params.match_dist_thresh =
            nlohmann_json["DataAssociationParams"]["NNAssociationParams"]
                         ["match_dist_thresh"];
        nn_association_params.dist_direction =
            nlohmann_json["DataAssociationParams"]["NNAssociationParams"]
                         ["dist_direction"];

// for data fusion param
#define kalman_motion_fusion_params                 \
    Instance()                                      \
        ->data_fusion_params_.motion_fusion_params_ \
        .kalman_motion_fusion_params_

        kalman_motion_fusion_params.filter_method =
            nlohmann_json["DataFusionParams"]["MotionFusionParams"]
                         ["KalmanMotionFusionParams"]["filter_method"];
        kalman_motion_fusion_params.enable_weighted_mean_update =
            nlohmann_json["DataFusionParams"]["MotionFusionParams"]
                         ["KalmanMotionFusionParams"]
                         ["enable_weighted_mean_update"];

#define type_fusion_params                                   \
    Instance()                                               \
        ->data_fusion_params_.track_attribute_fusion_params_ \
        .type_fusion_params_

        type_fusion_params.enable_type_fusion =
            nlohmann_json["DataFusionParams"]["TrackAttributeFusionParams"]
                         ["TypeFusionParams"]["enable_type_fusion"];
        type_fusion_params.type_mismatch_thresh =
            nlohmann_json["DataFusionParams"]["TrackAttributeFusionParams"]
                         ["TypeFusionParams"]["type_mismatch_thresh"];
        type_fusion_params.enable_cone_protection =
            nlohmann_json["DataFusionParams"]["TrackAttributeFusionParams"]
                         ["TypeFusionParams"]["enable_cone_protection"];
        type_fusion_params.cone_risk_thresh =
            nlohmann_json["DataFusionParams"]["TrackAttributeFusionParams"]
                         ["TypeFusionParams"]["cone_risk_thresh"];
        type_fusion_params.enable_forward_vision_type =
            nlohmann_json["DataFusionParams"]["TrackAttributeFusionParams"]
                         ["TypeFusionParams"]["enable_forward_vision_type"];

#define pbf_tracker_params \
    Instance()->data_fusion_params_.tracker_params_.pbf_tracker_params_

        pbf_tracker_params.motion_fusion_method =
            nlohmann_json["DataFusionParams"]["TrackerParams"]
                         ["PbfTrackerParams"]["motion_fusion_method"];
        pbf_tracker_params.color_fusion_method =
            nlohmann_json["DataFusionParams"]["TrackerParams"]
                         ["PbfTrackerParams"]["color_fusion_method"];
        pbf_tracker_params.existence_fusion_method =
            nlohmann_json["DataFusionParams"]["TrackerParams"]
                         ["PbfTrackerParams"]["existence_fusion_method"];
        pbf_tracker_params.type_fusion_method =
            nlohmann_json["DataFusionParams"]["TrackerParams"]
                         ["PbfTrackerParams"]["type_fusion_method"];

// for fusion system param
#define sequential_fusion_params \
    Instance()->fusion_system_params_.sequential_fusion_params_

        sequential_fusion_params.data_association_method =
            nlohmann_json["FusionSystemParams"]["SequentialFusionParams"]
                         ["data_association_method"];
        sequential_fusion_params.track_method =
            nlohmann_json["FusionSystemParams"]["SequentialFusionParams"]
                         ["track_method"];
        sequential_fusion_params.new_track_existence_thresh =
            nlohmann_json["FusionSystemParams"]["SequentialFusionParams"]
                         ["new_track_existence_thresh"];
        sequential_fusion_params.new_track_start_thresh =
            nlohmann_json["FusionSystemParams"]["SequentialFusionParams"]
                         ["new_track_start_thresh"];
        sequential_fusion_params.new_track_length_thresh =
            nlohmann_json["FusionSystemParams"]["SequentialFusionParams"]
                         ["new_track_length_thresh"];
        sequential_fusion_params.track_merge_dist_thresh =
            nlohmann_json["FusionSystemParams"]["SequentialFusionParams"]
                         ["track_merge_dist_thresh"];

// for lane builder param
#define lane_builder_params Instance()->lane_builder_params_

        lane_builder_params.lane_min_width =
            nlohmann_json["LaneBuilderParams"]["lane_min_width"];
        lane_builder_params.lane_max_width =
            nlohmann_json["LaneBuilderParams"]["lane_max_width"];
        lane_builder_params.lane_max_width =
            std::max(lane_builder_params.lane_max_width,
                     2.0f * lane_builder_params.lane_min_width);
        lane_builder_params.lane_default_width =
            nlohmann_json["LaneBuilderParams"]["lane_default_width"];
        lane_builder_params.lane_width_filter_buffer_size =
            nlohmann_json["LaneBuilderParams"]["lane_width_filter_buffer_size"];
        lane_builder_params.max_lanes_num =
            nlohmann_json["LaneBuilderParams"]["max_lanes_num"];
        lane_builder_params.max_lanemarkers_num =
            nlohmann_json["LaneBuilderParams"]["max_lanemarkers_num"];
        lane_builder_params.lane_model_change_thresh =
            nlohmann_json["LaneBuilderParams"]["lane_model_change_thresh"];
        lane_builder_params.enable_gen_virtual_lanemarkers =
            nlohmann_json["LaneBuilderParams"]
                         ["enable_gen_virtual_lanemarkers"];
        lane_builder_params.re_correct_trigger_thresh =
            nlohmann_json["LaneBuilderParams"]["re_correct_trigger_thresh"];
        lane_builder_params.enable_check_virtual_lanemarkers =
            nlohmann_json["LaneBuilderParams"]
                         ["enable_check_virtual_lanemarkers"];
        lane_builder_params.Bev_delay_angle =
            nlohmann_json["LaneBuilderParams"]
                         ["Bev_delay_angle"];

// for post_processor_params
#define post_processor_params Instance()->post_processor_params_
        post_processor_params.tsrm_enable =
            nlohmann_json["PostProcessorParams"]["tsrm_enable"];
        post_processor_params.tsrm_priority_higher_fvcm =
            nlohmann_json["PostProcessorParams"]["tsrm_priority_higher_fvcm"];
        post_processor_params.forward_hdmap_curvature_dist_thresh =
            nlohmann_json["PostProcessorParams"]
                         ["forward_hdmap_curvature_dist_thresh"];
        post_processor_params.enable_forward_hdmap_curvature_forever =
            nlohmann_json["PostProcessorParams"]
                         ["enable_forward_hdmap_curvature_forever"];
        post_processor_params.tsrm_sleep_frame =
            nlohmann_json["PostProcessorParams"]["tsrm_sleep_frame"];
        post_processor_params.enable_sd_spd_lm =
            nlohmann_json["PostProcessorParams"]["enable_sd_spd_lm"];
        post_processor_params.enable_bev_shape_fuse =
            nlohmann_json["PostProcessorParams"]["enable_bev_shape_fuse"];
// for debug
#define debug_params Instance()->debug_params_

        debug_params.shield_hdmap_type_color =
            nlohmann_json["DebugParams"]["shield_hdmap_type_color"];

// for fle param
#define fle_params Instance()->fle_params_

        fle_params.trajectory_length_threshold =
            nlohmann_json["FLEParams"]["trajectory_length_threshold"];
        fle_params.lon_dist_end_threshold =
            nlohmann_json["FLEParams"]["lon_dist_end_threshold"];
        fle_params.weight_of_lane_markers =
            nlohmann_json["FLEParams"]["weight_of_lane_markers"];
        fle_params.weight_coeff_k_of_lane_markers =
            nlohmann_json["FLEParams"]["weight_coeff_k_of_lane_markers"];
        fle_params.weight_coeff_b_of_lane_markers =
            nlohmann_json["FLEParams"]["weight_coeff_b_of_lane_markers"];
        fle_params.weight_coeff_max_of_lane_markers =
            nlohmann_json["FLEParams"]["weight_coeff_max_of_lane_markers"];
        fle_params.weight_coeff_min_of_lane_markers =
            nlohmann_json["FLEParams"]["weight_coeff_min_of_lane_markers"];
        fle_params.weight_x_min_lane_markers =
            nlohmann_json["FLEParams"]["weight_x_min_lane_markers"];
        fle_params.weight_x_max_of_lane_markers =
            nlohmann_json["FLEParams"]["weight_x_max_of_lane_markers"];
        fle_params.weight_of_road_edges =
            nlohmann_json["FLEParams"]["weight_of_road_edges"];
        fle_params.weight_of_gtr = nlohmann_json["FLEParams"]["weight_of_gtr"];
        fle_params.weight_of_ego_trajectory =
            nlohmann_json["FLEParams"]["weight_of_ego_trajectory"];
        fle_params.weight_coeff_b_of_measuration =
            nlohmann_json["FLEParams"]["weight_coeff_b_of_measuration"];
        fle_params.weight_coeff_k_of_measuration =
            nlohmann_json["FLEParams"]["weight_coeff_k_of_measuration"];
        fle_params.weight_coeff_a_of_prediction =
            nlohmann_json["FLEParams"]["weight_coeff_a_of_prediction"];
        fle_params.weight_coeff_b_of_prediction =
            nlohmann_json["FLEParams"]["weight_coeff_b_of_prediction"];
        fle_params.curves_windows_num =
            nlohmann_json["FLEParams"]["curves_windows_num"];
        fle_params.recent_curves_size =
            nlohmann_json["FLEParams"]["recent_curves_size"];
        fle_params.edge_weight_threshold =
            nlohmann_json["FLEParams"]["edge_weight_threshold"];
        fle_params.meas_proporation_threshold =
            nlohmann_json["FLEParams"]["meas_proporation_threshold"];
        fle_params.last_ref_curve_filter_param =
            nlohmann_json["FLEParams"]["last_ref_curve_filter_param"];

#define diagnostic_params Instance()->diagnostic_params_
        diagnostic_params.enable_fvcm_failsafe_judgment =
            nlohmann_json["DiagnosticParams"]["enable_fvcm_failsafe_judgment"];

#define map_convert_params Instance()->map_convert_params_
        map_convert_params.map_stay_ld =
            nlohmann_json["MapConvertParams"]["map_stay_ld"];
        map_convert_params.map_stay_bev =
            nlohmann_json["MapConvertParams"]["map_stay_bev"];

        VLOG(1) << "Successfully read json configuration parameters! ";
    }
    catch (std::exception &e)
    {
        VLOG(1) << "Error reading json configuration parameters: " << e.what()
                  << std::endl;
        std::exit(1);
    }
}

void ParamsManager::ReadVersionInfoParams(const std::string &version_file_path)
{
    try
    {
        std::ifstream fstream_input(version_file_path);
        std::string line;

        while (!fstream_input.eof() && (VERSION_INFO_PARAMS.git_branch == "" ||
                                        VERSION_INFO_PARAMS.commit_id == ""))
        {
            std::getline(fstream_input, line);
            if (line.find("git branch") != std::string::npos)
            {
                VERSION_INFO_PARAMS.git_branch = line.substr(11);
            }
            else if (line.find("git commit") != std::string::npos)
            {
                VERSION_INFO_PARAMS.commit_id = line.substr(11);
            }
            else
            {
                // do nothing
            }
        }
    }
    catch (std::exception &e)
    {
        AERROR << "Error reading version info: " << e.what();
    }
}

} // namespace fusion
} // namespace cem
