load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_tar")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library","cc_import")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

config_setting(name="x86_mode", values={"cpu": "k8"})

config_setting(name="arm_mode", values={"cpu": "aarch64"})

config_setting(
    name = "traffic_log_enable",
    values = {"define": "traffic_log_enable=true"},
)
config_setting(
    name = "geometry_log_enable",
    values = {"define": "geometry_log_enable=true"},
)
config_setting(
    name = "fml_trafficlight_log_enable",
    values = {"define": "fml_trafficlight_log_enable=true"},
)

config_setting(
    name = "sd_extract_map_element_enable",
    values = {"define": "sd_extract_map_element_enable=true"},
)

config_setting(
    name = "sd_junction_convert_enable",
    values = {"define": "sd_junction_convert_enable=true"},
)

config_setting(
    name = "sd_coarse_match_enable",
    values = {"define": "sd_coarse_match_enable=true"},
)

config_setting(
    name = "sd_coarse_match_type2_enable",
    values = {"define": "sd_coarse_match_type2_enable=true"},
)

config_setting(
    name = "sd_merge_log_enable",
    values = {"define": "sd_merge_log_enable=true"},
)


config_setting(
    name = "sd_bev_process_log_enable",
    values = {"define": "sd_bev_process_log_enable=true"},
)


config_setting(
    name = "sd_fine_match_enable",
    values = {"define": "sd_fine_match_enable=true"},
)

config_setting(
    name = "stop_line_mapping_enable",
    values = {"define": "stop_line_mapping_enable=true"},
)

config_setting(
    name = "sd_env_info_enable",
    values = {"define": "sd_env_info_enable=true"},
)

config_setting(
    name = "sd1debug",
    values = {"define": "sd1debug=true"},
)

config_setting(
    name = "fml_cross_log_enable",
    values = {"define": "fml_cross_log_enable=true"},
)
config_setting(
    name = "xxf_log_enable",
    values = {"define": "xxf_log_enable=true"},
)

config_setting(
    name = "trafficlight_rec_log_enable",
    values = {"define": "trafficlight_rec_log_enable=true"},
)

config_setting(
    name = "pc_x86_run_flag",
    values = {"define": "pc_x86_run_flag=true"},
)

cc_library(
    name = "local_map_master",
    srcs = glob(["app/**/*.cpp", "communication/**/*.cpp", "middleware/**/*.cpp","lib/**/*.cpp","lib/**/**/*.cpp","lib/**/*.cc","lib/**/**/*.cc"]),
    hdrs = glob(["**/*.h", "**/*.hpp","**/**/*.h", "**/**/*.hpp", "**/**/**/*.h"]),
    includes = [".","./app",
    "./lib/",
    "./lib/pre_processor/",
    "./lib/pre_processor/gtr/",
    "./communication",
    "./lib/data_association/",
    "./lib/common",
    "./lib/common/filter" ,
    "./lib/fusion_system",
    "./lib/data_fusion",
    "./lib/lane_builder",
    "./interface",
    "./test"
    ],
    deps = [
        "@eigen",
        "@boost//:boost",
        "@osqp//:osqp",
        "@ceres//:ceres",
        "@amp//:amp",
        "@opencv//:opencv",
        "@com_github_nlohmann_json//:json",
        "@fmt//:fmt",
        "@magic_enum//:magic_enum",
        ###"@geographic//:geographic",
        "//cyber_release",
        "@glog_gflags//:glog_gflags",
        "//modules/msg/drivers_msgs:veh_info_cc_proto",
        "//modules/msg/environment_model_msgs:local_map_info_cc_proto",
        "//modules/msg/orin_msgs:lane_fusion_msgs_cc_proto",
        "//modules/msg/orin_msgs:ba_msgs_cc_proto",
        "//modules/msg/orin_msgs:ehr_msgs_cc_proto",
        "//modules/msg/orin_msgs:sm_msgs_cc_proto",
        "//modules/msg/orin_msgs:e2e_map_cc_proto",
        "//modules/msg/drivers_msgs:ins_cc_proto",
        "//modules/msg/drivers_msgs:sdmap_inform_service_cc_proto",
        "//modules/msg/basic_msgs:header_cc_proto",
        "//modules/msg/localization_msgs:localization_info_cc_proto",
        "//modules/msg/localization_msgs:map_match_cc_proto",
        "//modules/msg/basic_msgs:module_status_cc_proto",
        "//modules/mpc/common_files/include_msg:headers",
        "//modules/msg/perception_msgs:perception_traffic_info_cc_proto",
        "//modules/msg/perception_msgs:dynamic_object_cc_proto",
        "//modules/msg/st_msgs:planning_result_cc_proto",
        "//modules/msg/drivers_msgs:navi_traffic_info_cc_proto",
        "//modules/msg/environment_model_msgs:scenario_reality_element_cc_proto",
        "//modules/msg/planning_msgs:plan_func_state_cc_proto",
        "//modules/msg/orin_msgs:routing_map_cc_proto",
        "//modules/msg/orin_msgs:map_event_cc_proto",
        "//modules/msg:obj_info_fusn_cc_proto",
        "//modules/simulator/deterministic_scheduler:deterministic_scheduler",
        "//modules/common/math:geometry",
        "//modules/msg/drivers_msgs:event_cc_proto"
    ],
    copts = ["-DMW_TYPE_CyberRT",
             "-Wno-error=unused-variable",
             "-std=c++17",
             "-Wno-error=sign-compare",
             "-Wno-error=unused-variable",
             "-Wno-error=unused-but-set-variable",
             "-Wno-error=reorder",],
    alwayslink = True,
    visibility = ["//visibility:public"],
    defines = select({
        ":traffic_log_enable":  ["TRAFFIC_LIGHT_LOG=1"],
        "//conditions:default": ["TRAFFIC_LIGHT_LOG=0"],
    }) + select({
        ":geometry_log_enable": ["MAP_CHANGE_LOG=1"],
        "//conditions:default": ["MAP_CHANGE_LOG=0"],
    }) + select({
        ":fml_trafficlight_log_enable": ["FML_TRALHT_LOG=1"],
        "//conditions:default": ["FML_TRALHT_LOG=0"],
    }) + select({
        "sd_extract_map_element_enable": ["SD_EXTRACT_MAP_ELEMENT_LOG=1"],
        "//conditions:default": ["SD_EXTRACT_MAP_ELEMENT_LOG=0"],
    }) + select({
        "sd_junction_convert_enable": ["SD_JUNCTION_CONVERTE=1"],
        "//conditions:default": ["SD_JUNCTION_CONVERTE=0"],
    }) + select({
        "sd_env_info_enable": ["SD_ENV_INFO_LOG=1"],
        "//conditions:default": ["SD_ENV_INFO_LOG=0"],
    }) + select({
        "sd_coarse_match_enable": ["SD_COARSE_MATCH=1"],
        "//conditions:default": ["SD_COARSE_MATCH=0"],
    }) + select({
        "sd_coarse_match_type2_enable": ["SD_COARSE_MATCH_TYPE2=1"],
        "//conditions:default": ["SD_COARSE_MATCH_TYPE2=0"],
    }) + select({
        "sd_fine_match_enable": ["SD_FINE_MATCH=1"],
        "//conditions:default": ["SD_FINE_MATCH=0"],
    }) + select({
        "stop_line_mapping_enable": ["STOP_LINE_MAPPING=1"],
        "//conditions:default": ["STOP_LINE_MAPPING=0"], 
    }) + select({
        "sd1debug": ["SD_NAVIGATION_DEBUG=1"],
        "//conditions:default": ["SD_NAVIGATION_DEBUG=0"],
    }) + select({
        ":fml_cross_log_enable": ["FML_CROSS_LOG=1"],
        "//conditions:default": ["FML_CROSS_LOG=0"],  
    }) + select({
        ":trafficlight_rec_log_enable": ["TRAFFIC_REC=1"],
        "//conditions:default": ["TRAFFIC_REC=0"],  
    })+ select({
        ":sd_merge_log_enable": ["SD_MERGE=1"],
        "//conditions:default": ["SD_MERGE=0"],  
    })+ select({
        ":sd_bev_process_log_enable": ["SD_BEV_PROCESS=1"],
        "//conditions:default": ["SD_BEV_PROCESS=0"],  
    })+ select({
        ":xxf_log_enable": ["XXF_LOG=1"],
        "//conditions:default": ["XXF_LOG=0"],  
    })+ select({
        "pc_x86_run_flag": ["PC_X86_RUN_FLAG=1"],
        "//conditions:default": ["PC_X86_RUN_FLAG=0"],  
    }),
)