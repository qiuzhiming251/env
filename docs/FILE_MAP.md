# FILE_MAP.md

## Purpose
This document is an initial file-level architecture map for the `env` repository.
It focuses on `*.cpp` and `*.cc` implementation files and gives one-line responsibilities.

Related:
- `docs/ARCHITECTURE.md`: system-level runtime/data-flow architecture.
- `docs/FILE_MAP.md`: file-level index and responsibilities.
Notes:
- This is an initial draft for fast navigation and onboarding.
- Descriptions for core paths (`master`, `sd_navigation`, `map_source_switch`) are higher confidence.
- Utility/common modules are described by naming and nearby usage; refine over time.

## Hot Path (Runtime)
1. `communication/message_adapter/cyberRT_adapter.cpp`: middleware message adapter (serialize/deserialize).
2. `app/master.cpp`: `Master::Proc` orchestrates full env pipeline.
3. `lib/pre_processor/*`: preprocess routing map and BEV map.
4. `lib/perception_and_ld_map_fusion/*`: map-perception matching/fusion + map source switch.
5. `lib/sd_navigation/*`: lane-level navigation recommendation (highway/city branches).
6. `lib/localmap_construction/*` + `lib/map_fusion/*`: construct and enrich local map output.
7. `communication/messenger.cpp`: publish outputs and diagnostics.

## File Map

### app/
- `app/master.cpp`: main caller/orchestration layer for env algorithm node (`Master::Proc`).

### communication/
- `communication/message_adapter/cyberRT_adapter.cpp`: CyberRT message adaptation between external proto and internal structs.
- `communication/messenger.cpp`: output publishing, callback routing, and module-level message dispatch support.

### middleware/
- `middleware/middleware_util.cpp`: middleware helper utilities and integration glue code.

### lib/base/
- `lib/base/localmap_conf.cpp`: local map runtime configuration loading/access.
- `lib/base/params_manager/params_manager.cpp`: parameter manager implementation.
- `lib/base/sensor_data_manager.cpp`: sensor data buffering and frame query service.

### lib/common/
- `lib/common/cpp/proto_helper.cpp`: proto conversion/helper utilities.
- `lib/common/debug_infos/debug_infos.cpp`: debug information aggregation/formatting.
- `lib/common/design_pattern/factory_method/factory_method.cpp`: factory method utility implementation.
- `lib/common/dst/dst_evidence.cpp`: DST evidence data processing helper.
- `lib/common/filter/custom_kalman_filter/custom_kalman_filter.cpp`: custom Kalman filter implementation.
- `lib/common/filter/kalman_filter/kalman_filter.cpp`: generic Kalman filter implementation.
- `lib/common/fitter/bezier_points.cpp`: bezier fitting helper.
- `lib/common/fitter/bspline_segment.cpp`: B-spline segment fitting helper.
- `lib/common/fitter/least_square_fitter.cpp`: least-square fitting wrapper.
- `lib/common/fitter/least_squares_solver.cpp`: least-square solver core.
- `lib/common/fitter/least_squares_solver_plus.cpp`: enhanced least-square solver variant.
- `lib/common/fitter/two_segment_bezier_interpolator.cpp`: two-segment bezier interpolation helper.
- `lib/common/math/math.cpp`: common math helpers.
- `lib/common/optimizer/cubic_polynomial_optimizer/cubic_polynomial_optimizer.cpp`: cubic polynomial optimization utility.
- `lib/common/time/TimeCostTool.cpp`: runtime timing and cost measurement utility.
- `lib/common/utility.cpp`: general common utility functions.
- `lib/common/utils/GeoMathUtil.cpp`: geometric math utility helpers.
- `lib/common/utils/Hungarian.cpp`: Hungarian algorithm implementation.
- `lib/common/utils/KDTreeUtil.cpp`: KD-tree search utility.
- `lib/common/utils/lane_geometry.cpp`: lane geometry computation helpers.

### lib/data_fusion/
- `lib/data_fusion/geometry_fusion/spatial_relationships.cc`: geometry/spatial relationship calculations for fusion logic.

### lib/localmap_construction/
- `lib/localmap_construction/cloud_config_traffic_light.cpp`: traffic light cloud/config related processing.
- `lib/localmap_construction/crosswalk_mapping.cpp`: crosswalk mapping/tracking.
- `lib/localmap_construction/diversion_mapping.cpp`: diversion/topology mapping logic.
- `lib/localmap_construction/intersection_mapping.cpp`: intersection mapping logic.
- `lib/localmap_construction/lane_mapping.cpp`: lane mapping and association.
- `lib/localmap_construction/lane_topology_processor.cpp`: lane topology post-processing.
- `lib/localmap_construction/routing_map_stopline_processor.cpp`: stopline correction/alignment on routing map.
- `lib/localmap_construction/stopline_mapping.cpp`: stopline mapping.
- `lib/localmap_construction/traffic_flow_mapping.cpp`: traffic flow mapping logic.
- `lib/localmap_construction/traffic_light_common.cpp`: shared traffic light helpers.
- `lib/localmap_construction/traffic_light_map_topo.cpp`: traffic light topology relation logic.
- `lib/localmap_construction/traffic_light_mapping.cpp`: traffic light mapping process.
- `lib/localmap_construction/traffic_light_tracker.cpp`: traffic light tracking logic.

#### lib/localmap_construction/crossroad_construction/
- `lib/localmap_construction/crossroad_construction/cross_data_manager.cpp`: crossroad construction data management.
- `lib/localmap_construction/crossroad_construction/cross_toplane_processor.cpp`: top lane processing inside crossroad scenes.
- `lib/localmap_construction/crossroad_construction/crossroad_construction.cpp`: crossroad construction main flow.
- `lib/localmap_construction/crossroad_construction/crossroad_lane_selector.cpp`: lane selection in crossroad construction.
- `lib/localmap_construction/crossroad_construction/edge_merger.cpp`: edge merge logic for crossroad boundaries.
- `lib/localmap_construction/crossroad_construction/kde_regression.cpp`: KDE regression helper in crossroad processing.
- `lib/localmap_construction/crossroad_construction/kde_smoother.cpp`: KDE smoothing helper.
- `lib/localmap_construction/crossroad_construction/lane_line_refiner.cpp`: lane line refinement in crossroad scenes.
- `lib/localmap_construction/crossroad_construction/laneline_cluster.cpp`: lane line clustering logic.
- `lib/localmap_construction/crossroad_construction/occ_processor.cpp`: OCC data processing for crossroad logic.
- `lib/localmap_construction/crossroad_construction/point_dbscan.cpp`: DBSCAN point clustering utility.
- `lib/localmap_construction/crossroad_construction/road_connector.cpp`: road connection construction logic.
- `lib/localmap_construction/crossroad_construction/virtual_egoroad_processor.cpp`: virtual ego-road generation/refinement.

### lib/map_fusion/
- `lib/map_fusion/extend_lane.cpp`: lane extension logic.
- `lib/map_fusion/lane_guidance.cpp`: lane guidance generation and output alignment.
- `lib/map_fusion/map_fusion.cpp`: fused map assembly logic.
- `lib/map_fusion/road_divider.cpp`: road divider related fusion logic.
- `lib/map_fusion/speed_limit_fusion.cpp`: speed limit fusion from multiple sources.

### lib/perception_and_ld_map_fusion/
- `lib/perception_and_ld_map_fusion/fusion_manager.cc`: main manager for BEV/LD preprocessing, matching, and fusion control.

#### common/
- `lib/perception_and_ld_map_fusion/common/kalman_filter.cc`: Kalman filter helper for fusion internals.
- `lib/perception_and_ld_map_fusion/common/pose_filte.cc`: pose filtering helper for fusion internals.

#### data_fusion/
- `lib/perception_and_ld_map_fusion/data_fusion/bev_filleting_machine.cc`: BEV slicing/filleting processing.
- `lib/perception_and_ld_map_fusion/data_fusion/geometry_match_info.cc`: geometry match result computation and aggregation.
- `lib/perception_and_ld_map_fusion/data_fusion/ld_map_filleting_machine.cc`: LD map slicing/filleting processing.
- `lib/perception_and_ld_map_fusion/data_fusion/match_maker.cc`: BEV-LD matching core.
- `lib/perception_and_ld_map_fusion/data_fusion/multi_source_data_mixer.cc`: multi-source data mixing/fusion logic.
- `lib/perception_and_ld_map_fusion/data_fusion/road_slice.cc`: road slicing representation/logic.

#### data_preprocessor/
- `lib/perception_and_ld_map_fusion/data_preprocessor/bev_map_preprocessor.cc`: BEV preprocessing for fusion pipeline.
- `lib/perception_and_ld_map_fusion/data_preprocessor/build_lane_topology.cc`: lane topology build/repair in preprocessing.
- `lib/perception_and_ld_map_fusion/data_preprocessor/lane_type_checker.cpp`: lane type checking/normalization.
- `lib/perception_and_ld_map_fusion/data_preprocessor/ld_map_preprocessor.cc`: LD map preprocessing, mode/scenario related preparation.
- `lib/perception_and_ld_map_fusion/data_preprocessor/test.cpp`: local preprocessor test/draft utility file.
- `lib/perception_and_ld_map_fusion/data_preprocessor/traffic_flow_manager.cpp`: traffic flow manager in preprocessing stage.

#### map_source_switch/
- `lib/perception_and_ld_map_fusion/map_source_switch/map_source_switch.cc`: map source switch logic (highway/city branches, downgrade checks, force-switch rules).

#### visualization/
- `lib/perception_and_ld_map_fusion/visualization/visualizer.cc`: debug visualization support for fusion internals.

### lib/pre_processor/
- `lib/pre_processor/pre_processor.cpp`: preprocessing coordinator for upstream map/perception inputs.
- `lib/pre_processor/bev_map/bev_map_pre_processor.cpp`: BEV map preprocessing before downstream fusion/navigation use.
- `lib/pre_processor/routing_map/routing_map_pre_processor.cpp`: routing map preprocessing before downstream use.

### lib/sd_navigation/
- `lib/sd_navigation/BevDataProcessor.cpp`: city navigation BEV preprocessing module.
- `lib/sd_navigation/CoarseMatching.cpp`: city navigation coarse matching module.
- `lib/sd_navigation/EnvInfoProcessor.cpp`: env info processing using map event and route context (`ld_route`/`sd_route`).
- `lib/sd_navigation/GeometryLaneMatcher.cpp`: geometry lane match utilities for navigation.
- `lib/sd_navigation/NavigationParams.cpp`: navigation parameters/defaults management.
- `lib/sd_navigation/RoadSelector.cpp`: city road/lane selection module.
- `lib/sd_navigation/SDMapElementExtract.cc`: city SD map element/junction extraction module.
- `lib/sd_navigation/SDMapTopoExtract.cpp`: SD map topology extraction helper module.
- `lib/sd_navigation/SdNavigationCity.cpp`: city navigation recommendation implementation.
- `lib/sd_navigation/SdNavigationHighway.cpp`: highway navigation recommendation implementation (highway BEV processing, selection, junction extraction, and recommendation flow).

### test/
- `test/kde_test.cc`: gtest-based KDE-related unit test.

## Ownership Focus (Current)
- Primary working areas:
  - `lib/sd_navigation/SdNavigationHighway.cpp`
  - `lib/perception_and_ld_map_fusion/map_source_switch/map_source_switch.cc`
  - `app/master.cpp` (highway path caller alignment)
- Legacy compatibility path:
  - `lib/sd_navigation/SdNavigationCity.cpp` (modify only when explicitly requested)

## Maintenance Rules
When adding/removing/renaming implementation files:
1. Update this file in the same change.
2. Keep one-line responsibility per file.
3. Mark uncertain descriptions explicitly with `(needs refinement)` if needed.



