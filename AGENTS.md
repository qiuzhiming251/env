# AGENTS.md

## Project Positioning
This repository is an environment-model module in an autonomous driving stack.
It consumes BEV static perception and map elements (LD map and SD map), performs perception-map fusion, and outputs lane-level local map for downstream planning/control.

## Runtime I/O
Input:
- BEV static perception elements
- LD map and SD map elements

Core processing:
- Local map construction and preprocessing
- Perception and LD map matching/fusion
- Map source switch (perception map vs HD map)
- Lane-level navigation recommendation

Output:
- LocalMap with lane-level navigation information for downstream modules

## High-Level Call Chain
1. `communication/message_adapter/cyberRT_adapter.cpp`
   - Message adapter layer.
   - Deserialize subscribed CyberRT messages into internal data structures.
   - Serialize module outputs back to published messages.
2. `app/master.cpp` (`Master::Proc`)
   - Main orchestration of env algorithm node.
   - Calls fusion, navigation, map source switch, and output publishing flow.
3. `interface/master_interface.h`
   - External interface abstraction exposed by env module.

## Architecture Map (Folder -> File -> Responsibility)

### app/
- `app/master.cpp`
  - Entry/caller layer.
  - `Master::Proc` drives the full algorithm pipeline and module interaction.

### communication/message_adapter/
- `communication/message_adapter/cyberRT_adapter.cpp`
  - Serialization/deserialization between middleware messages and internal structs.
- `communication/message_adapter/cyberRT_adapter.h`
  - Adapter interface definitions.

### interface/
- `interface/master_interface.h`
  - Encapsulated external interface for env module instance.

### lib/sd_navigation/
Navigation recommendation related code.

- `lib/sd_navigation/BevDataProcessor.cpp`
  - City navigation BEV preprocessing module.
- `lib/sd_navigation/CoarseMatching.cpp`
  - City navigation coarse matching module.
- `lib/sd_navigation/EnvInfoProcessor.cpp`
  - EnvInfo processing module.
  - Uses map event and route inputs (`ld_route` for highway and `sd_route` for city) to generate env info.
- `lib/sd_navigation/RoadSelector.cpp`
  - City road/lane selection module.
- `lib/sd_navigation/SDMapElementExtract.cc`
  - City SD map based junction extraction module.
- `lib/sd_navigation/SdNavigationCity.cpp`
  - City navigation recommendation implementation.
- `lib/sd_navigation/SdNavigationHighway.cpp`
  - Highway navigation recommendation implementation.
  - Includes highway BEV preprocessing, road selection, highway junction extraction, and related lane recommendation logic.

Other supporting files in this module:
- `lib/sd_navigation/GeometryLaneMatcher.cpp`: geometry matching helpers.
- `lib/sd_navigation/NavigationParams.cpp`: navigation parameter definitions.
- `lib/sd_navigation/SDMapTopoExtract.cpp`: SD map topology extraction helpers.

### lib/perception_and_ld_map_fusion/
Perception-map fusion module.

Focus submodule:
- `lib/perception_and_ld_map_fusion/map_source_switch/map_source_switch.cc`
- `lib/perception_and_ld_map_fusion/map_source_switch/map_source_switch.h`
  - Map source switching logic.

Related preprocessing submodule:
- `lib/perception_and_ld_map_fusion/data_preprocessor/bev_map_preprocessor.cc`
  - BEV data preprocessing.
- `lib/perception_and_ld_map_fusion/data_preprocessor/ld_map_preprocessor.cc`
  - LD map data preprocessing and scenario/mode related preprocessing logic.
- `lib/perception_and_ld_map_fusion/data_preprocessor/build_lane_topology.cc`
  - Lane topology building.
- `lib/perception_and_ld_map_fusion/data_preprocessor/lane_type_checker.cpp`
  - Lane type checking/adjustment helpers.
- `lib/perception_and_ld_map_fusion/data_preprocessor/traffic_flow_manager.cpp`
  - Traffic flow related preprocessing logic.

### lib/message/
Internal message structures used by env pipeline.

- `lib/message/env_model/navigation/navigation.h`
  - Core structures/enums used by navigation recommendation.
- `lib/message/env_model/routing_map/map_event.h`
  - Internal struct for subscribed map event topic.
  - Includes ODD/event information such as `odd_info` and `LOW_PRECISION` event type.
- `lib/message/env_model/routing_map/routing_map.h`
  - Core map input struct for env.
  - `RoutingMap` contains key map fields such as `lanes`, `lane_boundaries`, `stop_lines`, `cross_walks`, `arrows`, `RouteInfo route`, and `SDRouteInfo sd_route`.
- `lib/message/sensor/camera/bev_lane/bev_lane.h`
  - BEV static perception input definitions (`BevMapInfo`, `lanemarkers`, `lane_infos`, etc.).

## Current Requirement Scope (Very Important)
Current product requirement is highway-focused NOA.

Priority implementation path:
- `lib/sd_navigation/SdNavigationHighway.cpp`
- `lib/perception_and_ld_map_fusion/map_source_switch/`
- Highway-related call chain in `app/master.cpp`

City path policy:
- `lib/sd_navigation/SdNavigationCity.cpp` is city-scenario implementation.
- Treat city path as legacy/compatibility path.
- Do not modify city logic unless explicitly requested or required by interface constraints.

## Engineering Guardrails

### Highway navigation
- Prioritize correctness and continuity in highway lane recommendation.
- Avoid regressions around merge/split/ramp transitions.
- Keep behavior deterministic and reviewable.

### Highway map source switch
- Focus on highway branch (`HNOA`, freeway/ramp logic).
- Preserve `use_perception_` semantics:
  - `true` = perception map
  - `false` = HD map
- Preserve `EnvStatus` suppression reason semantics unless requirement explicitly changes them.
- If thresholds are changed, document old/new values and impacted conditions.

## Build/Test Environment Policy
Compile/runtime validation is on Linux.
Codex editing environment is Windows.

In this workspace:
- Do not require local Windows build/test as a blocking step.
- Perform static reasoning and call-chain consistency checks.
- For every functional change, always provide Linux validation commands.

Linux validation command references:
- `bazel build //:local_map_master`
- `bazel test //...` (or impacted targets)

## Coding Constraints
- Language: C++17
- Follow existing style and nearby code conventions
- Keep diffs minimal and task-focused
- Avoid unrelated refactor
- Preserve external interfaces unless explicitly requested

## Required Workflow
1. Before coding, always read `AGENTS.md`, `docs/ARCHITECTURE.md`, and `docs/FILE_MAP.md`.
2. Analyze impacted call chain (highway path first).
3. Define minimal patch scope.
4. Implement focused changes.
5. If tests are in scope, update/add test code (execution can be deferred to Linux).
6. Final report must include:
   - changed files
   - behavior delta
   - Linux validation commands
   - assumptions and residual risks

## Architecture Maintenance Rules
When touching architecture-sensitive code, update this file accordingly.

Must update AGENTS.md when:
- Call chain ownership changes (caller/callee relationship)
- Module/file responsibility changes
- New critical data structures are introduced
- Requirement scope changes (for example highway-only to mixed scope)

Keep updates concise and factual:
- Prefer one-line responsibility per file
- Separate "current requirement scope" from "legacy compatibility"
- Avoid stale comments that conflict with current product scope

## Prompt Pack (Highway-Only)
Use templates directly in Codex chat. Replace placeholders in `{...}`.

### Template 1: Highway Navigation Feature
Goal:
Implement `{feature_name}` for highway lane recommendation.

Context:
Current scope is highway-focused NOA. Prioritize `SdNavigationHighway` behavior.

Files in scope:
- `lib/sd_navigation/SdNavigationHighway.cpp`
- `lib/sd_navigation/SdNavigationHighway.h`
- Read-only context if needed: `app/master.cpp`

Constraints:
- C++17
- Keep external interfaces unchanged
- Minimal diff, no unrelated refactor
- Do not modify `SdNavigationCity` unless explicitly required

Definition of Done:
- Feature works in highway path
- No obvious highway regression in critical branches

Output format:
1. Changed files
2. Key logic changes
3. Unified diff
4. Linux validation commands
5. Risks/assumptions

### Template 2: Highway Map Source Switch Bugfix
Goal:
Fix `{bug_summary}` in highway map source switching.

Context:
Focus on `HNOA`/freeway/ramp branch and switching stability.

Files in scope:
- `lib/perception_and_ld_map_fusion/map_source_switch/map_source_switch.cc`
- `lib/perception_and_ld_map_fusion/map_source_switch/map_source_switch.h`
- `app/master.cpp` (only if caller alignment is needed)

Constraints:
- Preserve `use_perception_` semantics (`true`=perception map, `false`=HD map)
- Preserve existing `EnvStatus` suppression reason semantics unless explicitly requested
- Keep threshold changes centralized and documented

Definition of Done:
- Root cause identified with file/line references
- Fix implemented with minimal patch
- Highway call chain remains coherent

Output format:
1. Root cause analysis (file:line)
2. Fix summary
3. Unified diff
4. Linux validation commands
5. Regression checklist

### Template 3: Master-to-Module Consistency Audit
Goal:
Audit and fix consistency issues between `app/master.cpp` and `{module}` for `{topic}`.

Context:
Typical topics: highway scene flag, ramp flag, source switch state propagation, env_status output.

Files in scope:
- `app/master.cpp`
- `{module_files}`

Constraints:
- Keep data flow explicit
- Avoid behavior drift in highway path
- Minimal patch only

Definition of Done:
- Inconsistency list with evidence
- Minimal fix patch
- Before/after behavior notes

Output format:
1. Findings
2. Fix plan
3. Unified diff
4. Linux validation commands
5. Remaining open questions

## Prompt Usage Rules
- Always include: `Goal`, `Files in scope`, `Constraints`, `Definition of Done`, `Output format`.
- Ask for `unified diff` instead of code fragments.
- Keep one prompt focused on one engineering objective.
- For large work, split into multiple prompts (implementation/review/cleanup).

## Documentation Entry
Use these docs as the default architecture references:
- `docs/ARCHITECTURE.md`: system-level runtime/data-flow and module interaction.
- `docs/FILE_MAP.md`: file-level implementation responsibility index.

When module ownership or call chain changes, update these docs in the same change.
