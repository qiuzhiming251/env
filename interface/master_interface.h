#ifndef MASTER_INTERFACE_H_
#define MASTER_INTERFACE_H_

#include <memory>
#include "modules/msg/basic_msgs/header.pb.h"
#include "modules/msg/basic_msgs/module_status.pb.h"
#include "modules/msg/drivers_msgs/ins.pb.h"
#include "modules/msg/drivers_msgs/veh_info.pb.h"
#include "modules/msg/environment_model_msgs/local_map_info.pb.h"
#include "modules/msg/localization_msgs/localization_info.pb.h"
#include "modules/msg/environment_model_msgs/sr_element.pb.h"
#include "modules/msg/orin_msgs/routing_map.pb.h"
#include "modules/msg/orin_msgs/map_event.pb.h"
#include "modules/msg/perception_msgs/perception_traffic_info.pb.h"
#include "modules/msg/orin_msgs/sm_msgs.pb.h"
#include "modules/msg/st_msgs/planning_result.pb.h"
#include "modules/msg/planning_msgs/plan_func_state.pb.h"
#include "modules/msg/drivers_msgs/sdmap_inform_service.pb.h"
#include "modules/msg/obj_info_fusn.pb.h"
#include "modules/msg/localization_msgs/map_match.pb.h"
#include "modules/msg/drivers_msgs/navi_traffic_info.pb.h"
#include "modules/msg/drivers_msgs/event.pb.h"

namespace cem {
    namespace fusion {

        class MasterInterface
        {

        public:
            virtual ~MasterInterface(){}

            virtual void Proc() = 0;

            virtual std::shared_ptr<byd::msg::basic::ModuleStatus> GetModuleStatusPtr() = 0;

            virtual std::shared_ptr<byd::msg::orin::routing_map::RoutingMap> GetENVRoutingMapPtr() = 0;

            virtual std::shared_ptr<byd::msg::orin::routing_map::TrafficLightsE2EInfo> GetTrafficLightsE2EPtr() = 0;

            /********************消息callback在此处定义****************************/
            virtual void OnVehInfoCallback(const std::shared_ptr<byd::msg::drivers::VehInfo>& msg) = 0;

            virtual void OnEnvModelLocalMapInfoCallback(const std::shared_ptr<byd::msg::env_model::LocalMapInfo>& msg) = 0;

            virtual void OnMsgDriverInsCallback(const std::shared_ptr<byd::msg::drivers::Ins>& msg) = 0;

            virtual void OnMsgLocationDrCallback(const std::shared_ptr<byd::msg::localization::LocalizationEstimate>& msg) = 0;

            virtual void OnRoutingMapCallBack(const std::shared_ptr<byd::msg::orin::routing_map::RoutingMap>& msg) = 0;

            virtual void OnMsgMapEventCallback(const std::shared_ptr<byd::msg::orin::routing_map::MapEvent>& msg) = 0;
            virtual void OnMsgObstaclesCallback(const std::shared_ptr<byd::msg::pnc::ObjInfoFusn>& msg) = 0;

            virtual void OnPercepTrfInfoCallback(const std::shared_ptr<byd::msg::perception::PerceptionTrafficInfo>& msg) = 0;
            virtual void OnVisionTrfInfoCallback(const std::shared_ptr<byd::msg::perception::PerceptionTrafficInfo>& msg) = 0;

            virtual void OnPerceptionObstaclesInfoCallback(const std::shared_ptr<byd::msg::perception::PerceptionObstacles>& msg) = 0;

            virtual void OnEnvModelLidaRoadedgeCallback(const std::shared_ptr<byd::msg::env_model::LocalMapInfo>& msg) = 0;

            virtual void OnPlanResultCallback(const std::shared_ptr<byd::msg::planning::PLanningResultProto> &msg) = 0;

            virtual void OnPlanFuncStateCallback(const std::shared_ptr<byd::msg::pnc::PlanFuncState> &msg) = 0;

            virtual void OncanoutCallback(const std::shared_ptr<byd::msg::orin::sm_msgs::MsgSM_BYD_CAN_Output>& msg) = 0;
            virtual void OnOccCallback(const std::shared_ptr<byd::msg::perception::OCCInfo>& msg) =0;
            virtual void OnNaviTrafCallback(const std::shared_ptr<byd::msg::drivers::TrafficInfoNotify> &msg) = 0;
            virtual void OnDriversEventCallback(const std::shared_ptr<byd::msg::drivers::Event> &msg) = 0;

            virtual void OnSDTrafficLightCallback(const std::shared_ptr<byd::msg::drivers::sdTrafficLight>& msg) = 0;
            virtual void OnMapLocationResultCallback(const std::shared_ptr<byd::modules::localization::MapMatchResult> &msg) = 0;
            virtual void OnMapLocationResultBaiduCallback(
                const std::shared_ptr<
                    byd::modules::localization::MapMatchResult>& msg) = 0;
        };
        std::shared_ptr<MasterInterface> GetMasterInstance();
    } // namespace fusion
} // namespace cem

#endif
