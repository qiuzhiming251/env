#include "sensor_data_manager.h"

#include <algorithm>
#include <utility>
#include "glog/logging.h"

namespace cem {
namespace fusion {

SensorDataManager::SensorDataManager() { CHECK_EQ(this->Init(), true); }

bool SensorDataManager::Init()
{
    if (!inited_)
    {
        std::get<0>(sensors_).reset(new Sensor<LocalizationPtr>(100));
        std::get<1>(sensors_).reset(new Sensor<VehicleSignalPtr>());
        std::get<2>(sensors_).reset(new Sensor<Bcm50msPdu02Ptr>());
        std::get<3>(sensors_).reset(new Sensor<Ibs20msPdu08Ptr>());
        std::get<4>(sensors_).reset(new Sensor<Ibs20msPdu15Ptr>());
        std::get<5>(sensors_).reset(new Sensor<Sas10msPdu00Ptr>());
        std::get<6>(sensors_).reset(new Sensor<AdasIpd100msPdu12Ptr>());
        std::get<7>(sensors_).reset(new Sensor<Imcu20msPdu05Ptr>());
        std::get<8>(sensors_).reset(new Sensor<Icm100msPdu29Ptr>());
        std::get<9>(sensors_).reset(new Sensor<TsrmobjectPtr>());
        std::get<10>(sensors_).reset(new Sensor<BevMapInfoPtr>());
        std::get<11>(sensors_).reset(new Sensor<Mpd100msPdu04Ptr>());
        std::get<12>(sensors_).reset(new Sensor<Eps010msPdu00Ptr>());
        std::get<13>(sensors_).reset(new Sensor<PercepTrfInfoPtr>());
        std::get<14>(sensors_).reset(new Sensor<RoutingMapPtr>());
        std::get<15>(sensors_).reset(new Sensor<MapEventPtr>());
        std::get<16>(sensors_).reset(new Sensor<FusObjInfoPtr>());
        std::get<17>(sensors_).reset(new Sensor<LidarRoadEdgeInfoPtr>());
        std::get<18>(sensors_).reset(new Sensor<PLanningResultPtr>());
        std::get<19>(sensors_).reset(new Sensor<PlanFuncStatePtr>());
        std::get<20>(sensors_).reset(new Sensor<CAN1Ptr>());
        std::get<21>(sensors_).reset(new Sensor<OCCInfoPtr>());
        std::get<22>(sensors_).reset(new Sensor<SdTrafficLightPtr>());
        std::get<23>(sensors_).reset(new Sensor<MapMatchResultPtr>());
        std::get<24>(sensors_).reset(new Sensor<VisionTrfInfoPtr>());
        std::get<25>(sensors_).reset(new Sensor<NaviTrafficInfoPtr>());
        std::get<26>(sensors_).reset(new Sensor<DriversEventPtr>());
        std::get<27>(sensors_).reset(new Sensor<MapMatchResultBaiduPtr>());
        inited_ = true;
    }

    return true;
}

void SensorDataManager::Reset()
{
    inited_ = false;
    ForEachInTuple(sensors_, Resetter());
}

} // namespace fusion
} // namespace cem
