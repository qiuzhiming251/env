#ifndef SENSOR_DATA_MANAGER_H_
#define SENSOR_DATA_MANAGER_H_

#include <memory>

#include "common/cpp/cpp_headers.h"
#include "message/internal_message.h"
#include "sensor.h"
#include "sensor_meta.h"
#include "cyber/common/log.h"

namespace cem {
namespace fusion {

#define SensorPtrType decltype(std::shared_ptr<Sensor<MSG_PTR_TYPE>>())
#define QuerySensor GetTupleElementByType<SensorPtrType>

class SensorDataManager
{
private:
    bool inited_ = false;

    std::tuple<std::shared_ptr<Sensor<LocalizationPtr>>,
               std::shared_ptr<Sensor<VehicleSignalPtr>>,
               std::shared_ptr<Sensor<Bcm50msPdu02Ptr>>,
               std::shared_ptr<Sensor<Ibs20msPdu08Ptr>>,
               std::shared_ptr<Sensor<Ibs20msPdu15Ptr>>,
               std::shared_ptr<Sensor<Sas10msPdu00Ptr>>,
               std::shared_ptr<Sensor<AdasIpd100msPdu12Ptr>>,
               std::shared_ptr<Sensor<Imcu20msPdu05Ptr>>,
               std::shared_ptr<Sensor<Icm100msPdu29Ptr>>,
               std::shared_ptr<Sensor<TsrmobjectPtr>>,
               std::shared_ptr<Sensor<BevMapInfoPtr>>,
               std::shared_ptr<Sensor<Mpd100msPdu04Ptr>>,
               std::shared_ptr<Sensor<Eps010msPdu00Ptr>>,
               std::shared_ptr<Sensor<PercepTrfInfoPtr>>,
               std::shared_ptr<Sensor<RoutingMapPtr>>,
               std::shared_ptr<Sensor<MapEventPtr>>,
               std::shared_ptr<Sensor<FusObjInfoPtr>>,
               std::shared_ptr<Sensor<LidarRoadEdgeInfoPtr>>,
               std::shared_ptr<Sensor<PLanningResultPtr>>,
               std::shared_ptr<Sensor<PlanFuncStatePtr>>,
               std::shared_ptr<Sensor<CAN1Ptr>>,
               std::shared_ptr<Sensor<OCCInfoPtr>>,
               std::shared_ptr<Sensor<SdTrafficLightPtr>>,
               std::shared_ptr<Sensor<MapMatchResultPtr>>,
               std::shared_ptr<Sensor<VisionTrfInfoPtr>>,
               std::shared_ptr<Sensor<NaviTrafficInfoPtr>>,
               std::shared_ptr<Sensor<DriversEventPtr>>,
               std::shared_ptr<Sensor<MapMatchResultBaiduPtr>>>
        sensors_;

private:
    /**
     * @brief Just for Reset method.
     *
     */
    struct Resetter
    {
        template <typename T>
        void operator()(T &sensor)
        {
            sensor.reset();
        }
    };

public:
    bool Init();

    void Reset();

public:
    /**
     * @brief Add sensor frame to corresponding buffer.
     *
     * @tparam MSG_PTR_TYPE The shared_ptr type of sensor frame.
     * @param frame_ptr Sensor frame, a shared_ptr.
     */
    template <typename MSG_PTR_TYPE>
    inline void AddSensorMeasurements(const MSG_PTR_TYPE &frame_ptr)
    {
        QuerySensor(sensors_)->AddFrame(frame_ptr);
    }

    /**
     * @brief Clear sensor frames.
     *
     * @tparam MSG_PTR_TYPE The shared_ptr type of sensor frame.
     */
    template <typename MSG_PTR_TYPE>
    inline void ClearSensorMeasurements()
    {
        QuerySensor(sensors_)->ClearFrames();
    }

    /**
     * @brief Query sensor frames whose timestamp is in range
     *        (latest_query_timestamp_, timestamp].
     *
     * @tparam MSG_PTR_TYPE The shared_ptr type of sensor frame.
     * @param timestamp Queried timestamp.
     * @param frames Queried sensor frames.
     */
    template <typename MSG_PTR_TYPE>
    inline void GetLatestSensorFrames(double timestamp,
                                      std::vector<MSG_PTR_TYPE> *frames)
    {
        if (frames == nullptr)
        {
            AERROR << "Nullptr error.";
            return;
        }

        QuerySensor(sensors_)->QueryLatestFrames(timestamp, frames);
    }

    /**
     * @brief Query the lastest sensor frame in the buffer.
     *
     * @tparam MSG_PTR_TYPE The shared_ptr type of sensor frame.
     * @param frame_ptr Queried sensor frame.
     * @return bool Whether or not found valid lastest sensor frame.
     */
    template <typename MSG_PTR_TYPE>
    inline bool GetLatestSensorFrame(MSG_PTR_TYPE &frame_ptr)
    {
        frame_ptr = QuerySensor(sensors_)->QueryLatestFrame();
        return (frame_ptr != nullptr);
    }

    /**
     * @brief Query the lastest sensor frame near the timestamp in the buffer
     *        with given time precision.
     *
     * @tparam MSG_PTR_TYPE The shared_ptr type of sensor frame.
     * @param timestamp Queried timestamp.
     * @param epsilon_s Time precision of queried timestamp.
     * @param frame_ptr Queried sensor frame.
     * @return bool Whether or not found valid lastest sensor frame.
     */
    template <typename MSG_PTR_TYPE>
    inline bool GetLatestSensorFrame(double timestamp, double epsilon_s,
                                     MSG_PTR_TYPE &frame_ptr)
    {
        frame_ptr =
            QuerySensor(sensors_)->QueryLatestFrame(timestamp, epsilon_s);
        return (frame_ptr != nullptr);
    }

    DECLARE_SINGLETON(SensorDataManager)
};

} // namespace fusion
} // namespace cem

#endif
