#ifndef IBS20MS_PDU15_H_
#define IBS20MS_PDU15_H_

namespace cem {
namespace message {
namespace sensor {

struct Ibs20msPdu15
{
    cem::message::common::Header header;
    uint32_t iibs_20ms_group15_reserve01 = 0;
    float ivehspdavg = 0.0f;
    uint32_t ivehspdavgalvrc = 0;
    uint32_t ivehspdavgcrc = 0;
    float ivehspdavgdrvn = 0.0f;
    uint32_t ivehspdavgdrvnsrc = 0;
    uint32_t ivehspdavgdrvnv = 0;
    float ivehspdavgnondrvn = 0.0f;
    uint32_t ivehspdavgnondrvnv = 0;
    uint32_t ivehspdavgv = 0;
    uint64_t timestamp = 0;
    uint32_t e2e_status = 0;
};

} // namespace sensor
} // namespace message
} // namespace cem

#endif
