#ifndef IBS20MS_PDU08_H_
#define IBS20MS_PDU08_H_

namespace cem {
namespace message {
namespace sensor {

struct Ibs20msPdu08
{
    cem::message::common::Header header;
    uint64_t soc_recv_timestamp = 0;
    float ivselatacc = 0.0f;
    uint32_t ivselatacccrc = 0;
    uint32_t ivselataccrc = 0;
    uint32_t ivselataccv = 0;
    float ivselongtacc = 0.0f;
    uint32_t ivselongtaccv = 0;
    float ivehdynyawrate = 0.0f;
    uint32_t ivehdynyawratev = 0;
    uint32_t e2e_status = 0;
};

} // namespace sensor
} // namespace message
} // namespace cem

#endif
