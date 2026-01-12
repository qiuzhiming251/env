#ifndef ADAS_IPD100MS_PUD12_H_
#define ADAS_IPD100MS_PUD12_H_

namespace cem {
namespace message {
namespace sensor {

struct AdasIpd100msPdu12
{
    cem::message::common::Header header;
    uint32_t iautomainbeamlghtreq = 0;
    float idistsincetrgtcamr = 0.0f;
    uint32_t ifvcmblkd = 0;
    uint32_t ifvcmcalprgsreq = 0;
    uint32_t ifvcmfltsts = 0;
    uint32_t ifvcmspdlmtvalsts = 0;
    uint32_t iipd_100ms_group12_crc = 0;
    uint32_t iipd_100ms_group12_rc = 0;
    uint32_t iipd_100ms_group12_reserve01 = 0;
    uint32_t iipd_100ms_group12_reserve02 = 0;
    uint32_t ispdastcndstscamr = 0;
    uint32_t ispdastreqstscamr = 0;
    uint32_t ispdlmtmdlsts = 0;
    uint32_t ispdlmtmdresp = 0;
    float itrgtspdreqcamr = 0.0f;
    uint32_t e2e_status = 0;
};

} // namespace sensor
} // namespace message
} // namespace cem

#endif