#ifndef AP_IPD100MS_PUD12_H_
#define AP_IPD100MS_PUD12_H_

namespace cem {
namespace message {
namespace sensor {

struct ApIpd100msPdu12
{
    cem::message::common::Header header;
    uint8_t e2e_status;
    uint8_t iautomainbeamlghtreq;
    float idistsincetrgtcamr;
    uint8_t ifvcmblkd;
    uint8_t ifvcmcalprgsreq;
    uint8_t ifvcmfltsts;
    uint8_t ifvcmspdlmtvalsts;
    uint8_t iipd_100ms_group12_crc;
    uint8_t iipd_100ms_group12_rc;
    uint8_t iipd_100ms_group12_reserve01;
    uint8_t iipd_100ms_group12_reserve02;
    uint8_t ispdastcndstscamr;
    uint8_t ispdastreqstscamr;
    uint8_t ispdlmtmdresp;
    uint8_t ispdlmtmdlsts;
    uint8_t itsrspdlimunit;
    float itrgtspdreqcamr;
};

} // namespace sensor
} // namespace message
} // namespace cem

#endif