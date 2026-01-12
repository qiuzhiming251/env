#ifndef ICM100MS_PDU29_H_
#define ICM100MS_PDU29_H_

namespace cem {
namespace message {
namespace sensor {

struct Icm100msPdu29
{
    cem::message::common::Header header;
    uint8_t e2e_status = 0;
    uint16_t ificmposngsysdircn = 0;
    uint16_t inavcountrycode = 0;
    uint8_t inavcrntroadtyp = 0;
    float inavdistfromfrtelecceye = 0.0f;
    uint8_t inavnewicon = 0;
    float inavnewicondist;
    uint8_t inavspdlmttyp = 0;
    float inavspdlmtval = 0.0f;
    uint8_t inavspdlmtvalsts = 0;
    uint8_t inavspdlmtvaltyp = 0;
    uint8_t inavspdlmtvalunit = 0;
};

} // namespace sensor
} // namespace message
} // namespace cem

#endif
