#ifndef SAS10MS_PDU00_H_
#define SAS10MS_PDU00_H_

namespace cem {
namespace message {
namespace sensor {

struct Sas10msPdu00
{
    cem::message::common::Header header;
    uint32_t isas_010ms_group00_reserve01 = 0;
    uint32_t isas_010ms_group00_reserve02 = 0;
    float istrgwhlang = 0.0f;
    uint32_t istrgwhlangalvrc = 0;
    int32_t istrgwhlanggrd = 0;
    uint32_t istrgwhlangsnsrcrc = 0;
    uint32_t istrgwhlangsnsrcalsts = 0;
    uint32_t istrgwhlangsnsrflt = 0;
    uint32_t istrgwhlangsnsrinid = 0;
    uint32_t istrgwhlangsnsrmultcapb = 0;
    uint32_t istrgwhlangv = 0;
    uint64_t timestamp = 0;
    uint32_t e2e_status = 0;
};

} // namespace sensor
} // namespace message
} // namespace cem

#endif
