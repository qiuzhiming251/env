#ifndef MPD100MS_PDU04_H_
#define MPD100MS_PDU04_H_

namespace cem {
namespace message {
namespace sensor {

struct Mpd100msPdu04
{
    cem::message::common::Header header;
    uint8_t e2e_status = 0;
    uint8_t iautolanechngmsgreq = 0;
    uint8_t iautolanechngsetresp = 0;
    uint8_t iessswdspcmd = 0;
    uint8_t ihfrmndrswdspcmd = 0;
    uint8_t ijafnspgoswdspcmd = 0;
    uint8_t ijamsgreq = 0;
    uint8_t ilcrinhmi = 0;
    uint8_t ilcrmsgreq = 0;
    uint8_t ilcrswdspcmd = 0;
    uint8_t inopcityswdspcmd = 0;
    uint8_t inopswdspcmd = 0;
    uint8_t inopsysmsg = 0;
    uint8_t irpilotecomdstsdspcmd = 0;
    uint8_t irpilotecomdswavlbldspcmd = 0;
    uint8_t itrfclghtfnswdspcmd = 0;
};

} // namespace sensor
} // namespace message
} // namespace cem

#endif
