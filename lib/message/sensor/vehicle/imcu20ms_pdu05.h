#ifndef IMCU20MS_PDU05_H_
#define IMCU20MS_PDU05_H_

namespace cem {
namespace message {
namespace sensor {

struct Imcu20msPdu05
{
    cem::message::common::Header header;
    uint8_t e2e_status = 0;
    uint8_t ieptrdy = 0;
    uint8_t iimcu_025ms_group05_crc = 0;
    uint8_t iimcu_025ms_group05_rc = 0;
    uint8_t iimcu_025ms_group05_reserve01 = 0;
    uint8_t iimcu_025ms_group05_reserve02 = 0;
    uint16_t iimcu_025ms_group05_reserve03 = 0;
    uint16_t iimcu_025ms_group05_reserve04 = 0;
    uint8_t itrestdgear = 0;
    uint8_t itrshftlvrpos = 0;
    uint8_t itrshftlvrposv = 0;
};

} // namespace sensor
} // namespace message
} // namespace cem

#endif
