#ifndef EPS010MS_PDU00_H_
#define EPS010MS_PDU00_H_

namespace cem {
namespace message {
namespace sensor {

struct Eps010msPdu00
{
    cem::message::common::Header header;
    float iadasstrgdlvrdtoq = 0.0f;
    uint8_t iadasstrgdlvrdtoqv = 0;
    float iactupnonang = 0.0f;
    uint8_t iactupnonangv = 0;
    float iacturoadwhlang = 0.0f;
    uint8_t iacturoadwhlangv = 0;
    uint8_t ichlkahptresp = 0;
    uint8_t ichlkaangreqresp = 0;
    uint8_t ichlkaangreqresp_hf = 0;
    uint8_t ichlkatoqreqresp = 0;
    uint8_t ichlkatoqreqresp_hf = 0;
    float idrvrstrgdlvrdtoq = 0.0f;
    uint8_t idrvrstrgdlvrdtoqv = 0;
    uint8_t iepscurntdrvmd = 0;
    uint8_t iepsdrvngmdctrlinh = 0;
    uint8_t iepsflrsts = 0;
    uint8_t iepshflmtrsts = 0;
    uint8_t iepshfmontngresultsts = 0;
    uint8_t iepspriychnlflrsts = 0;
    uint8_t iepspriystsalvrc = 0;
    uint8_t iepspriystschksm = 0;
    float iepstoqvalaldmax = 0.0f;
    uint8_t iepstoqvalaldmaxv = 0;
    float iepstoqvalaldmin = 0.0f;
    uint8_t iepstoqvalaldminv = 0;
    uint8_t ieps_010ms_group00_reserve01 = 0;
    uint8_t ieps_010ms_group00_reserve03 = 0;
    uint8_t ieps_010ms_group00_reserve04 = 0;
    uint8_t istrgcustsetngdspcmd = 0;
    uint8_t e2e_status = 0;
};

} // namespace sensor
} // namespace message
} // namespace cem

#endif