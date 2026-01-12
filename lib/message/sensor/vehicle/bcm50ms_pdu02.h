#ifndef BCM50MS_PDU02_H_
#define BCM50MS_PDU02_H_

namespace cem {
namespace message {
namespace sensor {

struct Bcm50msPdu02
{
    cem::message::common::Header header;
    uint32_t idaytimerunninglampon = 0;
    uint32_t idipdbeamlghton = 0;
    uint32_t idircnindlampswsts = 0;
    uint32_t ifrtfoglghton = 0;
    uint32_t ifrtwiperparkposa = 0;
    uint32_t ifrtwiperswsts = 0;
    uint32_t ifrtwshrpumpa = 0;
    uint32_t ikeydetindx = 0;
    uint32_t ildircnio = 0;
    uint32_t ildircnindlghtf = 0;
    uint32_t imainbeamlghton = 0;
    uint32_t irdircnio = 0;
    uint32_t irdircnindlghtf = 0;
    uint32_t irrfoglghton = 0;
    uint32_t ivehsidelghtsts = 0;
    uint32_t e2e_status = 0;
};

} // namespace sensor
} // namespace message
} // namespace cem

#endif
