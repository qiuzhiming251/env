#ifndef SENSOR_LABEL_H
#define SENSOR_LABEL_H

#include "common/utility.h"
namespace cem {
namespace fusion {
class SensorLabel
{
public:
    inline bool set(SensorLabel B)
    {
        type = B.type | type;
        return true;
    }

    inline bool set(ContributingSensors B)
    {
        type = static_cast<uint16_t>(B) | type;
        return true;
    }

    inline bool is(SensorLabel A) const
    {
        return static_cast<int>(type & A.type) == static_cast<int>(A.type);
    }

    inline bool is(ContributingSensors A) const
    {
        return static_cast<int>(type & static_cast<uint16_t>(A)) ==
               static_cast<int>(static_cast<uint16_t>(A));
    }

    inline void reset() { type = 0; }

    inline void erase(SensorLabel B) { type &= ~B.type; }

    inline uint16_t gettype() { return type; }

    SensorLabel(uint16_t type) : type(type) {}

    uint16_t type;

    inline SensorLabel operator~()
    {
        SensorLabel ans(~type);
        return ans;
    }
    inline SensorLabel operator&(const SensorLabel &other)
    {
        type &= other.type;
        return *this;
    }
};

} // namespace fusion
} // namespace cem

#endif // SENSOR_LABEL_H