#ifndef BASE_PARAMS_H_
#define BASE_PARAMS_H_

#include <cstddef>

namespace cem {
namespace fusion {

struct BaseParams
{
    struct TrackParams
    {
        float low_speed_thresh = 5.0f;              // km/h
        size_t track_confirmed_thresh = 5;          // equal to 0.25s
        size_t track_normal_speed_lost_thresh = 20; // equal to 1s
        size_t track_low_speed_lost_thresh = 6000;  // equal to 5mins
        float first_seg_len_thresh = 5.0f;
    } track_params_;
};

} // namespace fusion
} // namespace cem

#endif
