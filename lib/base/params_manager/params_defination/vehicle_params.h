#ifndef VEHICLE_PARAMS_H_
#define VEHICLE_PARAMS_H_

namespace cem {
namespace fusion {

struct VehicleParams
{
    float vehicle_width = 1.925f;
    float vehicle_length = 4.9f;
    float cam_to_rear_axle_center_lon_dist = 1.912f;
    float cam_to_rear_axle_center_lat_dist = 0.078f;
};

} // namespace fusion
} // namespace cem

#endif
