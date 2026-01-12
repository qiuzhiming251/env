#include "lib/perception_and_ld_map_fusion/common/pose_filter.h"

namespace cem {
namespace fusion {

PoseFilter::PoseFilter(int state_size, int meas_size, int u_size)
  : KalmanFilter(state_size, meas_size, u_size) {
}

PoseFilter::~PoseFilter() {}

void PoseFilter::SetF(double delta_t) {
    //   |heading|     |1  0  0  0  0     0     0    |    |heading|
    //   |   x   |     |0  1  0  t  0 0.5*t^2   0    |    |   x   |
    //   |   y   |     |0  0  1  0  t     0  0.5*t^2 |    |   y   |
    //   |  vx   |  =  |0  0  0  1  0     t     0    |  x |  vx   |
    //   |  vy   |     |0  0  0  0  1     0     t    |    |  vy   |
    //   |  ax   |     |0  0  0  0  0     1     0    |    |  ax   |
    //   |  ay   |     |0  0  0  0  0     0     1    |    |  ay   |
    //
    //              |________transfer_mat_______|
    // unit: delta_t/sec  heading/rad pos/m velocity/ m*s^-1  acc/m*s^-2
  F_.setIdentity();
  F_(1, 3) = delta_t;
  F_(1, 5) = 0.5 * delta_t * delta_t;
  F_(2, 4) = delta_t;
  F_(2, 6) = 0.5 * delta_t * delta_t;
  F_(3, 5) = delta_t;
  F_(4, 6) = delta_t;
};

}  // namespace fusion
}  // namespace cem