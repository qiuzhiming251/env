#pragma once

#include "lib/perception_and_ld_map_fusion/common/kalman_filter.h"

namespace cem {
namespace fusion {

class PoseFilter : public KalmanFilter {
 public:
  PoseFilter(int state_size, int meas_size, int u_size);
  ~PoseFilter();
 private:
  void SetF(double delta_t) override;
};

}  // namespace fusion
}  // namespace cem