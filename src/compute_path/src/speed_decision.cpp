#include "compute_path/compute_path.h"

/**
 * speed decision: overtake, yield
 */

void HybridAstar::SpeedDecisionProcess() {
  for (auto& obstacle : obstacles_info_.obstacles) {
    if(obstacle.is_static){
      continue;
    }

    obstacle.speed_decision = "overtake";
  }
}
