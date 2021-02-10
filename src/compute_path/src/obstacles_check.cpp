/**
 * Obstacles collision check
 */

#include "compute_path/compute_path.h"

void HybridAstar::CreateObstaclesList() {
  obstacles_list_.clear();

  for (const auto& elem : obstacles_info_.obstacles) {
    geometry_msgs::Point temp_center;

    // get ceneter pose in start pose frame
    double temp_x = elem.center.x - prime_start_pose_.position.x;
    double temp_y = elem.center.y - prime_start_pose_.position.y;
    double temp_yaw = elem.center.z;

    //** 坐标相对位置不变，坐标系旋转，所以yaw取负
    double yaw = -tf::getYaw(prime_start_pose_.orientation);

    temp_center.x = temp_x * cos(yaw) - temp_y * sin(yaw);
    temp_center.y = temp_x * sin(yaw) + temp_y * cos(yaw);
    temp_center.z = NormalizeAngle(temp_yaw + yaw);

    obstacles_list_.push_back(GetObstacleFrame(temp_center, elem.obs_box));
  }
}

bool HybridAstar::ObstalcesCollision(const AstarNode* const node_p,
                                     const PATH_TYPE& update_set) {
  Collision collision_checker;

  collision_checker.obj_1_ = GetTruckFrame(*node_p);

  for (const auto& elem : obstacles_list_) {
    collision_checker.obj_2_ = elem;

    if (collision_checker.IsCollision()) {
      // std::cout << "*************collision********" << std::endl;
      return true;
    }
  }

  return false;
}
