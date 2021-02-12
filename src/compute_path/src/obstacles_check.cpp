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

  std::cout << "********* obstacles [x0, y0, x1, y1, ...]" << std::endl;
  for(const auto& elem : obstacles_list_){
    std::cout << "[" << elem[0].x << ", " << elem[0].y << ", " << elem[1].x
              << ", " << elem[1].y << ", " << elem[2].x << ", " << elem[2].y
              << ", " << elem[3].x << ", " << elem[3].y << "]" << std::endl;
  }
}

bool HybridAstar::ObstalcesCollision(const AstarNode* const node_p,
                                     const PATH_TYPE& update_set) {
  Collision collision_checker;

  for (int i = update_set.size() - 1; i >= 0;
       i -= detect_collision_point_seg_) {
    AstarNode truck_update_pos = PoseTransform(node_p, update_set[i]);
    collision_checker.obj_1_ = GetTruckFrame(truck_update_pos);

    for (const auto& elem : obstacles_list_) {
      collision_checker.obj_2_ = elem;

      if (collision_checker.IsCollision()) {
        // std::cout << "*************collision********" << std::endl;
        return true;
      }
    }
  }

  return false;
}
