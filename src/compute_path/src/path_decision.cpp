#include "compute_path/compute_path.h"

// path decision: left, right, ignore, vehicle place base obstacle
void HybridAstar::PathDecisionProcess() {
  for (int i = 0; i < obstacles_info_.obstacles.size(); ++i) {
    if (obstacles_info_.obstacles[i].is_static) {
      auto& obs = obstacles_info_.obstacles[i];

      // get ceneter pose in start pose frame
      double temp_x = obs.center.x - prime_start_pose_.position.x;
      double temp_y = obs.center.y - prime_start_pose_.position.y;
      double temp_yaw = obs.center.z;

      //** 坐标相对位置不变，坐标系旋转，所以yaw取负
      double yaw = -tf::getYaw(prime_start_pose_.orientation);

      // obstacle central place base start pose
      double planning_center_x = temp_x * cos(yaw) - temp_y * sin(yaw);
      double planning_center_y = temp_x * sin(yaw) + temp_y * cos(yaw);
      double planning_center_z = NormalizeAngle(temp_yaw + yaw);

      // ignore decision
      if (planning_center_x < 0.0 &&
          std::hypot(temp_x, temp_y) > (truck_length_ + obs.obs_box.x)) {
        obs.path_decision = "ignore";
        std::cout << "obstacle label: " << obs.label
                  << "  decision: " << obs.path_decision << std::endl;
        continue;
      }

      if (planning_center_x > goal_pose_.position.x &&
          std::hypot(obs.center.x - prime_goal_pose_.position.x,
                     obs.center.y - prime_goal_pose_.position.y) >
              (truck_length_ + obs.obs_box.x)) {
        obs.path_decision = "ignore";
        std::cout << "obstacle label: " << obs.label
                  << "  decision: " << obs.path_decision << std::endl;
        continue;
      }

      auto cmp = [](const geometry_msgs::Pose& point, double number) {
        return point.position.x < number;
      };
      auto it_lower = std::lower_bound(final_path_.begin(), final_path_.end(),
                                       planning_center_x, cmp);

      double path_y_on_obs_center_x = it_lower->position.y;

      if (path_y_on_obs_center_x > planning_center_y) {
        obs.path_decision = "left";
      } else {
        obs.path_decision = "right";
      }

      std::cout << "obstacle label: " << obs.label
                << "  decision: " << obs.path_decision << std::endl;
    }
  }
}
