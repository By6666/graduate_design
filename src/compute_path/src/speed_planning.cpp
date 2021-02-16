#include "compute_path/compute_path.h"

#include "qp_spline_speed/qp_spline_st_graph.h"

bool HybridAstar::SpeedPlanning() {
  std::cout << "************* speed planning *************" << std::endl;

  // using mlp to decision
  SpeedDecisionProcess();

  // get ST obstacles box
  std::vector<std::vector<std::array<double, 3>>> st_obs_boxes;
  CalculateObsSTBox(&st_obs_boxes);

  CommonPoint init_point(0.0, 0.0, 15.0, 0.0);
  math::common::Limits s_limits;

  // get s limits
  for (int i = 0; i < 80; ++i) {
    double curr_t = i * 0.1;
    double s_low = 0.0, s_up = 250.0;

    int dynamic_obs_cnt = 0;
    for (int j = 0; j < obstacles_info_.obstacles.size(); ++j) {
      const auto& obstacle = obstacles_info_.obstacles[j];
      if (obstacle.is_static) {
        continue;
      }

      const auto& st_obs_box = st_obs_boxes[dynamic_obs_cnt++];

      if (curr_t > st_obs_box.front()[0] && curr_t < st_obs_box.back()[0]) {
        const auto cmp = [](const std::array<double, 3>& elem, double x) {
          return elem[0] < x;
        };
        auto it_lower =
            std::lower_bound(st_obs_box.begin(), st_obs_box.end(), curr_t, cmp);

        if (obstacle.speed_decision == "yield") {
          s_up = std::min(s_up, it_lower->at(1));
        } else if (obstacle.speed_decision == "overtake") {
          s_low = std::max(s_low, it_lower->at(2));
        }
      }
    }

    s_limits.AppendLimit(curr_t, s_low, s_up);
  }

  std::cout << "s limits ********  [s, l, u]" << std::endl;
  for (const auto& elem : s_limits.limit_points()) {
    std::cout << elem.x() << ", " << elem.l() << ", " << elem.u() << std::endl;
  }

  math::common::Limits v_limits;
  // get v limits
  for (int i = 0; i < 80; ++i) {
    double curr_t = i * 0.1;
    v_limits.AppendLimit(curr_t, 0.0, 16.67);
  }

  // get a limits
  const std::pair<double, double>& a_limits{-4.0, 4.0};

  // get s refs
  math::common::References ref_s;
  for (int i = 0; i < 80; ++i) {
    double curr_t = i * 0.1;
    ref_s.AppendReference(curr_t, 250.0);
  }

  // get v refs
  math::common::References ref_v;
  for (int i = 0; i < 80; ++i) {
    double curr_t = i * 0.1;
    ref_v.AppendReference(curr_t, 12.0);
  }

  math::qp_spline::QpSplineStGraph qp_solve;

  qp_solve.SetCondition(init_point, s_limits, v_limits, a_limits, ref_s, ref_v);

  std::cout << "get all condition successful !!" << std::endl;

  math::qp_spline::SpeedData speed_opt;
  qp_solve.Solve(&speed_opt);
  speed_opt.DebugString();

  return true;
}

void HybridAstar::CalculateObsSTBox(
    std::vector<std::vector<std::array<double, 3>>>* const st_obs_boxes) {
  // get obstacle box in ST frame
  const auto planning_path = optimize_final_path_;

  for (const auto& obstacle : obstacles_info_.obstacles) {
    // std::cout << "current obstacle label: " << obstacle.label << std::endl;
    if (obstacle.is_static) {
      continue;
    }

    geometry_msgs::Point temp_center;

    // get ceneter pose in start pose frame
    double temp_x = obstacle.center.x - prime_start_pose_.position.x;
    double temp_y = obstacle.center.y - prime_start_pose_.position.y;
    double temp_yaw = obstacle.center.z;

    //** 坐标相对位置不变，坐标系旋转，所以yaw取负
    double yaw = -tf::getYaw(prime_start_pose_.orientation);

    temp_center.x = temp_x * cos(yaw) - temp_y * sin(yaw);
    temp_center.y = temp_x * sin(yaw) + temp_y * cos(yaw);
    temp_center.z = NormalizeAngle(temp_yaw + yaw);

    const auto& obs_path = obstacle.path;

    std::vector<std::array<double, 3>> st_obs_box;

    for (int i = 0; i < obs_path.size(); ++i) {
      double curr_t = i * obs_path_duration_;

      geometry_msgs::Point temp_point;
      temp_point.x = obs_path[i].x * cos(temp_center.z) -
                     obs_path[i].y * sin(temp_center.z) + temp_center.x;
      temp_point.y = obs_path[i].x * sin(temp_center.z) +
                     obs_path[i].y * cos(temp_center.z) + temp_center.y;
      temp_point.z = NormalizeAngle(obs_path[i].z + temp_center.z);

      Collision collision_checker;
      collision_checker.obj_1_ = GetObstacleFrame(temp_point, obstacle.obs_box);

      double s = 0.0;
      double low_s = 0.0, up_s = 0.0;
      bool low_s_flg = false, up_s_flg = false;

      std::vector<double> s_vec(planning_path.size());
      s_vec[0] = 0.0;
      for (int j = 1; j < planning_path.size(); ++j) {
        s_vec[j] =
            s_vec[j - 1] +
            std::hypot(
                planning_path[j].position.x - planning_path[j - 1].position.x,
                planning_path[j].position.y - planning_path[j - 1].position.y);
      }

      int j = 0;
      while (j < planning_path.size()) {
        s = s_vec[j];

        collision_checker.obj_2_ = GetTruckFrame(planning_path[j]);

        if (collision_checker.IsCollision()) {
          low_s = s;
          low_s_flg = true;
          break;
        }
        ++j;
      }

      int k = planning_path.size() - 1;

      while (k > j) {
        s = s_vec[k];
        collision_checker.obj_2_ = GetTruckFrame(planning_path[k]);
        if (collision_checker.IsCollision()) {
          up_s = s;
          up_s_flg = true;
          break;
        }
        --k;
      }

      if (low_s_flg && up_s_flg) {
        st_obs_box.push_back(std::array<double, 3>{curr_t, low_s, up_s});
      }
      // std::cout << curr_t << ", " << low_s << ", " << up_s << std::endl;
    }

    st_obs_boxes->push_back(st_obs_box);
  }
}
