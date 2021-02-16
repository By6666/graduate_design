#ifndef EXECUTE_H
#define EXECUTE_H

#include "compute_path/compute_path.h"
#include "compute_path/obstacle_info.h"
#include "compute_path/truck_info.h"
#include "compute_path/route_info.h"

inline void ImportInfo(const ObsInfo& obs_info, const TruckInfo& truck_info,
                       const RouteInfo& route_info, HybridAstar& hybrid_astar) {
  hybrid_astar.set_obstacles_info() = obs_info.get_obstacles_info();

  hybrid_astar.set_bounder_info() = route_info.get_bounder_info();
  hybrid_astar.set_ref_line() = route_info.get_ref_line();
  hybrid_astar.set_left_boundary() = route_info.get_left_boundary();
  hybrid_astar.set_right_boundary() = route_info.get_right_boundary();

  hybrid_astar.set_goal_pose() = truck_info.get_goal_pose();
  hybrid_astar.set_start_pose() = truck_info.get_start_pose();
  hybrid_astar.set_truck_width() = truck_info.get_truck_width();
  hybrid_astar.set_truck_length() = truck_info.get_truck_length();
  hybrid_astar.set_truck_base2back() = truck_info.get_truck_base2back();
}

inline void PublishFinalPath(const HybridAstar& hybrid_astar,
                             ros::Publisher& pub) {
  nav_msgs::Path final_path;
  final_path.header.frame_id = "global";
  final_path.header.stamp = ros::Time::now();

  if (hybrid_astar.get_optimize_final_path().size() > 0) {
    for (auto& elem : hybrid_astar.get_optimize_final_path()) {
      geometry_msgs::PoseStamped temp;
      temp.pose = elem;
      final_path.poses.push_back(temp);
    }
  } else {
    for (auto& elem : hybrid_astar.get_final_path()) {
      geometry_msgs::PoseStamped temp;
      temp.pose = elem;
      final_path.poses.push_back(temp);
    }
  }

  pub.publish(final_path);
}

#endif
