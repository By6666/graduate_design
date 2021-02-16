#ifndef OBSTACLE_INFO_H
#define OBSTACLE_INFO_H

#include <iostream>
#include <set>
#include <vector>

#include "compute_path/state.h"
#include "ros/ros.h"

#include "hollow_create/Obstacles.h"

class ObsInfo {
 public:
  ObsInfo();

  void ObstaclesInfoCall(const hollow_create::ObstaclesConstPtr& msg);

  inline bool get_obs_state() const {
    return obstacles_ok_;
  }

  inline const _Type_Obs& get_obstacles_info() const { return obstacles_info; }

 private:
  bool obstacles_ok_;
  _Type_Obs obstacles_info;
};

#endif
