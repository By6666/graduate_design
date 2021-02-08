#include "compute_path/obstacle_info.h"

ObsInfo::ObsInfo() : obstacles_ok_(false) {
  // ros::NodeHandle private_nh("~ObsInfo");

  // 待添加参数
  // private_nh.param<int>("obstacle_threshold", obstacle_threshold_, 80);
}

void ObsInfo::ObstaclesInfoCall(const hollow_create::ObstaclesConstPtr& msg) {
  obstacles_info = *msg;
  obstacles_ok_ = true;
}
