#ifndef TRUCK_INFO_H
#define TRUCK_INFO_H

#include <iostream>
#include <vector>

#include "geometry_msgs/Point.h"
// #include "geometry_msgs/PoseStamped.h"
// #include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

class TruckInfo {
 public:
  TruckInfo();

  void UpgrateParam();
  void TruckShow(const std::vector<geometry_msgs::Pose>& path,
                 const std::vector<geometry_msgs::Pose>& optimize_path,
                 const ros::Publisher& pub);

  void GoalPoseCallback(const geometry_msgs::PoseConstPtr& goal_initial);
  void StartPoseCallback(const geometry_msgs::PoseConstPtr& start_initial);

  inline bool get_start_goal_state() const { return start_ok_ && goal_ok_; }
  inline const geometry_msgs::Pose& get_goal_pose() const {
    return goal_pose_.pose;
  }
  inline const geometry_msgs::Pose& get_start_pose() const {
    return start_pose_.pose;
  }

  inline double get_truck_width() const { return truck_width_; }
  inline double get_truck_length() const { return truck_length_; }
  inline double get_truck_base2back() const { return truck_base2back_; }

 private:
  ros::NodeHandle private_nh_;

  int dist_limit_;
  bool start_ok_, goal_ok_;
  double start_yaw_, goal_yaw_;
  double dist_limit_coff_, truck_vel_, node2goal_r_;
  geometry_msgs::PoseStamped start_pose_, goal_pose_;
  double truck_length_, truck_width_, truck_base2back_;

  std::vector<geometry_msgs::Point> TruckFrame(
      const geometry_msgs::Pose& central);
};

#endif
