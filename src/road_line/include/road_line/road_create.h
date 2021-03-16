#ifndef ROAD_CREATE_H
#define ROAD_CREATE_H

#include <iostream>
#include <string>
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "tf/tf.h"

class RoadLine {
 public:
  RoadLine();

  void StartInfoCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& start_initial);
  void GoalInfoCallback(const geometry_msgs::PoseStampedConstPtr& goal_initial);

  bool ReadRoadLine();
  void CreateRoadBounder();
  void UpgrateParam();

  inline const nav_msgs::Path& get_left_bounder() const {
    return road_left_bounder_;
  }
  inline const nav_msgs::Path& get_right_bounder() const {
    return road_right_bounder_;
  }
  inline const nav_msgs::Path& get_back_bounder() const {
    return road_back_bounder_;
  }
  inline const nav_msgs::Path& get_forward_bounder() const {
    return road_forward_bounder_;
  }
  inline const nav_msgs::Path& get_center_line() const { return road_center_line_; }
  
  inline const nav_msgs::Path& get_all_bounder() const { return all_bounder_; }

  inline const geometry_msgs::Pose& get_goal_pose() const { return goal_pose_; }
  inline const geometry_msgs::Pose& get_start_pose() const {
    return start_pose_;
  }

  inline const nav_msgs::Path& get_left_bounder_2() const {
    return road_left_bounder_2_;
  }
  inline const nav_msgs::Path& get_right_bounder_2() const {
    return road_right_bounder_2_;
  }
  inline const nav_msgs::Path& get_left_bounder_3() const {
    return road_left_bounder_3_;
  }
  inline const nav_msgs::Path& get_right_bounder_3() const {
    return road_right_bounder_3_;
  }

 private:
  struct RoadXY {
    double x;
    double y;
    double yaw;

    RoadXY(double x_, double y_, double yaw_) : x(x_), y(y_), yaw(yaw_){};
  };

  double start_to_goal_dis_;         // 设置起点到终点的距离，单位m
  double road_line_simple_seg_dis_;  // 道路边界点采样间隔，单位m
  double road_left_width_, road_right_width_, back_forward_reselution_;
  std::string file_name_;
  std::vector<RoadXY> prime_road_;
  std::vector<RoadXY> origin_bounder_;
  std::vector<RoadXY> origin_bounder_2_;
  std::vector<RoadXY> origin_bounder_3_;
  geometry_msgs::Pose start_pose_, goal_pose_;
  nav_msgs::Path all_bounder_;
  nav_msgs::Path road_left_bounder_, road_right_bounder_;
  nav_msgs::Path road_back_bounder_, road_forward_bounder_;
  nav_msgs::Path road_center_line_;

  nav_msgs::Path road_left_bounder_2_,road_right_bounder_2_;
  nav_msgs::Path road_left_bounder_3_,road_right_bounder_3_;

  void CreateOriginBounder();
  geometry_msgs::Pose PoseTransform(const RoadXY& central, const RoadXY& pose);

};

inline double DegreeToRad(const double& deg) { return deg * M_PI / 180.0; }
inline double ReserveTwoDecimal(double n) {
  return std::round(n * 100.0) / 100.0;
}

#endif
