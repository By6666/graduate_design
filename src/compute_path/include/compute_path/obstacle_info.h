#ifndef OBSTACLE_INFO_H
#define OBSTACLE_INFO_H

#include <iostream>
#include <set>
#include <vector>

#include "compute_path/state.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "std_msgs/Header.h"

class ObsInfo {
 public:
  ObsInfo();

  void GridMapCall(const nav_msgs::OccupancyGridConstPtr& msg);
  void HollowInfoCall(
      const jsk_recognition_msgs::BoundingBoxArrayConstPtr& msg);
  void BounderInfoCall(const nav_msgs::PathConstPtr& msg);

  void MapHandel(const std::vector<int8_t>& map_data);

  inline bool get_obs_state() const { return obs_list_ok_ && bounder_ok_; }
  inline int get_map_width() const { return map.info.width; }
  inline int get_map_heigh() const { return map.info.height; }
  inline float get_map_resolution() const { return map.info.resolution; }
  inline int get_obstacle_threshold() const { return obstacle_threshold_; }

  inline std::set<int> get_obs_list() const { return obs_list_; }
  inline _Type_Hollow get_hollow_info() const { return hollow_info_; }
  inline std_msgs::Header get_map_header() const { return map.header; }
  inline geometry_msgs::Pose get_map_origin() const { return map.info.origin; }

  inline std::vector<int8_t> get_map_cost() const { return map.data; }
  inline const std::vector<std::vector<double> >& get_bounder_info() const {
    return bounder_points_;
  }

 private:
  bool hollow_ok_;
  bool obs_list_ok_;
  bool bounder_ok_;
  std::set<int> obs_list_;
  int obstacle_threshold_;
  _Type_Hollow hollow_info_;
  nav_msgs::OccupancyGrid map;
  std::vector<std::vector<double> > bounder_points_;
};

#endif