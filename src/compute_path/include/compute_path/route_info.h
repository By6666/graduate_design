#ifndef ROUTE_INFO_H
#define ROUTE_INFO_H

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"

#include <vector>

class RouteInfo {
 public:
  RouteInfo();

  void BounderInfoCall(const nav_msgs::PathConstPtr& msg);
  void CentreLineInfoCall(const nav_msgs::PathConstPtr& msg);

  inline const std::vector<std::vector<double>>& get_bounder_info() const {
    return bounder_points_;
  }

  inline const std::vector<geometry_msgs::Point>& get_ref_line() const {
    return ref_line_;
  }

  inline bool get_route_state() { return bounder_ok_ && ref_line_ok_; }

 private:
  bool bounder_ok_, ref_line_ok_;

  std::vector<std::vector<double>> bounder_points_;
  std::vector<geometry_msgs::Point> ref_line_;
};

#endif
