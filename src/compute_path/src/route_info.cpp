#include "compute_path/route_info.h"

#include "tf/tf.h"

RouteInfo::RouteInfo() : bounder_ok_(false), ref_line_ok_(false) {}

void RouteInfo::BounderInfoCall(const nav_msgs::PathConstPtr& msg) {
  std::vector<geometry_msgs::PoseStamped> buff = msg->poses;
  bounder_points_.clear();
  bounder_points_.reserve(buff.size());
  for (const auto& elem : buff) {
    bounder_points_.push_back(
        std::vector<double>{elem.pose.position.x, elem.pose.position.y});
  }
  bounder_ok_ = true;
}

void RouteInfo::CentreLineInfoCall(const nav_msgs::PathConstPtr& msg) {
  geometry_msgs::Point temp;

  for (const auto& elem : msg->poses) {
    temp.x = elem.pose.position.x;
    temp.y = elem.pose.position.y;

    // use z to stage yaw
    temp.z = tf::getYaw(elem.pose.orientation);

    ref_line_.push_back(temp);
  }

  ref_line_ok_ = true;
}
