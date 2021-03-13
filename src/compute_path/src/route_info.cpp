#include "compute_path/route_info.h"

#include "tf/tf.h"

RouteInfo::RouteInfo()
    : bounder_ok_(false),
      ref_line_ok_(false),
      left_boundary_ok_(false),
      right_boundary_ok_(false),
      eco_ref_ok_(false) {}

void RouteInfo::BounderInfoCall(const nav_msgs::PathConstPtr& msg) {
  std::vector<geometry_msgs::PoseStamped> buff = msg->poses;

  if (!bounder_ok_) {
    bounder_points_.clear();
    bounder_points_.reserve(buff.size());
    for (const auto& elem : buff) {
      bounder_points_.push_back(
          std::vector<double>{elem.pose.position.x, elem.pose.position.y});
    }
    bounder_ok_ = true;
  }
}

void RouteInfo::CentreLineInfoCall(const nav_msgs::PathConstPtr& msg) {
  geometry_msgs::Point temp;

  if (!ref_line_ok_) {
    ref_line_.clear();

    for (const auto& elem : msg->poses) {
      temp.x = elem.pose.position.x;
      temp.y = elem.pose.position.y;

      // use z to stage yaw
      temp.z = tf::getYaw(elem.pose.orientation);

      ref_line_.push_back(temp);
    }

    ref_line_ok_ = true;
  }
}

void RouteInfo::LeftBounderLineInfoCall(const nav_msgs::PathConstPtr& msg) {
  if (!left_boundary_ok_) {
    geometry_msgs::Point temp;

    left_boundary_.clear();

    for (const auto& elem : msg->poses) {
      temp.x = elem.pose.position.x;
      temp.y = elem.pose.position.y;

      // use z to stage yaw
      temp.z = tf::getYaw(elem.pose.orientation);

      left_boundary_.push_back(temp);
    }

    left_boundary_ok_ = true;
  }
}

void RouteInfo::RightBounderLineInfoCall(const nav_msgs::PathConstPtr& msg) {
  if (!right_boundary_ok_) {
    geometry_msgs::Point temp;

    right_boundary_.clear();

    for (const auto& elem : msg->poses) {
      temp.x = elem.pose.position.x;
      temp.y = elem.pose.position.y;

      // use z to stage yaw
      temp.z = tf::getYaw(elem.pose.orientation);

      right_boundary_.push_back(temp);
    }

    right_boundary_ok_ = true;
  }
}

void RouteInfo::EcoReferencesPathCall(
    const eco_references::EcoReferencesConstPtr& msg) {
  if (!eco_ref_ok_) {
    eco_ref_ = *msg;
    eco_ref_ok_ = true;
  }
}
