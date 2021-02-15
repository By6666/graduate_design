#ifndef TRANS_OBS_MSG_H
#define TRANS_OBS_MSG_H

#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include "visualization_msgs/MarkerArray.h"

#include <iostream>

#include "hollow_create/ObstacleObject.h"
#include "hollow_create/Obstacles.h"

typedef jsk_recognition_msgs::BoundingBoxArray ObsDisplay;

class TransObsMsg {
 public:
  TransObsMsg() {
    ros::NodeHandle private_nh("~");
    private_nh.param<int>("obs_path_box_seg", obs_path_box_seg_, 7);
  }

  void ObstaclesInfoCall(const hollow_create::ObstaclesConstPtr& msg);

  inline const ObsDisplay& GetDisplayMsg() const { return obs_to_display_; }
  inline const ObsDisplay& GetObsPathBoxMsg() const { return obs_path_box_; }
  inline const visualization_msgs::MarkerArray GetObsPath() const {
    return obstacles_trajectory_;
  }

 private:
  int obs_path_box_seg_;

  ObsDisplay obs_to_display_;
  visualization_msgs::MarkerArray obstacles_trajectory_;
  ObsDisplay obs_path_box_;

  void TransObsMsgFunction(const hollow_create::ObstaclesConstPtr& msg);
  void ReleaseObstaclesTrajectory(const hollow_create::ObstaclesConstPtr& msg);

  // pose transform
  geometry_msgs::Point PoseTransform(const geometry_msgs::Point& center,
                                     const geometry_msgs::Point& point) {
    double yaw = center.z;

    geometry_msgs::Point temp_point;
    temp_point.x = point.x * cos(yaw) - point.y * sin(yaw) + center.x;
    temp_point.y = point.x * sin(yaw) + point.y * cos(yaw) + center.y;
    temp_point.z = 0.0;

    return temp_point;
  }
};

void TransObsMsg::ObstaclesInfoCall(
    const hollow_create::ObstaclesConstPtr& msg) {
  if (obs_to_display_.boxes.size() != msg->obstacles.size()) {
    TransObsMsgFunction(msg);
    ReleaseObstaclesTrajectory(msg);
  }
}

void TransObsMsg::TransObsMsgFunction(
    const hollow_create::ObstaclesConstPtr& msg) {
  obs_to_display_.boxes.clear();

  obs_to_display_.header.frame_id = "global";
  obs_to_display_.header.stamp = ros::Time::now();

  jsk_recognition_msgs::BoundingBox box_temp;

  for (const auto& obstacle : msg->obstacles) {
    box_temp.header.frame_id = "global";
    box_temp.header.stamp = ros::Time::now();

    box_temp.label = obstacle.label;
    box_temp.dimensions.x = obstacle.obs_box.x;
    box_temp.dimensions.y = obstacle.obs_box.y;
    box_temp.dimensions.z = 1.0;

    box_temp.pose.position.x = obstacle.center.x;
    box_temp.pose.position.y = obstacle.center.y;
    box_temp.pose.position.z = 0.0;
    box_temp.pose.orientation =
        tf::createQuaternionMsgFromYaw(obstacle.center.z);

    obs_to_display_.boxes.push_back(box_temp);
  }
}

void TransObsMsg::ReleaseObstaclesTrajectory(
    const hollow_create::ObstaclesConstPtr& msg) {
  int id_init = 0;

  // clear display
  visualization_msgs::Marker clear_;
  clear_.action = visualization_msgs::Marker::DELETEALL;
  obstacles_trajectory_.markers.push_back(clear_);

  obs_path_box_.boxes.clear();
  obs_path_box_.header.frame_id = "global";
  obs_path_box_.header.stamp = ros::Time::now();

  // obstacles trajectory
  for (const auto& obstacle : msg->obstacles) {
    if (obstacle.is_static) {
      continue;
    }
    visualization_msgs::Marker obs_trajectory;
    obs_trajectory.header.frame_id = "global";

    obs_trajectory.ns =
        std::to_string(obstacle.label) + "_obstacles_trajectory_";
    obs_trajectory.id = id_init++;

    obs_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    obs_trajectory.action = visualization_msgs::Marker::ADD;

    obs_trajectory.scale.x = 0.1;

    obs_trajectory.color.r = 0.0f;
    obs_trajectory.color.g = 1.0f;
    obs_trajectory.color.b = 0.0f;
    obs_trajectory.color.a = 1.0f;

    jsk_recognition_msgs::BoundingBox box_temp;
    box_temp.header.frame_id = "global";
    box_temp.header.stamp = ros::Time::now();

    box_temp.label = obstacle.label;
    box_temp.dimensions.x = obstacle.obs_box.x;
    box_temp.dimensions.y = obstacle.obs_box.y;
    box_temp.dimensions.z = 1.0;

    int cnt = 0;

    for (const auto& point : obstacle.path) {
      auto temp_point = PoseTransform(obstacle.center, point);
      obs_trajectory.points.push_back(temp_point);

      if (obstacle.label == 1 && cnt == obs_path_box_seg_) {
        box_temp.pose.position.x = temp_point.x;
        box_temp.pose.position.y = temp_point.y;
        box_temp.pose.position.z = 0.0;
        box_temp.pose.orientation =
            tf::createQuaternionMsgFromYaw(obstacle.center.z + point.z);

        obs_path_box_.boxes.push_back(box_temp);

        cnt = 0;
      }

      ++cnt;
    }

    obstacles_trajectory_.markers.push_back(obs_trajectory);
  }
}

#endif
