#ifndef TRANS_OBS_MSG_H
#define TRANS_OBS_MSG_H

#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <iostream>

#include "hollow_create/ObstacleObject.h"
#include "hollow_create/Obstacles.h"

typedef jsk_recognition_msgs::BoundingBoxArray ObsDisplay;

class TransObsMsg {
 public:
  TransObsMsg() = default;

  void ObstaclesInfoCall(const hollow_create::ObstaclesConstPtr& msg);

  inline const ObsDisplay& GetDisplayMsg() const { return obs_to_display; }

 private:
  ObsDisplay obs_to_display;

  void TransObsMsgFunction(const hollow_create::ObstaclesConstPtr& msg);
};

void TransObsMsg::ObstaclesInfoCall(
    const hollow_create::ObstaclesConstPtr& msg) {
  if (obs_to_display.boxes.size() != msg->obstacles.size()) {
    TransObsMsgFunction(msg);
  }
}

void TransObsMsg::TransObsMsgFunction(
    const hollow_create::ObstaclesConstPtr& msg) {
  obs_to_display.boxes.clear();

  obs_to_display.header.frame_id = "global";
  obs_to_display.header.stamp = ros::Time::now();

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

    obs_to_display.boxes.push_back(box_temp);
  }
}

#endif
