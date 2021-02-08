#include <ros/ros.h>
#include <tf/tf.h>

#include <iostream>

#include "hollow_create/ObstacleObject.h"
#include "hollow_create/Obstacles.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "hollow_create");
  ros::NodeHandle nh("~");
  ros::Publisher pub = nh.advertise<hollow_create::Obstacles>("obstacles_info", 1);

  double size_x, size_y, place_x, place_y, yaw;

  nh.param<double>("yaw", yaw, 0.0);
  nh.param<double>("size_x", size_x, 10.0);
  nh.param<double>("size_y", size_y, 5.0);
  nh.param<double>("place_y", place_y, 0.0);
  nh.param<double>("place_x", place_x, 12.0);

  hollow_create::Obstacles obstacles;
  hollow_create::ObstacleObject obstacle_object;

  obstacles.header.frame_id = "global";
  obstacles.header.stamp = ros::Time::now();

  obstacle_object.header.frame_id = "global";
  obstacle_object.header.stamp = ros::Time::now();

  obstacle_object.obs_box.x = size_x;
  obstacle_object.obs_box.y = size_y;
  obstacle_object.obs_box.z = 1.0;

  obstacle_object.center.position.x = place_x;
  obstacle_object.center.position.y = place_y;
  obstacle_object.center.position.z = 0.0;

  obstacle_object.center.orientation = tf::createQuaternionMsgFromYaw(yaw);

  obstacle_object.is_static = true;
  obstacle_object.label = 0;


  obstacles.obstacles.push_back(obstacle_object);

  while (ros::ok()) {
    // nh.param<double>("yaw", yaw, 0.0);
    // nh.param<double>("size_x", size_x, 10.0);
    // nh.param<double>("size_y", size_y, 10.0);
    // nh.param<double>("place_y", place_y, 0.0);
    // nh.param<double>("place_x", place_x, 12.0);

    // obstacles.boxes.back().dimensions.x = size_x;
    // obstacles.boxes.back().dimensions.y = size_y;

    // obstacles.boxes.back().pose.position.x = place_x;
    // obstacles.boxes.back().pose.position.y = place_y;

    // obstacles.boxes.back().pose.orientation =
    //     tf::createQuaternionMsgFromYaw(yaw);

    pub.publish(obstacles);
    ros::Duration(0.5).sleep();
  }

  return 0;
}
