#include "road_line/road_create.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "road_line");
  ros::NodeHandle nh("~");

  ros::Publisher left_bounder =
      nh.advertise<nav_msgs::Path>("left_bounder", 1, true);
  ros::Publisher right_bounder =
      nh.advertise<nav_msgs::Path>("right_bounder", 1, true);
  ros::Publisher back_bounder =
      nh.advertise<nav_msgs::Path>("back_bounder", 1, true);
  ros::Publisher forward_bounder =
      nh.advertise<nav_msgs::Path>("forward_bounder", 1, true);

  ros::Publisher all_bounder =
      nh.advertise<nav_msgs::Path>("all_bounder", 1, true);

  ros::Publisher goal_pub =
      nh.advertise<geometry_msgs::Pose>("prime_goal_pose", 1);
  ros::Publisher start_pub =
      nh.advertise<geometry_msgs::Pose>("prime_start_pose", 1);

  RoadLine road;

  while (!road.ReadRoadLine()) {
    ros::Duration(0.5).sleep();
  }

  while (ros::ok()) {
    road.UpgrateParam();
    goal_pub.publish(road.get_goal_pose());
    start_pub.publish(road.get_start_pose());

    left_bounder.publish(road.get_left_bounder());
    right_bounder.publish(road.get_right_bounder());
    back_bounder.publish(road.get_back_bounder());
    forward_bounder.publish(road.get_forward_bounder());
    all_bounder.publish(road.get_all_bounder());

    ros::Duration(0.2).sleep();
  }
  return 0;
}