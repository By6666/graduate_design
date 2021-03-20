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
  ros::Publisher center_line =
      nh.advertise<nav_msgs::Path>("center_line", 1, true);

  ros::Publisher all_bounder =
      nh.advertise<nav_msgs::Path>("all_bounder", 1, true);

  ros::Publisher goal_pub =
      nh.advertise<geometry_msgs::Pose>("goal_pose", 1, true);
  ros::Publisher start_pub =
      nh.advertise<geometry_msgs::Pose>("start_pose", 1, true);


  ros::Publisher left_bounder_2 =
      nh.advertise<nav_msgs::Path>("left_bounder_2", 1, true);
  ros::Publisher right_bounder_2 =
      nh.advertise<nav_msgs::Path>("right_bounder_2", 1, true);
  ros::Publisher left_bounder_3 =
      nh.advertise<nav_msgs::Path>("left_bounder_3", 1, true);
  ros::Publisher right_bounder_3 =
      nh.advertise<nav_msgs::Path>("right_bounder_3", 1, true);

  RoadLine road;

  // subscribe start pose
  ros::Subscriber start_sub = nh.subscribe(
      "/initialpose", 1, &RoadLine::StartInfoCallback, &road);
  // subscribe goal pose
  ros::Subscriber goal_sub = nh.subscribe(
      "/move_base_simple/goal", 1, &RoadLine::GoalInfoCallback, &road);



  while (!road.ReadRoadLine()) {
    ros::Duration(0.5).sleep();
  }

  while (ros::ok()) {
    ros::spinOnce();
    
    road.UpgrateParam();

    goal_pub.publish(road.get_goal_pose());
    start_pub.publish(road.get_start_pose());

    left_bounder.publish(road.get_left_bounder());
    right_bounder.publish(road.get_right_bounder());
    back_bounder.publish(road.get_back_bounder());
    forward_bounder.publish(road.get_forward_bounder());
    center_line.publish(road.get_center_line());
    all_bounder.publish(road.get_all_bounder());

    left_bounder_2.publish(road.get_left_bounder_2());
    right_bounder_2.publish(road.get_right_bounder_2());

    left_bounder_3.publish(road.get_left_bounder_3());
    right_bounder_3.publish(road.get_right_bounder_3());

    ros::Duration(0.2).sleep();
  }
  return 0;
}
