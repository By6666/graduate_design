#include "hollow_create/trans_obs_msg.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacles_display");
  ros::NodeHandle nh("~");

  TransObsMsg obs_trans;

  // subscribe obstacles topic, get obstacles info
  ros::Subscriber obs_sub =
      nh.subscribe("/obstacles_create/obstacles_info", 10,
                   &TransObsMsg::ObstaclesInfoCall, &obs_trans);

  ros::Publisher pub = nh.advertise<ObsDisplay>("obstacles", 1);
  ros::Publisher pub_obs_path_box = nh.advertise<ObsDisplay>("obs_path_box", 1);

  ros::Publisher pub_obs_trajectorys =
      nh.advertise<visualization_msgs::MarkerArray>("obs_trajectorys", 1, true);

  while (ros::ok()) {
    ros::spinOnce();

    pub.publish(obs_trans.GetDisplayMsg());
    pub_obs_trajectorys.publish(obs_trans.GetObsPath());
    pub_obs_path_box.publish(obs_trans.GetObsPathBoxMsg());
    ros::Duration(2.0).sleep();
  }

  return 0;
}
