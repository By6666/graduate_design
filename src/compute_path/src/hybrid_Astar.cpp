#include "ros/ros.h"

#include <iostream>

#include "compute_path/execute.h"
#include "matplotlibcpp.h"
#include "common/log.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging("hybrid_A");
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "hybrid_A");
  ros::NodeHandle nh;

  ObsInfo obs_info;
  TruckInfo truck_info;
  RouteInfo route_info;
  HybridAstar hybrid_astar;

  // // subscribe map topic, get grid map info
  // ros::Subscriber map_sub =
  //     nh.subscribe("/map", 1, &ObsInfo::GridMapCall, &obs_info);

  // subscribe hollow topic, get obstacles info
  ros::Subscriber obs_sub =
      nh.subscribe("/obstacles_create/obstacles_info", 1,
                   &ObsInfo::ObstaclesInfoCall, &obs_info);

  // subscribe bounder info
  ros::Subscriber bounder_sub = nh.subscribe(
      "road_line/all_bounder", 1, &RouteInfo::BounderInfoCall, &route_info);

  // subscribe road center line info
  ros::Subscriber ref_line_sub = nh.subscribe(
      "road_line/center_line", 1, &RouteInfo::CentreLineInfoCall, &route_info);

  // subscribe road left line info
  ros::Subscriber left_line_sub =
      nh.subscribe("road_line/left_bounder", 1,
                   &RouteInfo::LeftBounderLineInfoCall, &route_info);

  // subscribe road right line info
  ros::Subscriber right_line_sub =
      nh.subscribe("road_line/right_bounder", 1,
                   &RouteInfo::RightBounderLineInfoCall, &route_info);

  // start prime sub
  ros::Subscriber prime_start_sub = nh.subscribe(
      "road_line/start_pose", 1, &TruckInfo::StartPoseCallback, &truck_info);

  // goal prime sub
  ros::Subscriber prime_goal_sub = nh.subscribe(
      "road_line/goal_pose", 1, &TruckInfo::GoalPoseCallback, &truck_info);

  // pub truck show info
  ros::Publisher truck_show_pub =
      nh.advertise<visualization_msgs::MarkerArray>("truck_show", 1);

  ros::Publisher update_show_pub =
      nh.advertise<visualization_msgs::MarkerArray>("update_pose_show", 1);

  // pub to velocity planning
  ros::Publisher path_pub =
      nh.advertise<nav_msgs::Path>("/path_planning_result", 1, true);
  ros::Publisher flag_pub =
      nh.advertise<geometry_msgs::Point>("/flag", 1, true);

  double time_sum = 0.0;
  int cnt = 0, test_times_cnt;
  bool test_model_flg, curvature_show_flg;
  nh.param<bool>("curvature_show_flg", curvature_show_flg, false);
  nh.param<bool>("test_model_flg", test_model_flg, false);
  nh.param<int>("test_times", test_times_cnt, 20);

  while (ros::ok()) {
    // get all info (1.grid map info, 2.hollow info)
    ros::spinOnce();

    // param update
    truck_info.UpgrateParam();
    hybrid_astar.UpgrateParam();
    if (!(obs_info.get_obs_state() && truck_info.get_start_goal_state() &&
          route_info.get_route_state())) {
      std::cout << "[obs_st, truck_st, route_st]: [" << obs_info.get_obs_state()
                << ", " << truck_info.get_start_goal_state() << ", "
                << route_info.get_route_state() << "]" << std::endl;
      ros::Duration(0.01).sleep();
      continue;
    }

    // input start, goal, cost map , obs info and hollow info to hybrid_astar
    ImportInfo(obs_info, truck_info, route_info, hybrid_astar);

    // hybrid Astar
    std::cout << "********** Hybrid Astar start !! **********" << std::endl;
    // hybrid_astar.PrintMapSize();
    ros::WallTime start = ros::WallTime::now();
    bool flag = hybrid_astar.ExecuteHybridAstar();
    ros::WallTime end = ros::WallTime::now();
    if (flag) {
      --test_times_cnt;
      std::cout << "hybrid_astar execute successful !!" << std::endl;
      PublishFinalPath(hybrid_astar, path_pub);
    }

    // if (test_model_flg && !test_times_cnt) hybrid_astar.PrintPath();
    std::cout << "*********** Hybrid Astar End !! ***********" << std::endl
              << "path_score: " << hybrid_astar.get_path_evaluate_value()
              << "  average_curvature_diff: "
              << hybrid_astar.get_curvature_average_diff()
              << "  max_curvature_diff: "
              << hybrid_astar.get_max_curvature_diff() << std::endl
              << "path_length: " << hybrid_astar.get_path_length()
              << "  extension_nums: " << hybrid_astar.get_extension_pos_nums()
              << "  spend time: " << (end - start).toSec() * 1000 << " ms"
              << std::endl;
    // cost time sum
    time_sum += (end - start).toSec() * 1000;
    std::cout << "average spend time : " << time_sum / (++cnt) << " ms"
              << std::endl
              << std::endl
              << std::endl;

    // show in rviz
    truck_info.TruckShow(hybrid_astar.get_final_path(),
                         hybrid_astar.get_optimize_final_path(),
                         truck_show_pub);
    hybrid_astar.UpdatePoseShow(update_show_pub);

    if (test_model_flg && !test_times_cnt) {
      while (ros::ok()) {
        truck_info.TruckShow(hybrid_astar.get_final_path(),
                             hybrid_astar.get_optimize_final_path(),
                             truck_show_pub);
        ros::Duration(0.1).sleep();
        hybrid_astar.UpdatePoseShow(update_show_pub);
        ros::Duration(0.1).sleep();

        geometry_msgs::Point flag_;
        flag_.x = 1;
        flag_pub.publish(flag_);
      }
      hybrid_astar.PrintMapSize();
      std::cout << std::endl << std::endl;
      break;
    }

    ros::Duration(0.5).sleep();
  }
  if (curvature_show_flg) {
    ros::shutdown();
    matplotlibcpp::plot(hybrid_astar.get_curvature_data());
    matplotlibcpp::grid(true);
    matplotlibcpp::show();
  }
  return 0;
}
