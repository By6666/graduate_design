#include "compute_path/compute_path.h"

void HybridAstar::CreateKDTreeBounderInfo() {
  ros::WallTime start = ros::WallTime::now();

  KDTreeSP::pointVec temp_vec;
  double yaw = -tf::getYaw(prime_start_pose_.orientation);

  double prime_x = prime_start_pose_.position.x;
  double prime_y = prime_start_pose_.position.y;

  for (const auto& elem : bounder_info_) {
    double temp_x = elem.front() - prime_x;
    double temp_y = elem.back() - prime_y;

    temp_vec.push_back(
        KDTreeSP::point_t{temp_x * cos(yaw) - temp_y * sin(yaw),
                          temp_x * sin(yaw) + temp_y * cos(yaw)});
  }

  kdtree_bounder_ = KDTreeSP::KDTree(temp_vec);
  ros::WallTime end = ros::WallTime::now();

  std::cout << "KDTree create *** size:" << temp_vec.size()
            << "  time:" << (end - start).toSec() * 1000 << " ms" << std::endl;

  std::cout << "***********refline: [x, y], size:" << ref_line_.size()
            << std::endl;

  for (auto& elem : ref_line_) {
    double temp_x = elem.x - prime_x;
    double temp_y = elem.y - prime_y;

    elem.x = temp_x * cos(yaw) - temp_y * sin(yaw);
    elem.y = temp_x * sin(yaw) + temp_y * cos(yaw);

    std::cout << elem.x << ", " << elem.y << std::endl;
  }

  std::cout << "***********left_boundary: [x, y], size:"
            << left_boundary_.size() << std::endl;
  for (auto& elem : left_boundary_) {
    double temp_x = elem.x - prime_x;
    double temp_y = elem.y - prime_y;

    elem.x = temp_x * cos(yaw) - temp_y * sin(yaw);
    elem.y = temp_x * sin(yaw) + temp_y * cos(yaw);

    std::cout << elem.x << ", " << elem.y << std::endl;
  }

  std::cout << "***********right_boundary: [x, y], size:"
            << right_boundary_.size() << std::endl;
  for (auto& elem : right_boundary_) {
    double temp_x = elem.x - prime_x;
    double temp_y = elem.y - prime_y;

    elem.x = temp_x * cos(yaw) - temp_y * sin(yaw);
    elem.y = temp_x * sin(yaw) + temp_y * cos(yaw);

    std::cout << elem.x << ", " << elem.y << std::endl;
  }
}

void HybridAstar::CalculateStartAndGoalPoint() {
  // set start pose
  start_pose_.position.x = 0.0;
  start_pose_.position.y = 0.0;

  start_pose_.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // get goal pose in start pose frame
  double temp_x = prime_goal_pose_.position.x - prime_start_pose_.position.x;
  double temp_y = prime_goal_pose_.position.y - prime_start_pose_.position.y;
  double temp_yaw = tf::getYaw(prime_goal_pose_.orientation);

  //** 坐标相对位置不变，坐标系旋转，所以yaw取负
  double yaw = -tf::getYaw(prime_start_pose_.orientation);

  goal_pose_.position.x = temp_x * cos(yaw) - temp_y * sin(yaw);
  goal_pose_.position.y = temp_x * sin(yaw) + temp_y * cos(yaw);
  goal_pose_.orientation = tf::createQuaternionMsgFromYaw(temp_yaw + yaw);

  // std::cout << "prime_start_pose[x, y, yaw] : [" <<
  // prime_start_pose_.position.x
  //           << ", " << prime_start_pose_.position.y << ", "
  //           << tf::getYaw(prime_start_pose_.orientation) << "]" << std::endl;

  // std::cout << "prime_goal_pose[x, y, yaw] : [" <<
  // prime_goal_pose_.position.x
  //           << ", " << prime_goal_pose_.position.y << ", "
  //           << tf::getYaw(prime_goal_pose_.orientation) << "]" << std::endl;

  // std::cout << "start_pose[x, y, yaw] : [" << start_pose_.position.x << ", "
  //           << start_pose_.position.y << ", "
  //           << tf::getYaw(start_pose_.orientation) << "]" << std::endl;

  std::cout << "goal_pose[x, y, yaw] : [" << goal_pose_.position.x << ", "
            << goal_pose_.position.y << ", "
            << tf::getYaw(goal_pose_.orientation) << "]" << std::endl;

  goal_pose_id_ = CalculateID(goal_pose_.position.x, goal_pose_.position.y,
                              tf::getYaw(goal_pose_.orientation));
  start_pose_id_ = CalculateID(start_pose_.position.x, start_pose_.position.y,
                               tf::getYaw(start_pose_.orientation));
  double start_h =
      CalculateDisTwoPoint(start_pose_.position, goal_pose_.position);
  node_stg_[goal_pose_id_] = AstarNode(goal_pose_, DBL_MAX, 0.0);
  node_stg_[start_pose_id_] = AstarNode(start_pose_, 0.0, start_h);

  goal_info_ = &node_stg_[goal_pose_id_];
  start_info_ = &node_stg_[start_pose_id_];
  // std::cout << "goal yaw = " << goal_info_->yaw << std::endl;

  // start point push openlist
  openlist_.emplace(start_pose_id_, KeyValue(start_info_->h, 0.0));
  start_info_->state = STATE::OPEN;
}

void HybridAstar::ConvertPathFrame() {
  // convert final path
  for (auto& elem : final_path_) {
    elem = PoseTransform(prime_start_pose_, elem);
  }

  // convert prime path
  for (auto& elem : prime_path_temp_) {
    elem = PoseTransform(prime_start_pose_, elem);
  }

  // convert optimize path
  for (auto& elem : optimize_final_path_) {
    elem = PoseTransform(prime_start_pose_, elem);
  }
}
