#include "road_line/road_create.h"

RoadLine::RoadLine() {
  ros::NodeHandle private_nh("~");

  private_nh.param<std::string>("file_name", file_name_, "");
  private_nh.param<double>("road_left_width", road_left_width_, 3.0);
  private_nh.param<double>("road_right_width", road_right_width_, 3.0);
  private_nh.param<double>("back_forward_reselution", back_forward_reselution_,
                           0.1);
  private_nh.param<double>("start_to_goal_dis", start_to_goal_dis_, 300.0);
  private_nh.param<double>("road_line_simple_seg_dis",
                           road_line_simple_seg_dis_, 2.0);

  CreateOriginBounder();

  road_left_bounder_.header.frame_id = "global";
  road_right_bounder_.header.frame_id = "global";
  road_back_bounder_.header.frame_id = "global";
  road_forward_bounder_.header.frame_id = "global";
  road_center_line_.header.frame_id = "global";

  road_left_bounder_2_.header.frame_id = "global";
  road_right_bounder_2_.header.frame_id = "global";
}

/**
 * 创建一份ref_line左右扩边的原始点
 * 其中front为left bounder, back为right bounder
 * */
void RoadLine::CreateOriginBounder() {
  int point_nums = std::ceil((road_left_width_ + road_right_width_) /
                             back_forward_reselution_) + 1;
  origin_bounder_.clear();
  origin_bounder_.reserve(point_nums);
  origin_bounder_.emplace_back(0.0, road_left_width_, 0.0);
  for (int i = 1; i < point_nums; ++i) {
    origin_bounder_.emplace_back(
        0.0, origin_bounder_.back().y - back_forward_reselution_, 0.0);
  }

  double road_left_width_2 = road_left_width_ / 3;
  double road_right_width_2 = road_right_width_ / 3;

  point_nums = std::ceil((road_left_width_2 + road_right_width_2) /
                             back_forward_reselution_) + 1;
  origin_bounder_2_.clear();
  origin_bounder_2_.reserve(point_nums);
  origin_bounder_2_.emplace_back(0.0, road_left_width_2, 0.0);
    for (int i = 1; i < point_nums; ++i) {
    origin_bounder_2_.emplace_back(
        0.0, origin_bounder_2_.back().y - back_forward_reselution_, 0.0);
  }
}

// pose transform
geometry_msgs::Pose RoadLine::PoseTransform(const RoadXY& central,
                                            const RoadXY& pose) {               
  const double yaw = central.yaw;

  geometry_msgs::Pose temp_pose;
  temp_pose.position.x = pose.x * cos(yaw) - pose.y * sin(yaw) + central.x;
  temp_pose.position.y = pose.x * sin(yaw) + pose.y * cos(yaw) + central.y;

  temp_pose.orientation = tf::createQuaternionMsgFromYaw(yaw + pose.yaw);

  return temp_pose;
}

bool RoadLine::ReadRoadLine() {
  FILE* File = fopen(file_name_.c_str(), "r");

  if (!File) {
    std::cout << "** file open fail !! **" << std::endl;
    return false;
  }
  prime_road_.clear();
  double x = 0.0, y = 0.0, yaw = 0.0, d4, d5, d6, d7, d8, d9, d10;
  int simple_cnt = 0,
      simple_segment = static_cast<int>(road_line_simple_seg_dis_ * 10.0);
  while (!feof(File)) {
    int read_result =
        fscanf(File, (const char*)"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &x,
               &y, &d4, &yaw, &d5, &d6, &d7, &d8, &d9, &d10);
    x = ReserveTwoDecimal(x);
    y = ReserveTwoDecimal(y + 2.0);
    yaw = ReserveTwoDecimal(yaw);
    if (read_result != 10) {
      std::cout << "** Read line fail !!! **" << std::endl;
    } else {
      if (simple_cnt == simple_segment) {
        prime_road_.emplace_back(x, y, DegreeToRad(yaw));
        simple_cnt = 0;
      }
      ++simple_cnt;
    }
  }
  fclose(File);

  // set start pose
  start_pose_.position.x = prime_road_[40 / simple_segment].x;
  start_pose_.position.y = prime_road_[40 / simple_segment].y;
  start_pose_.orientation =
      tf::createQuaternionMsgFromYaw(prime_road_[40 / simple_segment].yaw);
  // start_pose_.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // set goal pose
  // goal_pose_.position.x =
  //     prime_road_[prime_road_.size() - 180 / simple_segment].x;
  // goal_pose_.position.y =
  //     prime_road_[prime_road_.size() - 180 / simple_segment].y;
  // goal_pose_.orientation = tf::createQuaternionMsgFromYaw(
  //     prime_road_[prime_road_.size() - 180 / simple_segment].yaw);

  goal_pose_.position.x = prime_road_[640 / simple_segment].x;
  goal_pose_.position.y = prime_road_[640 / simple_segment].y;
  goal_pose_.orientation =
      tf::createQuaternionMsgFromYaw(prime_road_[540 / simple_segment].yaw);

  // goal_pose_.orientation = tf::createQuaternionMsgFromYaw(0.0);

  return true;
}

void RoadLine::CreateRoadBounder() {

  // refind minimal area when start and goal point changing


  road_left_bounder_.poses.clear();
  road_right_bounder_.poses.clear();
  road_back_bounder_.poses.clear();
  road_forward_bounder_.poses.clear();
  road_center_line_.poses.clear();
  all_bounder_.poses.clear();

  road_left_bounder_2_.poses.clear();
  road_right_bounder_2_.poses.clear();

  geometry_msgs::PoseStamped temp;
  temp.header.frame_id = "global";



  for (int i = 0; i < prime_road_.size(); ++i) {
    temp.pose = PoseTransform(prime_road_[i], origin_bounder_.front());
    road_left_bounder_.poses.push_back(temp);
    all_bounder_.poses.push_back(temp);

    temp.pose = PoseTransform(prime_road_[i], origin_bounder_.back());
    road_right_bounder_.poses.push_back(temp);
    all_bounder_.poses.push_back(temp);

    temp.pose = PoseTransform(prime_road_[i], RoadXY(0.0, 0.0, 0.0));
    road_center_line_.poses.push_back(temp);

    temp.pose = PoseTransform(prime_road_[i], origin_bounder_2_.front());
    road_left_bounder_2_.poses.push_back(temp);

    temp.pose = PoseTransform(prime_road_[i], origin_bounder_2_.back());
    road_right_bounder_2_.poses.push_back(temp);
  }

  for (auto& elem : origin_bounder_) {
    temp.pose = PoseTransform(prime_road_.front(), elem);
    road_back_bounder_.poses.push_back(temp);
    all_bounder_.poses.push_back(temp);
    temp.pose = PoseTransform(prime_road_.back(), elem);
    road_forward_bounder_.poses.push_back(temp);
    all_bounder_.poses.push_back(temp);
  }
}

void RoadLine::UpgrateParam() {
  ros::NodeHandle private_nh("~");

  private_nh.param<double>("road_left_width", road_left_width_, 5.4);
  private_nh.param<double>("road_right_width", road_right_width_, 5.4);
  private_nh.param<double>("back_forward_reselution", back_forward_reselution_,
                           0.1);
  CreateOriginBounder();
  CreateRoadBounder();
}

void RoadLine::StartInfoCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& start_initial) {
  start_pose_ = start_initial->pose.pose;
}

void RoadLine::GoalInfoCallback(
    const geometry_msgs::PoseStampedConstPtr& goal_initial) {
  goal_pose_ = goal_initial->pose;
}
