#ifndef COMPUTE_PATH_H
#define COMPUTE_PATH_H

#include <cmath>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <vector>

#include "KDTree.h"
#include "compute_path/collision_check.h"
#include "compute_path/hint_show.h"
#include "compute_path/state.h"
#include "geometry_msgs/Pose.h"
#include "optimize_path/PathFitInfo.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "visualization_msgs/MarkerArray.h"

typedef std::map<int, AstarNode> NODE_TYPE;
typedef std::vector<geometry_msgs::Pose> PATH_TYPE;
typedef std::vector<geometry_msgs::Pose> UPDATE_POS;
typedef std::vector<std::vector<geometry_msgs::Pose>> UPDATE_SET;
typedef std::priority_queue<ComNode, std::vector<ComNode>,
                            std::greater<ComNode>>
    OPENLIST;

class HybridAstar {
 public:
  HybridAstar();

  void PrintPath();
  bool ExecuteHybridAstar();

  void Init();
  void RecurPath();
  bool FinalPath();
  bool ComputePath();
  bool detectCollision(const AstarNode& node);
  //**  新的碰撞检测, 基于去除costmap的搜索方法
  bool DetectCollision(const AstarNode* const node_p,
                       const PATH_TYPE& update_set);
  bool PushNode(const AstarNode& node, AstarNode* const p);
  std::vector<AstarNode> GetNeighbors(const AstarNode* const node);

  void UpgrateParam();
  void CreateHollowList();
  void CreateUpdateSet();
  void CreateUpdateSet(int num);
  void UpdatePoseShow(const ros::Publisher& pub);
  UPDATE_SET UpdateSetTransform(const geometry_msgs::Pose& pose);
  UPDATE_POS UpdateNodeTransform(const geometry_msgs::Pose& pose);

  inline std::set<int>& set_obs_list() { return obs_list_; }
  inline _Type_Hollow& set_hollow_list() { return hollow_info_; }
  inline double& set_map_resolution() { return map_resolution_; }
  inline std::vector<int8_t>& set_map_data() { return map_data_; }
  inline geometry_msgs::Pose& set_goal_pose() { return goal_pose_; }
  inline geometry_msgs::Pose& set_start_pose() { return start_pose_; }
  inline geometry_msgs::Point& set_map_origin() { return map_origin_; }
  inline double& set_truck_width() { return truck_width_; }
  inline double& set_truck_length() { return truck_length_; }
  inline double& set_truck_base2back() { return truck_base2back_; }
  inline int& set_map_heigh() { return heigh_; }
  inline int& set_map_width() { return width_; }
  inline KDTreeSP::pointVec& set_bounder_info() { return bounder_info_; }

  inline int get_angle_size() const { return angle_size_; }
  inline double get_move_step() const { return move_step_; }
  inline PATH_TYPE get_final_path() const { return final_path_; }
  inline std::set<int> get_obs_list() const { return obs_list_; }
  inline PATH_TYPE get_prime_path() const { return prime_path_temp_; }
  inline UPDATE_POS get_update_set() const { return update_set_; }
  inline geometry_msgs::Pose get_goal_pose() const { return goal_pose_; }
  inline geometry_msgs::Pose get_start_pose() const { return start_pose_; }
  inline UPDATE_SET get_update_pass_set() const { return update_pass_set_; }
  inline double get_path_length() const { return path_length_; }
  inline double get_max_curvature_diff() const { return max_curvature_diff_; }
  inline double get_curvature_average_diff() const {
    return curvature_average_diff_;
  }
  inline double get_path_evaluate_value() const {
    return path_curvature_value_;
  }
  inline int get_extension_pos_nums() const { return expend_pos_stg_.size(); }
  inline const std::vector<double>& get_curvature_data() const {
    return curvature_data_stg_;
  }

  inline void PrintMapSize() const {
    std::cout << "map_size: [" << width_ << "," << heigh_ << "]"
              << "  angle_size: " << angle_size_
              << "  extension_nums: " << extension_point_num_ << std::endl;
  }

 private:
  /* necessary container */
  OPENLIST openlist_;
  NODE_TYPE node_stg_;
  PATH_TYPE final_path_;
  PATH_TYPE prime_path_temp_;
  std::vector<AstarNode*> prime_path_;
  std::vector<float> prime_path_optimize_;

  /* parameter for path optimize */
  int path_seg_num_;
  int path_resolution_;
  int path_fit_degree_;
  bool path_optimize_flg_;

  /* for hybrid Astar */
  int width_, heigh_;
  double min_turning_radius_;
  int goal_pose_id_, start_pose_id_;
  double node2goal_r_, node2goal_yaw_;

  //** 2020.03.04 modify
  int angle_size_;
  double move_step_, segment_dis_;
  int extension_point_num_;
  bool use_goal_flg_;
  std::vector<double> update_points_orientation_stg_;
  double heur_dis_cof_, heur_yaw_cof_goal_, heur_yaw_cof_last_,
      heur_curvature_cof_;
  double path_curvature_value_, path_length_, max_curvature_diff_,
      curvature_average_diff_;  // 评估最终路径的得分
  std::vector<double> curvature_data_stg_;

  //** 待删除
  double discrete_angle_;
  int path_discrete_num_;

  /* pre container */
  UPDATE_POS update_set_;
  std::set<int> obs_list_;
  _Type_Hollow hollow_info_;
  UPDATE_SET update_pass_set_;
  std::multiset<int> expend_pos_stg_;
  std::vector<PointSet_type> hollow_list_;
  KDTreeSP::pointVec bounder_info_;
  KDTreeSP::KDTree kdtree_bounder_;

  /* parameter for map */
  double map_resolution_;
  std::vector<int8_t> map_data_;
  geometry_msgs::Point map_origin_;
  AstarNode *start_info_, *goal_info_, *find_goal_info_;
  geometry_msgs::Pose start_pose_, goal_pose_;

  /* param for truck */
  double truck_length_, truck_width_, truck_base2back_;

  // ros::NodeHandle path_op_;
  ros::ServiceClient path_op_client_;

  //*** for detective collision
  double detect_collision_rate_;  // [0,1]
  double detect_collision_point_seg_;

  inline void CreateKDTreeBounderInfo() {
    ros::WallTime start = ros::WallTime::now();
    kdtree_bounder_ = KDTreeSP::KDTree(bounder_info_);
    ros::WallTime end = ros::WallTime::now();

    std::cout << "KDTree create *** size:" << bounder_info_.size()
              << "  time:" << (end - start).toSec() * 1000 << " ms"
              << std::endl;
  }
  bool NearestInTruckFrame(const PointSet_type& truck_frame,
                           const KDTreeSP::point_t& truck_central,
                           const KDTreeSP::point_t& nearest);

  bool SinglePointDetect(const AstarNode& truck_update_pos,
                         const PointSet_type& truck_frame);

  // get hollow frame
  PointSet_type GetHollowFrame(const geometry_msgs::Pose& central,
                               const geometry_msgs::Vector3& size) const;
  // get truck frame
  PointSet_type GetTruckFrame(const AstarNode& central) const;

  geometry_msgs::Pose PoseTransform(const geometry_msgs::Pose& central,
                                    const geometry_msgs::Pose& pose);
  geometry_msgs::Pose PoseTransformPath(const AstarNode* const central,
                                        const geometry_msgs::Pose& pose);
  AstarNode PoseTransform(const AstarNode* const central,
                          const geometry_msgs::Pose& pose);

  //** 2020.03.19 modify
  // calculate total id (3 dim)
  inline _Type_ID CalculateID(const AstarNode* const node) {
    return CalculateID(node->x, node->y, node->yaw);
  }
  inline _Type_ID CalculateID(const AstarNode& node) {
    return CalculateID(node.x, node.y, node.yaw);
  }
  inline _Type_ID CalculateID(double x, double y, double yaw) {
    return Code3D(CalculateXYIndex(x, y), CalculateYawIndex(yaw));
  }
  // inline _Type_ID CalculateXYIndex(double x, double y) {
  //   return Code2D(
  //       static_cast<_Type_ID>((y - map_origin_.y) /** 50.0*/ /
  //       map_resolution_), static_cast<_Type_ID>((x - map_origin_.x)
  //       /** 50.0*/ /
  //                             map_resolution_));
  // }
  inline _Type_ID CalculateXYIndex(double x, double y) {
    return Code2D(static_cast<_Type_ID>(y / map_resolution_),
                  static_cast<_Type_ID>(x / map_resolution_));
  }

  inline int calculateXYIndexMap(double x, double y) {
    int raw = static_cast<int>((y - map_origin_.y) / map_resolution_);
    int col = static_cast<int>((x - map_origin_.x) / map_resolution_);

    return raw * width_ + col;
  }

  //** 2020.03.04 modify
  //** 返回[-pi,pi）
  inline double NormalizeAngle(const double angle) {
    //** fmod 浮点余数
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) {
      a += (2.0 * M_PI);
    }
    return a - M_PI;
  }

  //** 2020.03.04 modify
  inline int CalculateYawIndex(double yaw) {
    //** 待修改
    // while (yaw < DBL_EPSILON) yaw += 2 * M_PI;
    yaw = NormalizeAngle(yaw);
    // yaw = NormalizeAngle(yaw);
    // return static_cast<int>(yaw / discrete_angle_) % angle_size_;
    return static_cast<int>((yaw + M_PI) / (2.0 * M_PI) * angle_size_);
  }
  // inline int CodeXD(double x) {
  //   return static_cast<int>((x - map_origin_.x) / map_resolution_);
  // }
  // inline int CodeYD(double y) {
  //   return static_cast<int>((y - map_origin_.y) / map_resolution_);
  // }
  //** 2020.03.19 modify
  inline _Type_ID Code2D(_Type_ID raw, _Type_ID col) {
    return raw * width_ + col;
  }
  inline _Type_ID Code3D(_Type_ID raw, _Type_ID col) {
    return raw * angle_size_ + col;
  }

  // calculate distence of tow points
  inline double CalculateDisTwoPoint(double x1, double y1, double x2,
                                     double y2) {
    return sqrt(pow(x1 - x2, 2.0) + pow(y1 - y2, 2.0));
  }
  inline double CalculateDisTwoPoint(const geometry_msgs::Point& lf,
                                     const geometry_msgs::Point& rg) {
    return CalculateDisTwoPoint(lf.x, lf.y, rg.x, rg.y);
  }
  inline double CalculateDisTwoPoint(const AstarNode& lf, const AstarNode& rg) {
    return CalculateDisTwoPoint(lf.x, lf.y, rg.x, rg.y);
  }

  //** 2020.03.04 modify
  // calculate heuristic value
  inline double CalculateHeuristic(const AstarNode& node,
                                   const AstarNode* const np
                                   /*const double dis_yaw_to_last*/) {
    double dis_yaw_to_last = fabs(node.yaw - np->yaw);
    dis_yaw_to_last = (dis_yaw_to_last > M_PI) ? (2.0 * M_PI - dis_yaw_to_last)
                                               : dis_yaw_to_last;
    double dis_yaw_to_goal = fabs(node.yaw - goal_info_->yaw);
    dis_yaw_to_goal = (dis_yaw_to_goal > M_PI) ? (2.0 * M_PI - dis_yaw_to_goal)
                                               : dis_yaw_to_goal;

    //** 2020.03.19 modify
    //** 添加对起点处的曲率变化约束
    double heur_yaw_cof_last =
        np->best_p == NULL ? heur_yaw_cof_last_ * 10 : heur_yaw_cof_last_;

    //** 添加曲率约束cost
    double dis_curvature = 0.0;
    if (np->best_p != NULL) {
      dis_curvature = fabs(TranformYawRange(node.yaw - np->yaw) -
                           TranformYawRange(np->yaw - np->best_p->yaw));
    }
    double heur_curvature_cof =
        pow(heur_curvature_cof_, JudgeOrientation(dis_curvature));

    return heur_dis_cof_ * CalculateDisTwoPoint(node.x, node.y,
                                                goal_pose_.position.x,
                                                goal_pose_.position.y) +
           heur_yaw_cof_goal_ * dis_yaw_to_goal +
           heur_yaw_cof_last * dis_yaw_to_last +
           heur_curvature_cof * dis_curvature;
    //  map_data_[calculateXYIndexMap(node.x, node.y)];
  }

  // calculate key
  inline KeyValue CalculateKey(const AstarNode& node) {
    return KeyValue(node.g + node.h, node.g);
  }
  inline KeyValue CalculateKey(const AstarNode* const node) {
    return KeyValue(node->g + node->h, node->g);
  }

  inline bool IsGoal(const AstarNode& node) {
    return (CalculateDisTwoPoint(node.x, node.y, goal_info_->x, goal_info_->y) <
            node2goal_r_) &&
           (fabs(node.yaw - goal_info_->yaw) < node2goal_yaw_);
  }
  inline bool IsGoal(const AstarNode* const node) {
    return (CalculateDisTwoPoint(node->x, node->y, goal_info_->x,
                                 goal_info_->y) < node2goal_r_) &&
           (TranformYawRange(fabs(node->yaw - goal_info_->yaw)) <
            node2goal_yaw_);
  }

  // is obstacle
  inline bool IsObstacle(int xy_index) {
    return obs_list_.find(xy_index) != obs_list_.end();
  }
  inline bool IsInMap(double x, double y) {
    return x >= map_origin_.x && y >= map_origin_.y &&
           (x - map_origin_.x) <= static_cast<double>(width_) &&
           (y - map_origin_.y) <= static_cast<double>(heigh_);
  }

  // transform yaw in range [-pi, pi]
  inline double TranformYawRange(double yaw) {
    while (yaw > M_PI) yaw -= 2.0 * M_PI;
    while (yaw < -M_PI) yaw += 2.0 * M_PI;
    return yaw;
  }

  // judge forward, left or right
  //** //** 2020.03.04 modify
  inline int JudgeOrientation(double delta_yaw) {
    double temp_delta_yaw = fabs(delta_yaw);
    if (temp_delta_yaw < DBL_EPSILON) return 0;

    int i = std::lower_bound(update_points_orientation_stg_.begin(),
                             update_points_orientation_stg_.end(),
                             temp_delta_yaw - 1e-5) -
            update_points_orientation_stg_.begin();

    i = (i >= update_points_orientation_stg_.size())
            ? update_points_orientation_stg_.size() - 1
            : i;
    if (delta_yaw > 0.0) return i * 2 - 1;
    if (delta_yaw < 0.0) return i * 2;
  }

  inline geometry_msgs::Pose XYYAWTramsformGeometryPose(float x, float y,
                                                        float yaw) const {
    geometry_msgs::Pose temp;
    temp.position.x = x;
    temp.position.y = y;
    temp.position.z = 0.0;

    temp.orientation = tf::createQuaternionMsgFromYaw(yaw);

    return temp;
  }

  inline std::vector<float> GetXYYAWvector() const {
    std::vector<float> result;
    result.reserve(prime_path_.size() * 3);

    for (auto& elem : prime_path_) {
      result.push_back(elem->x);
      result.push_back(elem->y);
      result.push_back(elem->yaw);
    }
    return result;
  }
};

#endif