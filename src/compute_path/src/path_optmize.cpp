#include "compute_path/compute_path.h"
#include "tf/tf.h"

#include <array>

#include "piecewise_jerk/piecewise_jerk_path_problem.h"
#include "common/linear_interpolation.h"

bool HybridAstar::OptimizerProcess() {
  // accoding to optimize seg, set optimize path nums
  // ** puls one, to add start point
  int num_of_knots =
      std::floor(goal_pose_.position.x / optimize_segment_dis_) + 1;

  // accoding to goal pose, set optimize path length
  double path_length = optimize_segment_dis_ * num_of_knots;

  // set init and goal state
  double goal_x = goal_pose_.position.x;
  double goal_y = goal_pose_.position.y;
  double goal_yaw = tf::getYaw(goal_pose_.orientation);

  // get init and goal state
  std::array<double, 3> init_state = {0.0, 0.0, 0.0};
  std::array<double, 3> goal_state = {goal_y, goal_yaw, 0.0};

  // get boundary
  std::vector<double> ref_line(num_of_knots);
  GetReferenceLine(&ref_line);

  // get reference
  std::vector<std::pair<double, double>> x_bounds(num_of_knots);
  GetxBounds(&x_bounds);

  // generate optimizer
  math::piecewise::PiecewiseJerkPathProblem optimizer(num_of_knots,
                                                      optimize_segment_dis_);
  optimizer.set_x_bounds(x_bounds);     // station bounds
  optimizer.set_ddx_bounds(-4.0, 4.0);  // caverture bounds

  optimizer.set_weight_x(1.0);
  // set_weight_dx(1e-6);
  optimizer.set_weight_ddx(1.0);
  optimizer.set_weight_dddx(2);

  optimizer.set_x_ref(ref_line);

  // set hard and soft constraints
  if (init_state.size() != optimizer.get_layers().Rank()) {
    std::cout << "Init state input error !!" << std::endl;
  }

  // set start init hard constrain
  for (int i = 0; i < init_state.size(); ++i) {
    optimizer.set_layers()[i].SetBoundary({init_state[i], init_state[i]}, 0);
  }
  // // set end hard constrain
  // for (int i = 1; i < goal_state.size(); ++i) {
  //   optimizer.set_layers()[i].SetBoundary({goal_state[i], goal_state[i]},
  //                                         num_of_knots - 1);
  // }

  // set end s target
  optimizer.set_layers()[0].SetTarget(goal_state[0], num_of_knots - 1);
  optimizer.set_layers()[0].SetWeight(1000.0, num_of_knots - 1);
  optimizer.set_layers()[1].SetTarget(goal_state[1], num_of_knots - 1);
  optimizer.set_layers()[1].SetWeight(1e5, num_of_knots - 1);

  // go optimize
  auto optimize_result = optimizer.Optimize();
  std::cout << optimize_result.second << std::endl;

  if (!optimize_result.first) return 0;

  // Extract output
  const std::vector<double>& y = optimizer.opt_x();
  const std::vector<double>& dy = optimizer.opt_dx();
  const std::vector<double>& ddy = optimizer.opt_ddx();

  // std::cout << "***** optimize result ****** [x, y, dy, ddy]" << std::endl;
  // for (int i = 0; i < num_of_knots; ++i) {
  //   std::cout << optimize_segment_dis_ * i << ", " << y[i] << ", " << dy[i]
  //             << ", " << ddy[i] << std::endl;
  // }
  std::cout << "goal_pose[x, y, yaw] : [" << goal_pose_.position.x << ", "
            << goal_pose_.position.y << ", "
            << tf::getYaw(goal_pose_.orientation) << "]" << std::endl;

  optimize_final_path_.clear();
  for (int i = 0; i < num_of_knots; ++i) {
    double curr_x = i * optimize_segment_dis_;

    geometry_msgs::Pose temp_pose;

    temp_pose.position.x = curr_x;
    temp_pose.position.y = y[i];
    temp_pose.position.z = ddy[i];
    temp_pose.orientation = tf::createQuaternionMsgFromYaw(dy[i]);

    optimize_final_path_.push_back(temp_pose);
  }

  return true;
}

// for path optimize
void HybridAstar::GetReferenceLine(std::vector<double>* const ref_line) {
  for (int i = 0; i < ref_line->size(); ++i) {
    double curr_x = i * optimize_segment_dis_;

    // final_path_
    ref_line->at(i) = FindNearstY(final_path_, curr_x);
    // ref_line->at(i) = FindNearstY(ref_line_, curr_x);
  }

  std::cout << "******* optimize ref line: [x, ref]" << std::endl;

  for (int i = 0; i < ref_line->size(); ++i) {
    double curr_x = i * optimize_segment_dis_;

    geometry_msgs::Pose temp_point;
    temp_point.position.x = curr_x;
    temp_point.position.y = ref_line->at(i);
    temp_point.orientation = tf::createQuaternionMsgFromYaw(0.0);

    geometry_msgs::Pose temp_point2 =
        PoseTransform(prime_start_pose_, temp_point);

    std::cout << temp_point2.position.x << ", " << temp_point2.position.y
              << std::endl;

    // std::cout << curr_x << ", " << ref_line->at(i) << std::endl;
  }
}

void HybridAstar::GetxBounds(
    std::vector<std::pair<double, double>>* const x_bounds) {
  int num_of_knots = x_bounds->size();

  // array[]  -> min_x, max_x, min_y, max_y
  std::vector<std::array<double, 4>> left_decision_obs_frame;
  std::vector<std::array<double, 4>> right_decision_obs_frame;

  for (int i = 0; i < obstacles_info_.obstacles.size(); ++i) {
    if (obstacles_info_.obstacles[i].is_static) {
      const auto& obs = obstacles_list_[i];

      if (obstacles_info_.obstacles[i].path_decision == "left") {
        auto min_max_frame = GetObjectMinMaxFrame(obs);
        // print obs min max boundary
        std::cout
            << "obstacle min_max_boundary:[min_x, max_x, min_y, max_y] : ["
            << min_max_frame[0] << ", " << min_max_frame[1] << ", "
            << min_max_frame[2] << ", " << min_max_frame[3] << "]" << std::endl;
        left_decision_obs_frame.push_back(min_max_frame);
      }

      if (obstacles_info_.obstacles[i].path_decision == "right") {
        auto min_max_frame = GetObjectMinMaxFrame(obs);
        // print obs min max boundary
        std::cout
            << "obstacle min_max_boundary:[min_x, max_x, min_y, max_y] : ["
            << min_max_frame[0] << ", " << min_max_frame[1] << ", "
            << min_max_frame[2] << ", " << min_max_frame[3] << "]" << std::endl;
        right_decision_obs_frame.push_back(min_max_frame);
      }
    }
  }

  // road left and right boundary
  for (int i = 0; i < num_of_knots; ++i) {
    double curr_x = i * optimize_segment_dis_;
    x_bounds->at(i).first = FindNearstY(right_boundary_, curr_x);
    x_bounds->at(i).second = FindNearstY(left_boundary_, curr_x);

    for (const auto& elem : left_decision_obs_frame) {
      if (curr_x > elem[0] && curr_x < elem[1]) {
        x_bounds->at(i).first = std::max(x_bounds->at(i).first, elem[3]);
      }
    }

    for (const auto& elem : right_decision_obs_frame) {
      if (curr_x > elem[0] && curr_x < elem[1]) {
        x_bounds->at(i).second = std::min(x_bounds->at(i).second, elem[2]);
      }
    }
  }

  // judge boundary base vehicle box
  for (int i = 0; i < num_of_knots; ++i) {
    double curr_x = i * optimize_segment_dis_;

    auto nearst_point = FindNearstPointOnSearchPath(final_path_, curr_x);

    auto vehicle_points = GetTruckFrame(nearst_point);

    auto vehicle_minmax_box = GetObjectMinMaxFrame(vehicle_points);

    x_bounds->at(i).first =
        x_bounds->at(i).first - (vehicle_minmax_box[2] - nearst_point.y);
    x_bounds->at(i).second =
        x_bounds->at(i).second - (vehicle_minmax_box[3] - nearst_point.y);
  }

  // print boundary limit
  std::cout << "******* optimize left and right: [x, right, left]" << std::endl;
  for (int i = 0; i < num_of_knots; ++i) {
    const auto& bounds = x_bounds->at(i);

    geometry_msgs::Pose temp_point;
    temp_point.position.x = i * optimize_segment_dis_;
    temp_point.position.y = bounds.first;
    temp_point.orientation = tf::createQuaternionMsgFromYaw(0.0);

    geometry_msgs::Pose temp_point2 =
        PoseTransform(prime_start_pose_, temp_point);

    // geometry_msgs::Pose temp_point;
    temp_point.position.x = i * optimize_segment_dis_;
    temp_point.position.y = bounds.second;
    temp_point.orientation = tf::createQuaternionMsgFromYaw(0.0);

    geometry_msgs::Pose temp_point3 =
        PoseTransform(prime_start_pose_, temp_point);

    std::cout << temp_point2.position.x << ", " << temp_point2.position.y
              << ", " << temp_point3.position.x << ", "
              << temp_point3.position.y << std::endl;

    // std::cout << i * optimize_segment_dis_ << ", " << bounds.first << ", "
    //           << bounds.second << std::endl;
  }
}

double HybridAstar::FindNearstY(const std::vector<geometry_msgs::Point>& line,
                                double arg_x) {
  auto cmp = [](double number, const geometry_msgs::Point& point) {
    return number < point.x;
  };
  auto it_upper = std::upper_bound(line.begin(), line.end(), arg_x, cmp);

  if (it_upper == line.end()) {
    return line.back().y;
  }

  const auto& first_point = *(it_upper - 1);
  const auto& second_point = *(it_upper);

  return math::common::lerp(first_point.y, first_point.x, second_point.y,
                            second_point.x, arg_x);
}

double HybridAstar::FindNearstY(const std::vector<geometry_msgs::Pose>& line,
                                double arg_x) {
  auto cmp = [](double number, const geometry_msgs::Pose& point) {
    return number < point.position.x;
  };
  auto it_upper = std::upper_bound(line.begin(), line.end(), arg_x, cmp);

  if (it_upper == line.end()) {
    return line.back().position.y;
  }

  const auto& first_point = *(it_upper - 1);
  const auto& second_point = *(it_upper);

  return math::common::lerp(first_point.position.y, first_point.position.x,
                            second_point.position.y, second_point.position.x,
                            arg_x);
}

AstarNode HybridAstar::FindNearstPointOnSearchPath(
    const std::vector<geometry_msgs::Pose>& line, double arg_x) {
  double y = 0.0, heading = 0.0;

  auto cmp = [](double number, const geometry_msgs::Pose& point) {
    return number < point.position.x;
  };
  auto it_upper = std::upper_bound(line.begin(), line.end(), arg_x, cmp);

  if (it_upper == line.end()) {
    y = line.back().position.y;
    heading = tf::getYaw(line.back().orientation);
  } else {
    const auto& first_point = *(it_upper - 1);
    const auto& second_point = *(it_upper);

    y = math::common::lerp(first_point.position.y, first_point.position.x,
                           second_point.position.y, second_point.position.x,
                           arg_x);

    heading = math::common::slerp(
        tf::getYaw(first_point.orientation), first_point.position.x,
        tf::getYaw(second_point.orientation), second_point.position.x, arg_x);
  }

  return AstarNode(arg_x, y, heading, 0.0, 0.0);
}

std::array<double, 4> HybridAstar::GetObjectMinMaxFrame(
    const PointSet_type& obs_points) {
  double min_x = DBL_MAX, min_y = DBL_MAX;
  double max_x = DBL_MIN, max_y = DBL_MIN;

  for (const auto& point : obs_points) {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }

  // print obs min max boundary
  // std::cout << "obstacle min_max_boundary:[min_x, max_x, min_y, max_y] : ["
  //           << min_x << ", " << max_x << ", " << min_y << ", " << max_y <<
  //           "]"
  //           << std::endl;
  return std::array<double, 4>{min_x, max_x, min_y, max_y};
}

// double init_v = 8.0, init_a = 0.0, stop_distance = 15.0;
// const double delta_t = 0.02;
// const double total_time = 3.0;
// size_t num_of_knots = static_cast<size_t>(total_time / delta_t) + 1;

// init_a = std::min(init_a, 0.0);

// std::array<double, 3> init_s = {0.0, init_v, init_a};
// std::array<double, 3> end_s = {stop_distance, 0.0, 0.0};

// math::piecewise::PiecewiseJerkSpeedProblem project(num_of_knots, delta_t);
// project.SetConsition(init_s, end_s);

// auto success = project.Optimize();
