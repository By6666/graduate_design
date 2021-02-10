/**
 * Road boundary collision check
 */

#include "compute_path/compute_path.h"


bool HybridAstar::BoundaryCollision(const AstarNode* const node_p,
                                    const PATH_TYPE& update_set) {
  for (int i = update_set.size() - 1; i >= 0;
       i -= detect_collision_point_seg_) {
    AstarNode truck_update_pos = PoseTransform(node_p, update_set[i]);
    PointSet_type truck_frame = GetTruckFrame(truck_update_pos);

    if (SinglePointDetect(truck_update_pos, truck_frame)) {
      return true;
    }
  }
  return false;
}

bool HybridAstar::SinglePointDetect(const AstarNode& truck_update_pos,
                                    const PointSet_type& truck_frame) {
  double cos_theta = std::cos(truck_update_pos.yaw);
  double sin_theta = std::sin(truck_update_pos.yaw);
  double x = truck_length_ / 2.0 - truck_base2back_;
  double y = 0.0;
  double truck_central_x = x * cos_theta - y * sin_theta + truck_update_pos.x;
  double truck_central_y = x * sin_theta + y * cos_theta + truck_update_pos.y;

  KDTreeSP::point_t truck_central{truck_central_x, truck_central_y};

  KDTreeSP::point_t nearest = kdtree_bounder_.nearest_point(truck_central);

  return NearestInTruckFrame(truck_frame, truck_central, nearest);
}

bool HybridAstar::NearestInTruckFrame(const PointSet_type& truck_frame,
                                      const KDTreeSP::point_t& truck_central,
                                      const KDTreeSP::point_t& nearest) {
  const static double dis_1 = std::pow(truck_width_ / 2.0, 2.0);  // 内
  const static double dis_2 = std::pow(truck_length_ / 2.0, 2.0) + dis_1;  // 外

  double curr_dis = KDTreeSP::dist2(truck_central, nearest);

  if (curr_dis > dis_2) return false;
  if (curr_dis < dis_1) return true;

  //**射线法判断点与多边形的关系
  bool flag = false;
  for (int i = 0, j = truck_frame.size() - 1; i < truck_frame.size(); j = i++) {
    if (((truck_frame[i].y > nearest.back()) !=
         (truck_frame[j].y > nearest.back())) &&
        (nearest.front() < (truck_frame[j].x - truck_frame[i].x) *
                                   (nearest.back() - truck_frame[i].y) /
                                   (truck_frame[j].y - truck_frame[i].y) +
                               truck_frame[i].x))
      flag = !flag;
  }
  return flag;
}
