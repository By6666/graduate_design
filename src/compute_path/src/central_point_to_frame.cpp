/**
 * Basing central point generates obstacle or vehicle box frame.  
 */

#include "compute_path/compute_path.h"


// get obstacle frame
/* truck ego exp:
      pt4   pt1
         ego   -->
      pt3   pt2
*/
PointSet_type HybridAstar::GetObstacleFrame(
    const geometry_msgs::Pose& central,
    const geometry_msgs::Vector3& size) const {
  PointSet_type frame(4);

  /* init pt1 ~ pt4 */
  // pt1
  frame[0].x = size.x / 2.0;
  frame[0].y = size.y / 2.0;
  frame[0].z = size.z;

  // pt2
  frame[1].x = size.x / 2.0;
  frame[1].y = -size.y / 2.0;
  frame[1].z = size.z;

  // pt3
  frame[2].x = -size.x / 2.0;
  frame[2].y = -size.y / 2.0;
  frame[2].z = size.z;

  // pt4
  frame[3].x = -size.x / 2.0;
  frame[3].y = size.y / 2.0;
  frame[3].z = size.z;

  double yaw = tf::getYaw(central.orientation);

  // std::cout << "----- hollow transform frame -----" << std::endl;
  for (auto& elem : frame) {
    double x_temp = elem.x;
    double y_temp = elem.y;

    elem.x = x_temp * cos(yaw) - y_temp * sin(yaw) + central.position.x;
    elem.y = x_temp * sin(yaw) + y_temp * cos(yaw) + central.position.y;
    elem.z = central.position.z;
    // std::cout << "[" << elem.x << "," << elem.y << "," << elem.z << "]"
    //           << std::endl;
  }

  return frame;
}

PointSet_type HybridAstar::GetObstacleFrame(
    const geometry_msgs::Point& central,
    const geometry_msgs::Vector3& size) const {
  PointSet_type frame(4);

  /* init pt1 ~ pt4 */
  // pt1
  frame[0].x = size.x / 2.0;
  frame[0].y = size.y / 2.0;
  frame[0].z = size.z;

  // pt2
  frame[1].x = size.x / 2.0;
  frame[1].y = -size.y / 2.0;
  frame[1].z = size.z;

  // pt3
  frame[2].x = -size.x / 2.0;
  frame[2].y = -size.y / 2.0;
  frame[2].z = size.z;

  // pt4
  frame[3].x = -size.x / 2.0;
  frame[3].y = size.y / 2.0;
  frame[3].z = size.z;

  double yaw = central.z;

  // std::cout << "----- hollow transform frame -----" << std::endl;
  for (auto& elem : frame) {
    double x_temp = elem.x;
    double y_temp = elem.y;

    elem.x = x_temp * cos(yaw) - y_temp * sin(yaw) + central.x;
    elem.y = x_temp * sin(yaw) + y_temp * cos(yaw) + central.y;
    elem.z = 0.0;
    // std::cout << "[" << elem.x << "," << elem.y << "," << elem.z << "]"
    //           << std::endl;
  }

  return frame;
}

// get truck frame
PointSet_type HybridAstar::GetTruckFrame(const AstarNode& central) const {
  PointSet_type frame(4);

  /* init pt1 ~ pt4 */
  // pt1
  frame[0].x = truck_length_ - truck_base2back_;
  frame[0].y = truck_width_ / 2.0;
  frame[0].z = 0.0;

  // pt2
  frame[1].x = truck_length_ - truck_base2back_;
  frame[1].y = -truck_width_ / 2.0;
  frame[1].z = 0.0;

  // pt3
  frame[2].x = -truck_base2back_;
  frame[2].y = -truck_width_ / 2.0;
  frame[2].z = 0.0;

  // pt4
  frame[3].x = -truck_base2back_;
  frame[3].y = truck_width_ / 2.0;
  frame[3].z = 0.0;

  double yaw = central.yaw;
  // std::cout << "-----central.yaw ----- == " << yaw << std::endl;
  // std::cout << "-----truck transform frame -----" << std::endl;
  for (auto& elem : frame) {
    double x_temp = elem.x;
    double y_temp = elem.y;

    elem.x = x_temp * cos(yaw) - y_temp * sin(yaw) + central.x;
    elem.y = x_temp * sin(yaw) + y_temp * cos(yaw) + central.y;
    // std::cout << "[" << elem.x << "," << elem.y << "," << elem.z << "]"
    //           << std::endl;
  }

  return frame;
}
