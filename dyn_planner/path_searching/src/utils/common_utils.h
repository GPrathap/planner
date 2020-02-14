#ifndef _UTILS_COMMON_UTILS_H_
#define _UTILS_COMMON_UTILS_H_

#include <stack>
#include <vector>
#include <array>
#include <iostream>
#include <set>
#include <cstdlib>
#include <climits>
#include <cmath>
#include <cerrno>
#include <cfenv>
#include <cstring>
#include <Eigen/Dense>
#include <chrono> 
#include <vector>
// #include <cnpy.h>
#include <random>
#include <algorithm>
#include <list>
#include <numeric>
#include <unordered_map>
#include <memory>
#include <type_traits>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <signal.h>
#include <deque>
#include <atomic>

#include <boost/function_output_iterator.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/timer.hpp>
#include <boost/foreach.hpp>
#include "../include/colours.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
// #include <eigen_conversions/eigen_msg.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
// #include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
// #include <pcl_conversions/pcl_conversions.h>

// #include <ros/ros.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <ros/console.h>
// #include <tf_conversions/tf_eigen.h>
// #include <tf/transform_listener.h>
// #include <tf/message_filter.h>
// #include <message_filters/subscriber.h>
// #include <geometry_msgs/QuaternionStamped.h>
// #include <geometry_msgs/Vector3Stamped.h>
// #include <sensor_msgs/NavSatFix.h>
// #include <std_msgs/UInt8.h>
// #include <nav_msgs/Path.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <nav_msgs/Odometry.h>
// #include <rviz_visual_tools/rviz_visual_tools.h>
// #include <tf/tf.h>
// #include <sensor_msgs/Joy.h>

// #include <dji_sdk/Gimbal.h>

namespace kamaz {
  namespace hagen{
class CommonUtils {
 public:
  
  CommonUtils() = default;
  ~CommonUtils() = default;
  void generate_samples_from_ellipsoid(Eigen::MatrixXd covmat
            , Eigen::Matrix3d rotation_mat, Eigen::Vector3d cent
            , Eigen::MatrixXd& container);
  
  void get_roration_matrix(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Matrix3d& r);
  double get_cost_of_path(std::vector<Eigen::Vector3d> path1);
  std::vector<Eigen::Vector3d> next_poses(Eigen::VectorXd start_position, Eigen::VectorXd end_position
      , double distance);

  void get_point_on_the_trajectory(Eigen::Vector3d way_point
      , Eigen::Vector3d start_point,  Eigen::Vector3d& path_position);

  double voxel_side_length;
  Eigen::Vector3d init_min_point;
  std::string world_frame_id;
};

}  
}
#endif  // _UTILS_COMMON_UTILS_H_
