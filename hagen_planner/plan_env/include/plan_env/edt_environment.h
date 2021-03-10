#ifndef _EDT_ENVIRONMENT_H_
#define _EDT_ENVIRONMENT_H_

#include <Eigen/Eigen>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <plan_env/edtoctomap.h>
#include<random>
#include<cmath>
#include<chrono>
#include <traj_utils/math_utils.h>

using std::cout;
using std::endl;
using std::list;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace hagen_planner
{
class EDTEnvironment
{
private:
  /* data */
  EDTOctoMap::Ptr sdf_map_;
  double resolution_inv_;
  MathUtils math_util;
  int num_of_points = 10;
  Eigen::MatrixXd search_space;

public:
  EDTEnvironment(/* args */) {}
  ~EDTEnvironment() {}

  void init();
  void setMap(EDTOctoMap::Ptr map);
  bool odomValid() { return sdf_map_->odomValid(); }
  bool mapValid() { return sdf_map_->mapValid(); }
  nav_msgs::Odometry getOdom() { return sdf_map_->getOdom(); }
  void getMapRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) { sdf_map_->getRegion(ori, size); }
  std::vector<Eigen::Vector3d> getMapCurrentRange();
  std::vector<Eigen::Vector3d> nearest_obstacles_to_current_pose(Eigen::Vector3d x
                , int max_neighbours);
  void get_close_obstacles(Eigen::Vector3d pos, std::vector<Eigen::Vector3d>& obs_vec);
  double get_free_distance(Eigen::Vector3d x);
  bool is_inside_map(Eigen::Vector3d x);
  std::vector<std::array<double, 6>> get_obs_map();
  void get_close_obstacle(Eigen::Vector3d x, Eigen::Vector3d& close_obs, double& dis);
  bool collision_free(Eigen::Vector3d start, Eigen::Vector3d end, double min_dis, double avoidance_dis);
  bool get_projected_point(Eigen::Vector3d center, double radius, double avoidance_dis, Eigen::Vector3d& projected_pose);
  bool get_projected_point(Eigen::Vector3d start, Eigen::Vector3d end, double inner_dis, double outer_dis
                          , double avoidance_dis, double avoidance_dis_max, Eigen::Vector3d& projected_pose);
  void get_cone_points(Eigen::MatrixXd& poses);
  void get_constraint_polyhedra(Eigen::Vector3d pose);
  typedef shared_ptr<EDTEnvironment> Ptr;
  int getInflateOccupancy(Eigen::Vector3d pos);
  double getResolution();
};

}  // namespace hagen_planner

#endif