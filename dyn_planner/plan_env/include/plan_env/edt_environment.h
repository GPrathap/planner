#ifndef _EDT_ENVIRONMENT_H_
#define _EDT_ENVIRONMENT_H_

#include <Eigen/Eigen>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <plan_env/sdf_map.h>

using std::cout;
using std::endl;
using std::list;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace dyn_planner
{
class EDTEnvironment
{
private:
  /* data */
  SDFMap::Ptr sdf_map_;
  double resolution_inv_;

public:
  EDTEnvironment(/* args */) {}
  ~EDTEnvironment() {}

  void init();
  void setMap(SDFMap::Ptr map);
  bool odomValid() { return sdf_map_->odomValid(); }
  bool mapValid() { return sdf_map_->mapValid(); }
  nav_msgs::Odometry getOdom() { return sdf_map_->getOdom(); }
  void getMapRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) { sdf_map_->getRegion(ori, size); }
  std::vector<Eigen::Vector3d> getMapCurrentRange();
  std::vector<Eigen::Vector3d> nearest_obstacles_to_current_pose(Eigen::Vector3d x
                , int max_neighbours);
  double get_free_distance(Eigen::Vector3d x);
  bool is_inside_map(Eigen::Vector3d x);
  std::vector<std::array<double, 6>> get_obs_map();
  typedef shared_ptr<EDTEnvironment> Ptr;
};

}  // namespace dyn_planner

#endif