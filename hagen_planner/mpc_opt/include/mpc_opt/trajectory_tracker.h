#ifndef _NON_LINEAR_TRAJECTORY_TRACKER_OPT_
#define _NON_LINEAR_TRAJECTORY_TRACKER_OPT_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <vector>
#include <memory>
#include <casadi/casadi.hpp>
#include <plan_env/edt_environment.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/planning_saving.h>
#include <traj_utils/param_passer.h>
#include <rebound_opt/rebound_optimizer.h>
#include <boost/circular_buffer.hpp>
#include <tf/tf.h>
#include <time.h>
#include "kalman.h"
#include "mpc_opt/nonlinear_mpc_opt.h"
#include "mpc_opt/bspline_utils.h"

using namespace casadi;
using namespace std;

namespace hagen_planner
{
class TrajectoryTracker : public  NonLinearMPCOpt
{
private:
  double delta_t_desired = 0.02;
  KalmanFilter* kf; 
  // KalmanFilter* kf_position;
  KalmanFilter* kf_nmpc;
  std::string solver_state_error = "Infeasible_Problem_Detected";
  boost::circular_buffer<Eigen::Vector3d>* traj_cmd;

  
public:
   TrajectoryTracker();
  ~TrajectoryTracker() = default;

  void solver_init();
  void init(ros::NodeHandle& nh);
  void mpc_solver();
  void mpc_solver_with_collocation();
  void mpc_solver_with_multiple_shooting();
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void setTrajectoryGenerator(const BSplineUtils::Ptr& manager);
  void setBoundaryChecker(const ego_planner::BsplineOptimizer::Ptr& manager);
 
  typedef std::shared_ptr<TrajectoryTracker> Ptr;
  BSplineUtils::Ptr bspline_utils_;
  ego_planner::BsplineOptimizer::Ptr bound_checker;
  PlanningSaving::Ptr planner_saving;
  ParamPasser::Ptr passer;
  nav_msgs::Odometry reference_odom;
  bool is_reference_trj_set = false;
};
}  // namespace hagen_planner
#endif
