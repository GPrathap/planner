#ifndef _NON_LINEAR_TRAJECTORY_TRACKER_VALIDATE_OPT_
#define _NON_LINEAR_TRAJECTORY_TRACKER_VALIDATE_OPT_

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
class TrajectoryTrackerValidate : public  NonLinearMPCOpt
{
private:
  double delta_t_desired = 0.02;
  KalmanFilter* kf; 
  // KalmanFilter* kf_position;
  KalmanFilter* kf_nmpc;
  std::string solver_state_error = "Infeasible_Problem_Detected";
  boost::circular_buffer<Eigen::Vector3d>* traj_cmd;

  
public:
   TrajectoryTrackerValidate();
  ~TrajectoryTrackerValidate() = default;

  void solver_init();
  void init(ros::NodeHandle& nh);
  std::vector<Eigen::Vector3d> mpc_solver();
  std::vector<Eigen::Vector3d> mpc_solver_with_collocation();
  std::vector<Eigen::Vector3d> mpc_solver_with_multiple_shooting();

  void setTrajectoryGenerator(const BSplineUtils::Ptr& manager);

  typedef std::shared_ptr<TrajectoryTrackerValidate> Ptr;
  BSplineUtils::Ptr bspline_utils_;
  PlanningSaving::Ptr planner_saving;
  ParamPasser::Ptr passer;
};
}  // namespace hagen_planner
#endif
