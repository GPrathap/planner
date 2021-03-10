#ifndef _NON_LINEAR_MPC_REGULATOR_OPT_
#define _NON_LINEAR_MPC_REGULATOR_OPT_

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
#include <atomic> 

using namespace casadi;
using namespace std;

namespace hagen_planner
{
class TrajectoryRegulator : public  NonLinearMPCOpt
{
  private:
    KalmanFilter* kf;
    // KalmanFilter* kf_position;
    KalmanFilter* kf_nmpc;
    std::string solver_state_error = "Infeasible_Problem_Detected";
    std::string solver_state_success = "Solve_Succeeded";
    nav_msgs::Odometry previous_projected_pose;
    bool init_previous_pose = false;
    int pose_conuter = 0;
    int velocity_counter = 0;
   
    
  
  public:

    TrajectoryRegulator();
    ~TrajectoryRegulator() = default;

    void solver_init();
    void init(ros::NodeHandle& nh);
    void mpc_solver();
    void mpc_solver_with_collocation();
    void mpc_solver_with_multiple_shooting();
    

    typedef std::shared_ptr<TrajectoryRegulator> Ptr;
    PlanningSaving::Ptr planner_saving;
    ParamPasser::Ptr passer;
    bool early_stop =  false;

};
}  // namespace hagen_planner
#endif
