#ifndef _NON_LINEAR_MPC_OPT_
#define _NON_LINEAR_MPC_OPT_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <vector>
#include <memory>
#include <casadi/casadi.hpp>
#include <plan_env/edt_environment.h>
#include <traj_utils/planning_visualization.h>
#include <boost/circular_buffer.hpp>
#include <tf/tf.h>
#include <time.h>
#include "kalman.h"


using namespace casadi;
using namespace std;

namespace hagen_planner
{
class NonLinearMPCOpt
{
private:
  

public:
  int n_states;
  int n_controls;
  std::map<std::string, DM> args, res, args_mhe, res_mhe;
  SXDict nlp_prob;
  Dict opts;

  SX states;
  SX controls;
  SX rhs;
  Function f;
  SX g;
  SX X;
  vector<vector<double>> obs_map;
  int obs_length;
  SX obj = 0;


  EDTEnvironment::Ptr edt_env_;
  nav_msgs::Odometry current_projected_pose;
  ros::Publisher pos_current_pos_pub;
  ros::Subscriber odometry_sub_ref_;
  double maximum_acceptable_error = 0.4;
  int simulated_duration = 40;
  nav_msgs::Odometry odom;
  Eigen::Vector3d previos_pose;
  static double limit_vel_, limit_acc_, limit_ratio_;

  NonLinearMPCOpt();
  ~NonLinearMPCOpt() = default;

  void init(ros::NodeHandle& nh);
  void solver_init();
  tuple<double, SX, SX> shift(double T, double t0, SX x0, SX u, Function f);
  void setEnvironment(const EDTEnvironment::Ptr& env);
  double getYawFromQuat(const geometry_msgs::Quaternion &data);
  void getCollocationPoints(std::vector<double>& B, std::vector<std::vector<double>>& C, std::vector<double>& D);
  shared_ptr<vector<double>> vehicle_current_state;
  ros::NodeHandle node_;



  double delta_t = 0.2;
  int prediction_horizon = 5;
  int estimation_horizon = 5;
  double robot_diam = 0.3;
  double v_max = 0.4;
  double v_min = -v_max;
  double omega_max = pi/4;
  double omega_min = -omega_max;
  vector<double> map_dim = {-6, 6, -6, 6, 0, 6};
  double avoidance_distance = 0.7;
  bool still_running = false;
  int fluctuation_length = 5;
  int has_intermediate_goal =  false;
  double max_fluctuatio_allowed = 1.4;
  double obs_max_allowed_length = 10.0;
  double obs_min_allowed_length = 0.2;
  int collocation_degree = 3;
  bool use_collocation = false;

  Eigen::Vector3d origin_, map_size_;
  Eigen::Vector3d min_range_, max_range_;  // map range in pos
  std::vector<std::vector<double>> C;
  std::vector<double> D;
  std::vector<double> B;

  
  
  PlanningVisualization::Ptr visualization_;

  bool force_terminate = false;
  bool need_intermediate_goal = false;
  
  DM x0;
  DM xs;
  DM u0;

  template <typename T, typename Total, size_t N>
  class Cumulative_Sum
  {
    public:
      void operator()(T sample)
      {
          if (num_samples_ < N)
          {
              samples_[num_samples_++] = sample;
              total_ += sample;
          }
          else
          {
              T& oldest = samples_[num_samples_++ % N];
              total_ += sample - oldest;
              oldest = sample;
          }
      }

      // operator double() const { return total_/ std::min(num_samples_, N); }
      operator double() const { return total_; }

    private:
      T samples_[N];
      size_t num_samples_{0};
      Total total_{0};
  };

};
}  // namespace hagen_planner
#endif
