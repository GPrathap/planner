#ifndef _PLANNING_FSM_TRACKER_GROUP_H_
#define _PLANNING_FSM_TRACKER_GROUP_H_

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <traj_utils/planning_visualization.h>
#include <plan_env/edtoctomap.h>
#include <plan_env/edt_environment.h>
#include "state_machine/Bspline.h"
#include "mpc_opt/nonlinear_mpc_opt.h"
#include "mpc_opt/trajectory_tracker_validate.h"
#include <traj_utils/planning_saving.h>
#include <traj_utils/math_utils.h>
#include "nav_msgs/Odometry.h"
#include "hagen_msgs/PoseCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <geometry_msgs/TwistStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <deque>
#include <math.h>
#include <tf/tf.h>
#include <queue>
#include <mpc_opt/bspline_utils.h>
#include <state_machine/backward.hpp>
#include <boost/circular_buffer.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <atomic>

#include <stack>
#include <vector>
#include <array>
#include <iostream>
#include <cmath>
#include <set>
#include <cstdlib>
#include <climits>
#include <cmath>
#include <cerrno>
#include <cfenv>
#include <cstring>
// #include <cnpy.h>
#include<complex>
#include<ctime>
#include<cstdlib>
#include<iostream>
#include<map>
#include <traj_utils/planning_visualization.h>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <octomap_msgs/conversions.h>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <octomap/octomap.h>

#include <Eigen/Core>
#include <unsupported/Eigen/Splines>

using std::vector;

namespace hagen_planner
{
class FSM_Trajectory_Tracker_Validate
{
private:
  /* ---------- flag ---------- */
  bool trigger_, have_trajector_;
  enum EXEC_STATE
  {
    WAIT_GOAL,
    EXEC_TRAJ
  };
  EXEC_STATE exec_state_;

  void changeExecState(EXEC_STATE new_state, string pos_call);
  void printExecState();

  /* ---------- planning utils ---------- */
  

  /* ---------- parameter ---------- */
  double thresh_no_replan_, thresh_replan_;

  /* ---------- planning api ---------- */
    int current_wp_;
   bool stop_execution = false;
  /* ---------- sub and pub ---------- */
  ros::NodeHandle node_;

  ros::Timer exec_timer_, safety_timer_, cmd_timer_;
  ros::Timer vis_timer_, query_timer_;

  ros::Subscriber waypoint_sub_, odometry_sub_, rc_sub, vehicle_current_pose_sub_
  , stop_execution_sub_, continue_execution_sub_, obstacle_poses_sub_, lidar_poses_2d_sub_;

  ros::Publisher replan_pub_, bspline_pub_, wait_for_goal, stat_moving, stop_moving, pos_cmd_pub, state_pub;

  void execFSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void currentPoseCallback(const nav_msgs::OdometryConstPtr& msg);
  void waypointCallback(const nav_msgs::PathConstPtr& msg);
  double getYawFromQuat(const geometry_msgs::Quaternion &data);
  void stopExecutionCallback(const std_msgs::Empty msg);
  void continueExecutionCallback(const std_msgs::Empty msg);
  void detectedObs(const visualization_msgs::MarkerArray::ConstPtr& msg);
  void laserScan(const sensor_msgs::LaserScan::ConstPtr& scan_in);

public:
  FSM_Trajectory_Tracker_Validate(/* args */)
  {
  }
  ~FSM_Trajectory_Tracker_Validate()
  {
  }

  EDTOctoMap::Ptr sdf_map_;
  EDTEnvironment::Ptr edt_env_;
  BSplineUtils::Ptr bspline_utils_;
  PlanningVisualization::Ptr visualization_;
  TrajectoryTrackerValidate::Ptr trajectroy_tracker;

  void init(ros::NodeHandle& nh);
  bool intermidiate_goal_is_set = false;
  std::atomic_bool planner_status;

  std::string data_dir, test_poses, evalution_script;
  
  void solverThread();
  std::thread startSolverThread();

  void fsmExecutor();
  std::thread execFSMThread();

  void cmdExecutor();
  std::thread execCMDThread();

  std::vector<Eigen::Vector3d> staticSolver();

  Eigen::Vector3d stop_pose;
  vector<vector<double>> trees, trees_real;
  hagen_msgs::PoseCommand cmd;
  nav_msgs::Odometry odom;
  nav_msgs::Odometry current_pose;
  sensor_msgs::PointCloud laser_cloud;
  boost::circular_buffer<Eigen::Vector3d>* traj_real;
  bool is_allowed_for_execution = true;
  std::mutex lock_on_solver; 
  std::condition_variable condition_on_solver;
  bool granted_execution = false;
  std::thread solver_thread;
  std::thread fsm_thread;
  std::thread cmd_thread;
  bool stop_pose_init = false;
  laser_geometry::LaserProjection projector_;
  double current_yaw = 0;
  double stop_yaw_angle = 0;
  // double current_yaw = 0;
  int sampling_rate = 30;
  std::vector<Eigen::Vector3d> poses_vector;
  std::vector<Eigen::Vector3d> reference_trajectory;
  double avoidance_distance = 0.3;
  bool has_intermeditate_goal = false;
  bool had_intermeditate_goal = false;
  Eigen::Vector3d start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, inter_end_pt_;

  MathUtils utils;

  std::deque<Eigen::Vector3d> waypoints_list;

  template <typename T, typename Total, size_t N>
  class Moving_Average
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

      operator double() const { return total_ / std::min(num_samples_, N); }

    private:
      T samples_[N];
      size_t num_samples_{0};
      Total total_{0};
  };

  Moving_Average<double, double, 20> mov_fil;
  KalmanFilter* kf_yaw;
  bool init_kf_yaw =  false;
  double yaw_sign = 1;
  double previous_yaw = 0;
  // double previous_direction = 1;
};

}  // namespace hagen_planner

#endif