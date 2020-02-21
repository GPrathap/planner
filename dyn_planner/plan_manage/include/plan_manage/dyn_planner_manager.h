#ifndef _KGB_TRAJECTORY_GENERATOR_H_
#define _KGB_TRAJECTORY_GENERATOR_H_

#include <ros/ros.h>
#include <path_searching/kinodynamic_rrt_star.h>
#include <bspline_opt/non_uniform_bspline.h>
#include <plan_env/edt_environment.h>
#include <bspline_opt/trajectory_planning.h>

namespace dyn_planner
{
class DynPlannerManager
{
private:
  /* algorithm */
  // shared_ptr<KinodynamicRRTstar> path_finder;

  EDTEnvironment::Ptr edt_env_;

  KinodynamicRRTstar::Ptr path_finder_;

  // BsplineOptimizer::Ptr bspline_optimizer_;

  double time_sample_;
  double max_vel_;

  /* processing time */
  double time_search_ = 0.0;
  double time_optimize_ = 0.0;
  double time_adjust_ = 0.0;

  /* helper function */
  Eigen::Vector3d getFarPoint(const vector<Eigen::Vector3d>& path, Eigen::Vector3d x1, Eigen::Vector3d x2);

public:
  DynPlannerManager()
  {
  }
  ~DynPlannerManager();

  /* ---------- main API ---------- */
  /* generated traj */
  int traj_id_, dynamic_;
  double traj_duration_, t_start_, t_end_, margin_, time_start_;
  ros::Time time_traj_start_;
  Eigen::Vector3d pos_traj_start_;
  NonUniformBspline traj_pos_, traj_vel_, traj_acc_, traj_pos_alternative;
  bool is_alternative_path_exist;
  // kamaz::hagen::TrajectoryPlanning trajectory_planner;
  std::vector<Eigen::Vector3d> desired_poses;
  std::vector<Eigen::Vector3d> desired_velocities;
  /* guided optimization */
  NonUniformBspline traj_init_;
  vector<vector<Eigen::Vector3d>> guide_paths_;
  vector<Eigen::Vector3d> guide_pts_;
  int current_state = 0;

  bool generateTrajectory(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                          Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, double increase_cleareance
                          , int path_index, std::atomic_bool &is_allowed_to_run);  // front-end && back-end

  bool orthoGradReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d end_pt,
                       Eigen::Vector3d end_vel);  // gradient-based replan using orthogonal gradient

  bool guidedGradReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d end_pt,
                        Eigen::Vector3d end_vel);  // gradient-based replan using guiding points

  void retrieveTrajectory();

  void setParam(ros::NodeHandle& nh);
  void setPathFinder(const KinodynamicRRTstar::Ptr& finder);
  // void setOptimizer(const BsplineOptimizer::Ptr& optimizer);
  void setEnvironment(const EDTEnvironment::Ptr& env);

  bool checkTrajCollision(Eigen::Vector3d& intermidiate_goal
                                                    , bool& intermidiate_goal_is_set);


  /* ---------- evaluation ---------- */
  void getSolvingTime(double& ts, double& to, double& ta);
  void getCostCurve(vector<double>& cost, vector<double>& time)
  {
    // bspline_optimizer_->getCostCurve(cost, time);
  }

  typedef shared_ptr<DynPlannerManager> Ptr;
};
}  // namespace dyn_planner

#endif