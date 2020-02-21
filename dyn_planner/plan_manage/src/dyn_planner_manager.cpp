#include <plan_manage/dyn_planner_manager.h>

#include <fstream>

namespace dyn_planner
{
DynPlannerManager::~DynPlannerManager()
{
  traj_id_ = 0;
}

void DynPlannerManager::setParam(ros::NodeHandle& nh)
{
  nh.param("manager/time_sample", time_sample_, -1.0);
  nh.param("manager/max_vel", max_vel_, -1.0);
  nh.param("manager/dynamic", dynamic_, -1);
  nh.param("manager/margin", margin_, -1.0);
}

void DynPlannerManager::setPathFinder(const KinodynamicRRTstar::Ptr& finder)
{
  path_finder_ = finder;
}

void DynPlannerManager::setEnvironment(const EDTEnvironment::Ptr& env)
{
  edt_env_ = env;
}

bool DynPlannerManager::checkTrajCollision(Eigen::Vector3d& intermidiate_goal, bool& intermidiate_goal_is_set)
{
  /* check collision */
  for (double t = t_start_; t <= t_end_; t += 0.02)
  {
    Eigen::Vector3d pos = traj_pos_.evaluateDeBoor(t);
    double second_dist = edt_env_->get_free_distance(pos);
    if (second_dist < margin_)
    {
      std::cout<< pos.transpose() << "------" << "-----" << second_dist << "-----"  << std::endl;
      return false;
    }
  }
  return true;
}

void DynPlannerManager::retrieveTrajectory()
{
  traj_vel_ = traj_pos_.getDerivative();
  traj_acc_ = traj_vel_.getDerivative();

  traj_pos_.getTimeSpan(t_start_, t_end_);
  pos_traj_start_ = traj_pos_.evaluateDeBoor(t_start_);
  traj_duration_ = t_end_ - t_start_;
  traj_id_ += 1;
}

void DynPlannerManager::getSolvingTime(double& ts, double& to, double& ta)
{
  ts = time_search_;
  to = time_optimize_;
  ta = time_adjust_;
}

bool DynPlannerManager::generateTrajectory(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc
                                                    , Eigen::Vector3d end_pt, Eigen::Vector3d end_vel
                                                    , double increase_cleareance, int path_index, std::atomic_bool &is_allowed_to_run)
{
  std::cout << "[planner]: -----------------------" << std::endl;
  cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << ", " << start_acc.transpose()
       << "\ngoal:" << end_pt.transpose() << ", " << end_vel.transpose() << endl;

  if ((start_pt - end_pt).norm() < 0.2)
  {
    cout << "Close goal" << endl;
    return false;
  }

  time_traj_start_ = ros::Time::now();
  time_start_ = -1.0;
  ros::Time t1, t2;
  t1 = ros::Time::now();
  /* ---------- search kino path ---------- */
  path_finder_->reset();

  int status = path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true, is_allowed_to_run,  dynamic_, time_start_
                                      , increase_cleareance, path_index);
  current_state = status;
  if (status == KinodynamicRRTstar::NO_PATH)
  {
    cout << "[planner]: init search fail!" << endl;
    path_finder_->reset();
    status = path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false, is_allowed_to_run, dynamic_, time_start_
                                     , increase_cleareance, path_index);
    if (status == KinodynamicRRTstar::NO_PATH)
    {
      cout << "[planner]: Can't find path." << endl;
      return false;
    }
    else
    {
      cout << "[planner]: retry search success." << endl;
    }
  }
  else
  {
    cout << "[planner]: init search success." << endl;
  }

  

  t2 = ros::Time::now();

  /* ---------- bspline parameterization ---------- */
  t1 = ros::Time::now();

  int K_rrt;
  double ts_rrt = 0.1;
  Eigen::MatrixXd vel_acc;

  Eigen::MatrixXd samples_rrt = path_finder_->getSamplesRRT(ts_rrt, K_rrt);

  // cout << "ts: " << ts << endl;
  // cout << "samples_rrt:\n" << samples_rrt.transpose() << endl;
  
  // kamaz::hagen::TrajectoryPlanning trajectory_planner1;
  // trajectory_planner1.generate_ts(samples_rrt);
  // std::cout<< "Total time: " << trajectory_planner1.total_time << std::endl;
  // trajectory_planner1.traj_opt7();

  // double cstep = 0.05;
  // double time_ = 0.0;
  // double tstep = 0.01;

  // int max_iter = (int) (trajectory_planner1.total_time / cstep); 
  // std::cout<< "===============max_iter============"<< max_iter << std::endl;
  // desired_poses.clear();
  // desired_velocities.clear();
  // for (int iter =1; iter < max_iter; iter++){
  //   std::vector<Eigen::VectorXd> desired_state;
  //   // std::cout<< "=======2" << std::endl;
  //   trajectory_planner1.get_desired_state(time_+cstep, desired_state);
  //   // desired_states.push_back(desired_state);
  //   desired_poses.push_back(desired_state[0]);
  //   desired_velocities.push_back(desired_state[1]);
  //   time_ = time_ + cstep;
  // }

  // t2 = ros::Time::now();
  // t_sample = (t2 - t1).toSec();


  // Eigen::MatrixXd control_pts;
  // NonUniformBspline::getControlPointEqu3(samples, ts, control_pts);
  // NonUniformBspline init = NonUniformBspline(control_pts, 3, ts);

  Eigen::MatrixXd control_pts_rrt;
  NonUniformBspline::getControlPointEqu3(samples_rrt, ts_rrt, control_pts_rrt);
  /* ---------- time adjustment ---------- */
  // NonUniformBspline pos = NonUniformBspline(control_pts, 3, ts);
  NonUniformBspline pos = NonUniformBspline(control_pts_rrt, 3, ts_rrt);
  double tm, tmp, to, tn;
  pos.getTimeSpan(tm, tmp);
  to = tmp - tm;
  bool feasible = pos.checkFeasibility(false);
  int iter_num = 0;
  while (!feasible && ros::ok())
  {
    ++iter_num;
    feasible = pos.reallocateTime();
    if (iter_num >= 50)
      break;
  }
  cout << "[Main]: iter num: " << iter_num << endl;
  pos.getTimeSpan(tm, tmp);
  tn = tmp - tm;
  cout << "[planner]: Reallocate ratio: " << tn / to << endl;
  pos.checkFeasibility(true);
  // drawVelAndAccPro(pos);
  traj_pos_ = pos;
  return true;
}

}  // namespace dyn_planner
