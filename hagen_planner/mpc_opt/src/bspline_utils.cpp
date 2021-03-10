#include <mpc_opt/bspline_utils.h>

#include <fstream>

namespace hagen_planner
{
BSplineUtils::~BSplineUtils()
{
  traj_id_ = 0;
}

void BSplineUtils::setParam(ros::NodeHandle& nh)
{
  nh.param("bspline_utils/time_sample", time_sample_, -1.0);
  nh.param("bspline_utils/max_vel", max_vel_, -1.0);
  nh.param("bspline_utils/margin", margin_, -1.0);
  nh.param("bspline_utils/smoothing_factor", smoothing_factor, 100);
  nh.param("bspline_utils/smoothing_order", smoothing_order, 3);
}


void BSplineUtils::retrieveTrajectory()
{
  traj_vel_ = traj_pos_.getDerivative();
  traj_acc_ = traj_vel_.getDerivative();
  traj_pos_.getTimeSpan(t_start_, t_end_);
  pos_traj_start_ = traj_pos_.evaluateDeBoor(t_start_);
  traj_duration_ = t_end_ - t_start_;
  traj_id_ += 1;
}

void BSplineUtils::getSolvingTime(double& ts, double& to, double& ta)
{
  ts = time_search_;
  to = time_optimize_;
  ta = time_adjust_;
}

bool BSplineUtils::generateTrajectory(std::deque<Eigen::Vector3d> waypoints_list){
  std::cout << " ==== [trajectory generator]: ==== " << std::endl;
  if(waypoints_list.size()<4){
    cout << "At least 3 waypoints should be generate trajectory..." << endl;
    return false;
  }
  if ((waypoints_list[0] - waypoints_list[1]).norm() < 0.2)
  {
    cout << "Close goal, wont generate trajectory..." << endl;
    return false;
  }

  std::vector<Eigen::Vector3d> smoothed_path;
  Eigen::MatrixXd points(3, waypoints_list.size());
  int row_index = 0;
  for(auto const way_point : waypoints_list){
      points.col(row_index) << way_point[0], way_point[1], way_point[2];
      row_index++;
  }

  Eigen::Spline3d spline = Eigen::SplineFitting<Eigen::Spline3d>::Interpolate(points, smoothing_order);
  float time_ = 0;
  int _number_of_steps = waypoints_list.size() + smoothing_factor;
  for(int i=0; i<_number_of_steps; i++){
      time_ += 1.0/(_number_of_steps*1.0);
      Eigen::VectorXd values = spline(time_);
      smoothed_path.push_back(values);
  }


  int ki = smoothed_path.size();
  Eigen::MatrixXd samples(3, ki+3);
  Eigen::VectorXd sx(ki), sy(ki), sz(ki);
  int sample_num = 0;
  for(auto knok : smoothed_path){
    sx(sample_num) = knok[0], sy(sample_num) = knok[1], sz(sample_num) = knok[2];
    sample_num++;
  }
  samples.block(0, 0, 1, ki) = sx.transpose();
  samples.block(1, 0, 1, ki) = sy.transpose();
  samples.block(2, 0, 1, ki) = sz.transpose();
  samples.col(ki)<< 0, 0, 0;
  samples.col(ki+1)<< 0, 0, 0;
  samples.col(ki+2) << 0, 0 ,0;
 

  Eigen::MatrixXd control_pts;
  NonUniformBspline::getControlPointEqu3(samples, time_sample_, control_pts);
  NonUniformBspline pos = NonUniformBspline(control_pts, 3, time_sample_);
  control_points = control_pts.transpose();
  double tm, tmp, to, tn;
  pos.getTimeSpan(tm, tmp);
  to = tmp - tm;
  bool feasible = pos.checkFeasibility(false);
  int iter_num = 0;
  //TODO check here 
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
  cout << "[Planner]: Reallocate ratio: " << tn / to << endl;
  pos.checkFeasibility(true);
  // drawVelAndAccPro(pos);
  visualization_->drawBspline(pos, 0.1, Eigen::Vector4d(1.0, 0.5, 0.0, 1), true, 0.12,
                                Eigen::Vector4d(1, 0.7, 0.3, 1));
  traj_pos_ = pos;
  return true;
}

}  // namespace hagen_planner
