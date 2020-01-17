#include <ros/ros.h>
#include "plan_manage/Bspline.h"
#include "bspline_opt/non_uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geometry_msgs/TwistStamped.h>
#include <deque>


using namespace dyn_planner;

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

Moving_Average<double, double, 10> mov_fil;
Eigen::Vector3d starting_pose;
ros::Publisher state_pub, pos_cmd_pub, traj_pub, pos_pub, vel_pub;
int yaw_angle_smoothing_window_size = 20;
nav_msgs::Odometry odom;

quadrotor_msgs::PositionCommand cmd;
// double pos_gain[3] = {5.7, 5.7, 6.2};
// double vel_gain[3] = {3.4, 3.4, 4.0};
double pos_gain[3] = {5.7, 5.7, 6.2};
double vel_gain[3] = {3.4, 3.4, 4.0};

bool receive_traj = false;
vector<NonUniformBspline> traj;
ros::Time time_traj_start;
ros::Time time_stop_start;
Eigen::Vector3d stop_pose;
Eigen::Vector3d stop_velocity;
bool stop_pose_is_set = false;
int traj_id;
double traj_duration;
double traj_duration_for_stopping;
double t_cmd_start, t_cmd_end;
double target_yaw_angle = 0.0;
double current_yaw = 0.0;
bool stop_moving = true;
vector<Eigen::Vector3d> traj_cmd, traj_real;
std::deque<double> yaw_angle_changes;
std::deque<double> velocity_regulator_on_x;
std::deque<double> velocity_regulator_on_y;
std::deque<double> velocity_regulator_on_z;

Eigen::Vector3d hover_pt;


std::deque<double> linspace(double start_in, double end_in, double number_of_steps)
{
  std::deque<double> linspaced;
  double start = start_in;
  double end = end_in;
  double num = number_of_steps;
  if (num == 0) {
        return linspaced; 
  }
  if (num == 1)
  {
      linspaced.push_back(start);
      return linspaced;
  }
  double delta = (end - start) / (num - 1);
  for(int i=0; i < num-1; ++i)
  {
      linspaced.push_back(start + delta * i);
  }
  linspaced.push_back(end);
  return linspaced;
}

void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution,
                          Eigen::Vector4d color, int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;

  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void drawState(Eigen::Vector3d pos, Eigen::Vector3d vec, int id,
               Eigen::Vector4d color) {
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;
  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;
  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);
  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2) ;
  mk_state.points.push_back(pt);
  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);
  state_pub.publish(mk_state);
}

void bsplineCallback(plan_manage::BsplineConstPtr msg) {
  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }

  Eigen::MatrixXd ctrl_pts(msg->pts.size(), 3);
  for (int i = 0; i < msg->pts.size(); ++i) {
    Eigen::Vector3d pt;
    pt(0) = msg->pts[i].x;
    pt(1) = msg->pts[i].y;
    pt(2) = msg->pts[i].z;
    ctrl_pts.row(i) = pt.transpose();
  }

  NonUniformBspline bspline(ctrl_pts, msg->order, 0.1);
  bspline.setKnot(knots);

  time_traj_start = msg->start_time;
  traj_id = msg->traj_id;

  traj.clear();
  traj.push_back(bspline);
  traj.push_back(traj[0].getDerivative());
  traj.push_back(traj[1].getDerivative());

  traj[0].getTimeSpan(t_cmd_start, t_cmd_end);
  traj_duration = t_cmd_end - t_cmd_start;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - time_traj_start).toSec();

  Eigen::Vector3d pos, vel, acc;
  starting_pose = traj[0].evaluateDeBoor(t_cmd_start + t_cur);
  Eigen::Vector3d end_pose = traj[0].evaluateDeBoor(t_cmd_end);
  Eigen::Vector3d normalized_vector = (end_pose-starting_pose).normalized();
  target_yaw_angle = std::atan2(normalized_vector[1], normalized_vector[0]);
  yaw_angle_changes = linspace(current_yaw, target_yaw_angle, yaw_angle_smoothing_window_size);
  ROS_INFO_STREAM("bsplineCallback: "<< stop_pose_is_set );
  receive_traj = true;
  stop_moving = false;
  stop_pose_is_set = false;
}

void replanCallback(std_msgs::Empty msg) {
  /* reset duration */
  const double time_out = 0.25;
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - time_traj_start).toSec() + time_out;
  traj_duration = min(t_stop, traj_duration);
  t_cmd_end = t_cmd_start + traj_duration;
}

void stopCallback(std_msgs::Empty msg){
  stop_moving = true;
  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - time_traj_start).toSec();
  if(!stop_pose_is_set){
      if(receive_traj){
        if (t_cur >= traj_duration) {
          stop_pose = traj[0].evaluateDeBoor(t_cmd_end);
          // stop_velocity = traj[1].evaluateDeBoor(t_cmd_end);
           stop_velocity = Eigen::Vector3d(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);
        }else{
          stop_pose = traj[0].evaluateDeBoor(t_cmd_start + t_cur);
          // stop_velocity = traj[1].evaluateDeBoor(t_cmd_start + t_cur);
          stop_velocity = Eigen::Vector3d(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);
        }
      }else{
          stop_pose = Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
          stop_velocity = Eigen::Vector3d(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);
      }
      velocity_regulator_on_x = linspace(stop_velocity[0], 0.0, traj_duration_for_stopping);
      velocity_regulator_on_y = linspace(stop_velocity[1], 0.0, traj_duration_for_stopping);
      velocity_regulator_on_z = linspace(stop_velocity[2], 0.0, traj_duration_for_stopping);
      stop_pose_is_set = true;
      time_stop_start = ros::Time::now();
   }
}

void odomCallbck(const nav_msgs::Odometry& msg) {
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;

  odom = msg;
  traj_real.push_back(Eigen::Vector3d(odom.pose.pose.position.x,
                                      odom.pose.pose.position.y,
                                      odom.pose.pose.position.z));
  if (traj_real.size() > 10000)
    traj_real.erase(traj_real.begin(), traj_real.begin() + 1000);
}

void visCallback(const ros::TimerEvent& e) {
  displayTrajWithColor(traj_real, 0.03, Eigen::Vector4d(0.925, 0.054, 0.964, 1), 1);
  displayTrajWithColor(traj_cmd, 0.03, Eigen::Vector4d(1, 1, 0, 1), 2);
}

void cmdCallback(const ros::TimerEvent& e) {
  /* no publishing before receive traj */
  if (!receive_traj) return;
  Eigen::Vector3d pos, vel, acc;
  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - time_traj_start).toSec();
  
  if (stop_moving) {
   
    // ROS_INFO_STREAM("Stop pose: "<< stop_pose.transpose());
    // ROS_INFO_STREAM("Stop velocity: "<< stop_velocity.transpose());
    Eigen::Vector3d projected_pose;
    if(!velocity_regulator_on_x.empty()){
      Eigen::Vector3d next_velocity(velocity_regulator_on_x.front(), velocity_regulator_on_y.front(), velocity_regulator_on_z.front());
      velocity_regulator_on_x.pop_front();
      velocity_regulator_on_y.pop_front();
      velocity_regulator_on_z.pop_front();
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - time_stop_start).toSec();
      projected_pose = stop_pose + t_cur*(next_velocity - stop_velocity);
      stop_pose = projected_pose;
      stop_velocity = next_velocity;
      pos = projected_pose;
      vel = next_velocity;
    }else{
      pos = stop_pose;
      vel.setZero();
    }
    acc.setZero();
  }else if (t_cur < traj_duration && t_cur >= 0.0) {
    pos = traj[0].evaluateDeBoor(t_cmd_start + t_cur);
    vel = traj[1].evaluateDeBoor(t_cmd_start + t_cur);
    acc = traj[2].evaluateDeBoor(t_cmd_start + t_cur);
    // Eigen::Vector3d normalized_vector = vel.normalized();
    // double yaw_angle = std::atan2(normalized_vector[1], normalized_vector[0]);
    // if((starting_pose-pos).norm()>1.0){
    //     mov_fil(yaw_angle);
    // }
    // cmd.yaw = mov_fil;
  } else if (t_cur >= traj_duration) {
    /* hover when finish traj */
    pos = traj[0].evaluateDeBoor(t_cmd_end);
    vel.setZero();
    acc.setZero();
  } else {
    cout << "[Traj server]: invalid time." << endl;
  }
  
  if(yaw_angle_changes.empty()){
    current_yaw = target_yaw_angle;
  }else{
    current_yaw = yaw_angle_changes.front();
    yaw_angle_changes.pop_front();
  }

  if(std::abs(current_yaw - target_yaw_angle)< 0.1){
    current_yaw = target_yaw_angle;
  }

  cmd.yaw = current_yaw;
  cmd.yaw_dot = yaw_angle_smoothing_window_size;
  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  pos_cmd_pub.publish(cmd);

  drawState(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
  drawState(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));

  traj_cmd.push_back(pos);
  if (pos.size() > 10000)
    traj_cmd.erase(traj_cmd.begin(), traj_cmd.begin() + 1000);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");
   
  nh.param("traj/yaw_angle_smoothing_window_size", yaw_angle_smoothing_window_size, -1);
  nh.param("traj/traj_duration_for_stopping", traj_duration_for_stopping, 10.0);
  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber replan_sub = node.subscribe("planning/replan", 10, replanCallback);
  ros::Subscriber stopping_sub = node.subscribe("planning/stop_moving", 1, stopCallback);
  ros::Subscriber odom_sub = node.subscribe("/odom_world", 50, odomCallbck);

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  state_pub = node.advertise<visualization_msgs::Marker>("planning/state", 10);
  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  pos_pub = node.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 50);
  vel_pub = node.advertise<geometry_msgs::TwistStamped> ("/mavros/setpoint_velocity/cmd_vel", 50);
  ros::Timer vis_timer = node.createTimer(ros::Duration(0.5), visCallback);
  traj_pub = node.advertise<visualization_msgs::Marker>("planning/traj", 10);

  // node.param("bspline/limit_acc", NonUniformBspline::limit_acc_, -1.0);
  // node.param("bspline/limit_ratio", NonUniformBspline::limit_ratio_, -1.0);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  ros::Duration(1.0).sleep();
  cout << "[Traj server]: ready." << endl;
  ros::spin();
  return 0;
}
