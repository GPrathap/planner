
#include <plan_manage/planning_fsm.h>

namespace dyn_planner
{
void PlanningFSM::init(ros::NodeHandle& nh)
{
  /* ---------- init global param---------- */
  nh.param("bspline/limit_vel", NonUniformBspline::limit_vel_, -1.0);
  nh.param("bspline/limit_acc", NonUniformBspline::limit_acc_, -1.0);
  nh.param("bspline/limit_ratio", NonUniformBspline::limit_ratio_, -1.0);

  /* ---------- fsm param ---------- */
  nh.param("fsm/flight_type", flight_type_, -1);
  nh.param("fsm/thresh_replan", thresh_replan_, -1.0);
  nh.param("fsm/thresh_no_replan", thresh_no_replan_, -1.0);
  nh.param("fsm/wp_num", wp_num_, -1);
  nh.param("fsm/max_tries_for_path_finding", MAX_TRIES_FOR_FIND_PATH, -1);

  for (int i = 0; i < wp_num_; i++)
  {
    nh.param("fsm/wp" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/wp" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/wp" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  current_wp_ = 0;
  exec_state_ = EXEC_STATE::INIT;
  have_goal_ = false;

  /* ---------- init edt environment ---------- */
  sdf_map_.reset(new SDFMap);
  sdf_map_->init(nh);

  edt_env_.reset(new EDTEnvironment);
  edt_env_->setMap(sdf_map_);

  path_finder_.reset(new KinodynamicRRTstar);
  path_finder_->setParam(nh);
  path_finder_->setEnvironment(edt_env_);
  path_finder_->init();

  planner_manager_.reset(new DynPlannerManager);
  planner_manager_->setParam(nh);
  planner_manager_->setPathFinder(path_finder_);
  planner_manager_->setEnvironment(edt_env_);

  visualization_.reset(new PlanningVisualization(nh));

  /* ---------- callback ---------- */
  exec_timer_ = node_.createTimer(ros::Duration(0.01), &PlanningFSM::execFSMCallback, this);

  safety_timer_ = node_.createTimer(ros::Duration(0.1), &PlanningFSM::safetyCallback, this);

  waypoint_sub_ = node_.subscribe("/waypoint_generator/waypoints", 1, &PlanningFSM::waypointCallback, this);

  replan_pub_ = node_.advertise<std_msgs::Empty>("planning/replan", 10);
  wait_for_goal = node_.advertise<std_msgs::Empty>("planning/wait_for_goal", 10);
  stat_moving = node_.advertise<std_msgs::Empty>("planning/start_moving", 10);
  stop_moving = node_.advertise<std_msgs::Empty>("planning/stop_moving", 10);
  bspline_pub_ = node_.advertise<plan_manage::Bspline>("planning/bspline", 10);
  odometry_sub_ = node_.subscribe<nav_msgs::Odometry>("/odom_world", 50, &PlanningFSM::odomCallback, this); 
}

void PlanningFSM::odomCallback(const nav_msgs::OdometryConstPtr& msg){
  if (msg->child_frame_id == "X" || msg->child_frame_id == "O")
    return;
}


void PlanningFSM::waypointCallback(const nav_msgs::PathConstPtr& msg)
{
  if (msg->poses[0].pose.position.z < 0.0)
    return;

  cout << "Triggered!" << endl;
  trigger_ = true;

  if (flight_type_ == FLIGHT_TYPE::MANUAL_GOAL)
  {
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
  }
  else if (flight_type_ == FLIGHT_TYPE::PRESET_GOAL)
  {
    end_pt_(0) = waypoints_[current_wp_][0];
    end_pt_(1) = waypoints_[current_wp_][1];
    end_pt_(2) = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % wp_num_;
  }

  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();
  have_goal_ = true;

  if (exec_state_ == WAIT_GOAL)
    changeExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
    changeExecState(REPLAN_TRAJ, "TRIG");
}

void PlanningFSM::changeExecState(EXEC_STATE new_state, string pos_call)
{
  string state_str[5] = { "INIT", "WAIT_GOAL", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  int pre_s = int(exec_state_);
  exec_state_ = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void PlanningFSM::printExecState()
{
  string state_str[5] = { "INIT", "WAIT_GOAL", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

void PlanningFSM::execFSMCallback(const ros::TimerEvent& e)
{
  static int fsm_num = 0;
  static int avoid_if_cant_calculate = 0;
  fsm_num++;
  if (fsm_num == 100)
  {
    printExecState();
    if (!edt_env_->odomValid())
      cout << "PlanningFSM: no odom." << endl;
    if (!edt_env_->mapValid())
      cout << "PlanningFSM : no map." << endl;
    if (!trigger_)
      cout << "wait for goal." << endl;
    fsm_num = 0;
  }
  switch (exec_state_)
  {
    case INIT:
    {
      if (!edt_env_->odomValid())
      {
        return;
      }
      // if (!edt_env_->mapValid())
      // {
      //   return;
      // }
      if (!trigger_)
      {
        return;
      }
      changeExecState(WAIT_GOAL, "FSM");
      break;
    }

    case WAIT_GOAL:
    {
      std_msgs::Empty emt;
      wait_for_goal.publish(emt);
      stop_moving.publish(emt);
      change_path_index = 0;
      if (!have_goal_){
        // std::cout<< "Wait for goal..." << std::endl;
        return;
      }
      else
      {
        changeExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      std_msgs::Empty emt;
      stop_moving.publish(emt);
      nav_msgs::Odometry odom = edt_env_->getOdom();
      start_pt_(0) = odom.pose.pose.position.x;
      start_pt_(1) = odom.pose.pose.position.y;
      start_pt_(2) = odom.pose.pose.position.z;

      // start_vel_(0) = odom.twist.twist.linear.x;
      // start_vel_(1) = odom.twist.twist.linear.y;
      // start_vel_(2) = odom.twist.twist.linear.z;
      start_vel_.setZero();
      start_acc_.setZero();
      path_finder_->setEnvironment(edt_env_);
      bool success = planSearchOpt();
      if (success)
      {
        // Eigen::Vector3d intermidiate_goal;
        // bool intermidiate_goal_is_set = false;
        // bool safe = planner_manager_->checkTrajCollision(intermidiate_goal, intermidiate_goal_is_set);
        // if (!safe)
        // {
        //   change_path_index++;
        //   ROS_WARN("New traj in collision: ");
        //   std::cout<< change_path_index << std::endl;
        //   increase_cleareance += 0.0;
        //   changeExecState(GEN_NEW_TRAJ, "FSM");
        // }else{
        //   change_path_index = 0;
        //   changeExecState(EXEC_TRAJ, "FSM");
        // }
        // std::cout<< "starting..." << std::endl;
        std_msgs::Empty emt;
        stat_moving.publish(emt);
        changeExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        if(MAX_TRIES_FOR_FIND_PATH == avoid_if_cant_calculate){
          have_goal_ = false;
          avoid_if_cant_calculate = 0;
          // std_msgs::Empty emt;
          // stop_moving.publish(emt);
          changeExecState(WAIT_GOAL, "FSM");
        }else{
          avoid_if_cant_calculate++;
          changeExecState(GEN_NEW_TRAJ, "FSM");
        }
      }
      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - planner_manager_->time_traj_start_).toSec();
      t_cur = min(planner_manager_->traj_duration_, t_cur);
      Eigen::Vector3d pos = planner_manager_->traj_pos_.evaluateDeBoor(planner_manager_->t_start_ + t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > planner_manager_->traj_duration_ - 1e-2)
      {
        cout << "-------- planner_manager_->traj_duration_ "<< planner_manager_->traj_duration_ << endl;
        have_goal_ = false;
        std_msgs::Empty emt;
        stop_moving.publish(emt);
        changeExecState(WAIT_GOAL, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < thresh_no_replan_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((planner_manager_->pos_traj_start_ - pos).norm() < thresh_replan_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      std_msgs::Empty emt;
      stop_moving.publish(emt);
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - planner_manager_->time_traj_start_).toSec();
      // start_pt_ = planner_manager_->traj_pos_.evaluateDeBoor(planner_manager_->t_start_ + t_cur);
      // start_vel_ = planner_manager_->traj_vel_.evaluateDeBoor(planner_manager_->t_start_ + t_cur);
      nav_msgs::Odometry odom = edt_env_->getOdom();
      // TODO reset the velocity to zero as well
      start_pt_(0) = odom.pose.pose.position.x;
      start_pt_(1) = odom.pose.pose.position.y;
      start_pt_(2) = odom.pose.pose.position.z;

      // start_vel_(0) = odom.twist.twist.linear.x;
      // start_vel_(1) = odom.twist.twist.linear.y;
      // start_vel_(2) = odom.twist.twist.linear.z;
      start_vel_.setZero();
      start_acc_.setZero();

      cout << "t_cur: " << t_cur << endl;
      // cout << "start pt: " << start_pt_.transpose() << endl;

      /* inform server */
      // std_msgs::Empty replan_msg;
      // replan_pub_.publish(replan_msg);
      path_finder_->setEnvironment(edt_env_);
      bool success = planSearchOpt();
      if (success)
      {
        increase_cleareance = 0;
        changeExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        // have_goal_ = false;
        // changeExecState(WAIT_GOAL, "FSM");
        increase_cleareance += 0.0;
        changeExecState(GEN_NEW_TRAJ, "FSM");
      }

      //  Eigen::Vector3d intermidiate_goal;
      //   bool intermidiate_goal_is_set = false;
      //   bool safe = planner_manager_->checkTrajCollision(intermidiate_goal, intermidiate_goal_is_set);
      //   if (!safe)
      //   {
      //     change_path_index++;
      //     ROS_WARN("Generated traj in collision: ");
      //     std::cout<< change_path_index << std::endl;
      //     increase_cleareance += 0.0;
      //     changeExecState(GEN_NEW_TRAJ, "FSM");
      //   }else{
      //     change_path_index = 0;
      //     changeExecState(EXEC_TRAJ, "FSM");
      //   }
      // }
      break;
    }
  }
}

void PlanningFSM::safetyCallback(const ros::TimerEvent& e)
{
  planner_manager_->setEnvironment(edt_env_); 
  /* ---------- check trajectory ---------- */
  if (exec_state_ == EXEC_STATE::EXEC_TRAJ)
  {
    Eigen::Vector3d intermidiate_goal;
    bool intermidiate_goal_is_set = false;
    bool safe = planner_manager_->checkTrajCollision(intermidiate_goal, intermidiate_goal_is_set);
    if (!safe)
    {
      change_path_index++;
      ROS_WARN("Current traj in collision: ");
      std::cout<< change_path_index << std::endl;
      changeExecState(REPLAN_TRAJ, "SAFETY");
      return;
    }else{
      change_path_index = 0;
    }
  }
  /* ---------- check goal safety ---------- */
  if (have_goal_)
  {
    double dist =
        planner_manager_->dynamic_ ?
            edt_env_->evaluateCoarseEDT(end_pt_, planner_manager_->time_start_ + planner_manager_->traj_duration_) :
            edt_env_->evaluateCoarseEDT(end_pt_, -1.0);

    if (dist <= planner_manager_->margin_)
    {
      /* try to find a max distance goal around */
      bool new_goal = false;
      const double dr = 0.5, dtheta = 30, dz = 0.3;
      double new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;
      for (double r = dr; r <= 5 * dr + 1e-3; r += dr)
      {
        for (double theta = -90; theta <= 270; theta += dtheta)
        {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz)
          {
            new_x = end_pt_(0) + r * cos(theta / 57.3);
            new_y = end_pt_(1) + r * sin(theta / 57.3);
            new_z = end_pt_(2) + dz;
            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            dist = planner_manager_->dynamic_ ?
                       edt_env_->evaluateCoarseEDT(new_pt,
                                                   planner_manager_->time_start_ + planner_manager_->traj_duration_) :
                       edt_env_->evaluateCoarseEDT(new_pt, -1.0);
            if (dist > max_dist)
            {
              /* reset end_pt_ */
              goal(0) = new_x;
              goal(1) = new_y;
              goal(2) = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > planner_manager_->margin_)
      {
        cout << "change goal, replan." << endl;
        end_pt_ = goal;
        have_goal_ = true;
        end_vel_.setZero();
        if (exec_state_ == EXEC_TRAJ)
        {
          changeExecState(REPLAN_TRAJ, "SAFETY");
        }
        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      }
      else
      {
        // have_goal_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeExecState(WAIT_GOAL, "SAFETY");
        cout << "goal near collision, keep retry" << endl;
        changeExecState(REPLAN_TRAJ, "FSM");
        std_msgs::Empty emt;
        replan_pub_.publish(emt);
      }
    }
  }
}

bool PlanningFSM::planSearchOpt()
{

  int path_index = std::floor(change_path_index/2);
  bool plan_success = planner_manager_->generateTrajectory(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_
            , increase_cleareance, path_index);

  if (plan_success)
  {
    planner_manager_->retrieveTrajectory();

    /* publish traj */
    plan_manage::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = planner_manager_->time_traj_start_;
    bspline.traj_id = planner_manager_->traj_id_;
    Eigen::MatrixXd ctrl_pts = planner_manager_->traj_pos_.getControlPoint();
    for (int i = 0; i < ctrl_pts.rows(); ++i)
    {
      Eigen::Vector3d pvt = ctrl_pts.row(i);
      geometry_msgs::Point pt;
      pt.x = pvt(0);
      pt.y = pvt(1);
      pt.z = pvt(2);
      bspline.pts.push_back(pt);
    }
    Eigen::VectorXd knots = planner_manager_->traj_pos_.getKnot();
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    Eigen::Vector3d intermidiate_goal;
    bool intermidiate_goal_is_set = false;
    bool safe = planner_manager_->checkTrajCollision(intermidiate_goal, intermidiate_goal_is_set);
    if (!safe)
    {
      change_path_index++;
      ROS_WARN("Current traj in collision: in new trajectory generation...");
      return false;
    }

    bspline_pub_.publish(bspline);

    /* visulization */
    // vector<Eigen::Vector3d> kino_path = path_finder_->getKinoTraj(0.02);
    // visualization_->drawPath(kino_path, 0.1, Eigen::Vector4d(1, 0, 0, 1));
    
    vector<vector<Eigen::Vector3d>> rrt_paths = path_finder_->getRRTTrajS(0.02);
    int ids = 8;
    srand(time(NULL));
    for(auto rrt_path : rrt_paths){
      double r1 = ((double) rand() / (RAND_MAX));
      double r2 = ((double) rand() / (RAND_MAX));
      visualization_->drawPath(rrt_path, 0.1, Eigen::Vector4d(0, r1 ,r2, 1), ids);
      ids++;
    }
    if(rrt_paths.size()>0){
        // visualization_->drawPath(rrt_paths[path_finder_->rrtstart3d.index_of_loweres_cost], 0.1, Eigen::Vector4d(0.2, 1 ,0.5, 1), ids);
        // std::cout << "size of desired path...:"<< planner_manager_->desired_poses.size() << std::endl;
        // visualization_->drawPath(planner_manager_->desired_poses, 0.2, Eigen::Vector4d(0.6, 0.5 ,0.8, 1), 3);
        visualization_->drawBspline(planner_manager_->traj_pos_, 0.1, Eigen::Vector4d(1.0, 0.5, 0.0, 1), true, 0.12,
                                Eigen::Vector4d(1, 0.7, 0.3, 1));
        visualization_msgs::Marker marker;
        bool is_using_whole_space = path_finder_->get_search_space(marker);
        if(is_using_whole_space){
          visualization_->publish_marker(marker, 22, "seach_space");
        }
    }
    return true;
  }
  else
  {
    cout << "generate new traj fail." << endl;
    return false;
  }
}

// PlanningFSM::
}  // namespace dyn_planner
