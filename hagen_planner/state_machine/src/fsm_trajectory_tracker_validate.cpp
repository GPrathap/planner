
#include <state_machine/fsm_trajectory_tracker_validate.h>

namespace hagen_planner
{
void FSM_Trajectory_Tracker_Validate::init(ros::NodeHandle& nh)
{

  /* ----------init global param---------- */
  nh.param("bspline/limit_vel", NonUniformBspline::limit_vel_, -1.0);
  nh.param("bspline/limit_acc", NonUniformBspline::limit_acc_, -1.0);
  nh.param("bspline/limit_ratio", NonUniformBspline::limit_ratio_, -1.0);
  /* ---------- fsm param ---------- */
  nh.param("fsm/sampling_rate", sampling_rate, 30);
  nh.param("fsm/avoidance_distance", avoidance_distance, 0.3);
  nh.param<std::string>("fsm/data_dir", data_dir, "/home/geesara/catkin_ws/src/validate/forest_gen/octomaps/");
  nh.param<std::string>("fsm/test_poses", test_poses, "/home/geesara/catkin_ws/src/validate/forest_gen/octomaps/test.csv");
  nh.param<std::string>("fsm/evalution_script", evalution_script, "/home/geesara/catkin_ws/src/validate/forest_gen/octomaps/result.txt");

  current_wp_ = 0;
  exec_state_ = EXEC_STATE::WAIT_GOAL;
  have_trajector_ = false;

  /* ---------- init edt environment ---------- */
  sdf_map_.reset(new EDTOctoMap);
  sdf_map_->init(nh);

  edt_env_.reset(new EDTEnvironment);
  edt_env_->setMap(sdf_map_);

  bspline_utils_.reset(new BSplineUtils);
  bspline_utils_->setParam(nh);
  bspline_utils_->visualization_.reset(new PlanningVisualization(nh));

  trajectroy_tracker.reset(new TrajectoryTrackerValidate);
  trajectroy_tracker->visualization_.reset(new PlanningVisualization(nh));
  trajectroy_tracker->passer.reset(new ParamPasser(nh));
  trajectroy_tracker->planner_saving.reset(new PlanningSaving(nh));
  trajectroy_tracker->init(nh);
  trajectroy_tracker->setEnvironment(edt_env_);
  trajectroy_tracker->setTrajectoryGenerator(bspline_utils_);
  
  visualization_.reset(new PlanningVisualization(nh));

  traj_real = new boost::circular_buffer<Eigen::Vector3d>(10000);
  
  bspline_pub_ = node_.advertise<state_machine::Bspline>("/planning/bspline", 10);
  vehicle_current_pose_sub_ = node_.subscribe<nav_msgs::Odometry>("/planning/current_state", 5, &FSM_Trajectory_Tracker_Validate::currentPoseCallback, this);
  pos_cmd_pub = node_.advertise<hagen_msgs::PoseCommand>("/planning/pos_cmd", 50);

  Eigen::MatrixXd k_A(1, 1); // System dynamics matrix
  Eigen::MatrixXd k_C(1, 1); // Output matrix
  Eigen::MatrixXd k_Q(1, 1); // Process noise covariance
  Eigen::MatrixXd k_R(1, 1); // Measurement noise covariance
  Eigen::MatrixXd k_P(1, 1); // Estimate error covariance
  k_A << 1;
  k_C << 1;
  // Reasonable covariance matrices
  bool passed = trajectroy_tracker->passer->passing_matrix("covariance_matrix_for_yaw_angle_q", k_Q);
  if(!passed){
    double q_cov = 1;
    k_Q << q_cov;
  }
  passed = trajectroy_tracker->passer->passing_matrix("covariance_matrix_for_yaw_angle_r", k_R);
  if(!passed){
    double r_cov = 5000;
     k_R << r_cov;
  }

  k_P << 1;
  std::cout << "A: \n" << k_A << std::endl;
  std::cout << "C: \n" << k_C << std::endl;
  std::cout << "Q: \n" << k_Q << std::endl;
  std::cout << "R: \n" << k_R << std::endl;
  std::cout << "P: \n" << k_P << std::endl;
    //  // Construct the filter

  kf_yaw = new KalmanFilter(0, k_A, k_C, k_Q, k_R, k_P);

  Eigen::VectorXd inter_stop_pose(6);
}

void FSM_Trajectory_Tracker_Validate::currentPoseCallback(const nav_msgs::OdometryConstPtr& msg){
  current_pose = *msg;
}

void FSM_Trajectory_Tracker_Validate::waypointCallback(const nav_msgs::PathConstPtr& msg)
{
  std_msgs::Empty emt;
  stop_moving.publish(emt);
  granted_execution = false;
  trajectroy_tracker->force_terminate = true;

  waypoints_list.clear();
  if (msg->poses.size() < 0.0){
    cout<< "empty waypoints are detected.." << endl;
    return;
  }
    
  cout << "Triggered!" << endl;
  stop_pose <<  odom.pose.pose.position.x,  odom.pose.pose.position.y,  odom.pose.pose.position.z;
  stop_yaw_angle =  getYawFromQuat(odom.pose.pose.orientation);
  trigger_ = true;

  waypoints_list.push_back(stop_pose);
  cout<< "Intermediate goals poses:" << endl;
  double mininum_height = edt_env_->getMapCurrentRange()[0](2);
  for(int i=0; i<(int)msg->poses.size(); i++){
    Eigen::Vector3d wp(msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, msg->poses[i].pose.position.z);
    cout<< wp.transpose() << endl;
    if(msg->poses[i].pose.position.z< mininum_height){
      cout<< "z shuold be higher than "<< mininum_height << endl;
      waypoints_list.clear();
      return;
    }
    waypoints_list.push_back(wp);
  }
  if(waypoints_list.size()<4){
    cout<< "At least three way point is need, please tray again...!" << endl;
    waypoints_list.clear();
    return;
  }

  while(trajectroy_tracker->still_running){
    cout<< "Waiting till stopping the solver...!" << endl;
    trajectroy_tracker->force_terminate = true;
    granted_execution = false;
    usleep(5000);
  }

  have_trajector_ = bspline_utils_->generateTrajectory(waypoints_list);
  if(!have_trajector_){
    cout<< "Trajectory can not be generated, please regenerate again..." << endl;
  }
}

void FSM_Trajectory_Tracker_Validate::changeExecState(EXEC_STATE new_state, string pos_call)
{
  string state_str[2] = {"WAIT_GOAL", "EXEC_TRAJ" };
  int pre_s = int(exec_state_);
  exec_state_ = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void FSM_Trajectory_Tracker_Validate::printExecState(){
  string state_str[2] = {"WAIT_GOAL", "EXEC_TRAJ"};
  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

double FSM_Trajectory_Tracker_Validate::getYawFromQuat(const geometry_msgs::Quaternion &data){
    tf::Quaternion quat(data.x, data.y, data.z, data.w);
    tf::Matrix3x3 m(quat);
    double_t roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

std::vector<Eigen::Vector3d> FSM_Trajectory_Tracker_Validate::staticSolver() {
    trees.clear();
    trees_real.clear();
    waypoints_list.clear();
    std::vector<Eigen::Vector3d> path_poses;
    cout << "Triggered!" << endl;
  
    trigger_ = true;
    Eigen::Vector3d start_pt_ = poses_vector[0];
    Eigen::Vector3d end_pt_ = poses_vector[poses_vector.size()-1];

    if(poses_vector.size() == 2){
      waypoints_list.push_back(start_pt_);
      cout<< "Intermediate goals poses:" << endl;
      std::vector<Eigen::Vector3d> intermidiate_pose = utils.next_poses(start_pt_, end_pt_, 1.0);
      cout<< start_pt_.transpose() << endl;
      for(int i=0; i<(int)intermidiate_pose.size(); i++){
        cout<< intermidiate_pose[i].transpose() << endl;
        waypoints_list.push_back(intermidiate_pose[i]);
      }

      waypoints_list.push_back(end_pt_);
      cout<< end_pt_.transpose() << endl;
      if(waypoints_list.size()<4){
        cout<< "At least three way point is need, please tray again...!" << endl;
        waypoints_list.clear();
        return path_poses;
      }
    }else{
      for(auto waypoint : poses_vector){
        waypoints_list.push_back(waypoint);
      }
    }
    

    bool have_trajector_ = bspline_utils_->generateTrajectory(waypoints_list);
    if(!have_trajector_){
      cout<< "Trajectory can not be generated, please regenerate again..." << endl;
    }else{
      double tm, tmp;
      auto trajectory = bspline_utils_->traj_pos_;
      trajectory.getTimeSpan(tm, tmp);
      reference_trajectory.clear();
      for (double t = tm; t <= tmp; t += 0.1)
      {
        Eigen::Vector3d pt = trajectory.evaluateDeBoor(t);
        reference_trajectory.push_back(pt);
      }
    }

    trajectroy_tracker->force_terminate = false;
    trajectroy_tracker->xs = DM({end_pt_(0), end_pt_(1), end_pt_(2), 0});
    
    trajectroy_tracker->x0 = DM(4,1);
    trajectroy_tracker->x0(0,0) = start_pt_(0);
    trajectroy_tracker->x0(1,0) = start_pt_(1);
    trajectroy_tracker->x0(2,0) = start_pt_(2);
    trajectroy_tracker->x0(3,0) = 0;

    trajectroy_tracker->u0 = DM(4,1);
    trajectroy_tracker->u0(0,0) = 0;
    trajectroy_tracker->u0(1,0) = 0;
    trajectroy_tracker->u0(2,0) = 0;
    trajectroy_tracker->u0(3,0) = 0;

    path_poses = trajectroy_tracker->mpc_solver();
    
    std::cout<< "Solver is finished..." << std::endl;
    return path_poses;
}

}  // namespace hagen_planner


namespace backward
{
  backward::SignalHandling sh;
}

using namespace hagen_planner;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// using namespace dyn_planner;
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hagen_planner_trajectory_tracker_validate");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");
  FSM_Trajectory_Tracker_Validate fsm;
  fsm.init(nh);

  int init_sys = false;

  ros::Publisher pub_point_cloud, pub_trajectory, octomap_publisher_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<std::vector<string>> records;
  ifstream fin1;
  std::vector<string> row;
  string line1;

  pub_point_cloud = nh.advertise<sensor_msgs::PointCloud2> ("/plan_validate/pointcloud", 10);
  pub_trajectory = node.advertise<visualization_msgs::Marker>("/plan_validate/trajectory", 10);
  octomap_publisher_ = node.advertise<octomap_msgs::Octomap>("/plan_validate/octomap", 1, 100);

  fin1.open(fsm.test_poses);
  while(!fin1.eof()){
    fin1>>line1;
    string line, word, temp; 
    stringstream s(line1); 
    row.clear(); 
    while (getline(s, word, ',')) { 
      row.push_back(word); 
    }
    records.push_back(row);
  }
    
  int counter_record = 1;
  for(auto record : records){
      std::string pcd_path = fsm.data_dir + "forest" + record[1] + ".pcd";
      // std::cout<< pcd_path << std::endl;
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_path, *cloud) == -1) //* load the file
      {
        std::cout<< "Couldn't read file test_pcd.pcd \n" ;
        return (-1);
      }

      pcl::PointCloud<pcl::PointXYZ> cloud_inflate_vis_;
      for (int i = 0; i < (int)cloud->points.size(); i++) {
        pcl::PointXYZ p3d_inf = cloud->points[i];
        cloud_inflate_vis_.push_back(p3d_inf);
      }
    
      cloud_inflate_vis_.width = cloud_inflate_vis_.points.size();
      cloud_inflate_vis_.height = 1;
      cloud_inflate_vis_.is_dense = true;
      cloud_inflate_vis_.header.frame_id = "world";
      cloud_inflate_vis_.header.seq = 1;
      pcl_conversions::toPCL(ros::Time::now(), cloud_inflate_vis_.header.stamp);

      sensor_msgs::PointCloud2 map_inflate_vis;
      pcl::toROSMsg(cloud_inflate_vis_, map_inflate_vis);
      pub_point_cloud.publish(map_inflate_vis);

      fsm.sdf_map_->loadStaticMap(cloud);
      // std::cout << std::stod(record[2]) << " " << std::stod(record[3]) << " " << std::stod(record[4]) << " " << std::stod(record[5]) << " " << std::stod(record[6]) << " " << std::stod(record[7]) << " " << std::endl;
      
      fsm.poses_vector.clear();
      for(int p=0; p<(int)((record.size()-2)/3); p++){
        Eigen::Vector3d waypoint;
        int index = 2+p*3;
        waypoint << std::stod(record[index]), std::stod(record[index+1]), std::stod(record[index+2]);
        fsm.poses_vector.push_back(waypoint);
      }
      std::cout<<"Number of waypoints "<< fsm.poses_vector.size() << std::endl;
      if(fsm.poses_vector.size()<2){
        continue;
      }

      // auto start = fsm.poses_vector[0];
      // auto end_p = fsm.poses_vector[fsm.poses_vector.size()-1];
      // if(fsm.sdf_map_->get_free_distance(start)<1.0){
      //   std::cout << "start pose in the obstacles: " << counter_record <<std::endl;
      // }
      // if(fsm.sdf_map_->get_free_distance(end_p)<1.0){
      //   std::cout << "end pose in the obstacles: " <<counter_record <<std::endl;
      // }

      counter_record++;
      std::ofstream outfile;
      outfile.open(fsm.evalution_script, std::ios_base::app);
      const clock_t begin_time = clock();
      std::vector<Eigen::Vector3d> path = fsm.staticSolver();
      double time_diff =  double( clock () - begin_time ) /  CLOCKS_PER_SEC;
      double dis_ratio = -1;
      double projected_trajectory = 0;
      double reference_trajectory_dis = 0;
      if(fsm.reference_trajectory.size()>1){
        Eigen::Vector3d previous = fsm.reference_trajectory[0];
        for (int i = 1; (unsigned)i < fsm.reference_trajectory.size(); i++){
            double dis = std::abs((previous - fsm.reference_trajectory[i]).norm());
            previous = fsm.reference_trajectory[i];
            reference_trajectory_dis += dis;
        }
      }
      if(path.size() > 1){
        Eigen::Vector3d previous = path[0];
        for (int i = 1; (unsigned)i < path.size(); i++){
            double dis = std::abs((previous - path[i]).norm());
            previous = path[i];
            projected_trajectory += dis;
        }
        double dis_ratio = reference_trajectory_dis;
        if(dis_ratio ==0 ){
          dis_ratio = (fsm.poses_vector[0] - fsm.poses_vector[1]).norm();
        }
        dis_ratio = projected_trajectory/ dis_ratio; 
        outfile<<  time_diff << "," <<  projected_trajectory << ","<< dis_ratio <<"\n";
      }else{
        outfile<<  -1 << "," <<  -1 << ","<< -1 <<"\n";
      } 
  }
  ros::spin();
  return 0;
}

