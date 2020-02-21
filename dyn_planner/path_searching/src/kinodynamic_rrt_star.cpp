#include <path_searching/kinodynamic_rrt_star.h>
#include <sstream>

using namespace std;
using namespace Eigen;

namespace dyn_planner
{
KinodynamicRRTstar::~KinodynamicRRTstar()
{
  
}

void KinodynamicRRTstar::push_job(kamaz::hagen::RRTStar3D* worker, std::atomic_bool &is_allowed_to_run) {
  ptask_t task = boost::make_shared<task_t>(boost::bind(&kamaz::hagen::RRTStar3D::rrt_planner_and_save
              , worker, boost::ref(is_allowed_to_run)));
  boost::shared_future<std::vector<kamaz::hagen::PathNode>> fut(task->get_future());
  pending_data.push_back(fut);
  io_service.post(boost::bind(&task_t::operator(), task));
}

int KinodynamicRRTstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                             Eigen::Vector3d end_pt, Eigen::Vector3d end_v, bool init, std::atomic_bool &is_allowed_to_run, bool dynamic, double time_start
                             , double increase_cleareance, int path_index)
{
  
  std::vector<Eigen::Vector3d> curr_range = this->edt_env_->getMapCurrentRange();
  std::cout<< "x_dimentions:size " << curr_range.size() << std::endl;

  kamaz::hagen::PathNode start_pt_;
  start_pt_.state.head(3) = start_pt;
  start_pt_.state.tail(3) = start_v;
  
  kamaz::hagen::PathNode end_pt_;
  end_pt_.state.head(3) = end_pt;
  end_pt_.state.tail(3) = end_v;
  
  kamaz::hagen::CommonUtils common_utils;

  kamaz::hagen::RRTKinoDynamicsOptions kino_ops;
  kino_ops.max_vel = max_vel_;
  kino_ops.max_fes_vel = lqr_feasibility_max_vel;
  kino_ops.dt = lqr_min_dt;
  kino_ops.max_itter = 30;
  kino_ops.ell = lqr_num_of_iteration;
  kino_ops.initdt = 0.05;
  kino_ops.min_dis = lqr_min_dis;
  kino_ops.obstacle_radios = obstacle_radios;
  kino_ops.consider_obs = consider_obs;
  kino_ops.number_of_closest_obs = number_of_closest_obs;
  std::vector<Eigen::Vector2d> Q;
  Eigen::Vector2d dim_in;
  dim_in << rrt_star_steer_min, rrt_star_steer_max;
  if((end_pt - start_pt).norm()<rrt_star_steer_min){
     dim_in << rrt_star_steer_min/2.0, rrt_star_steer_min;
  }
  Q.push_back(dim_in);

  start_vel_ = start_v;
  end_vel_ = end_v;
  
  kamaz::hagen::RRTPlannerOptions rrt_planner_options;
  rrt_planner_options.x_init = start_pt_;
  rrt_planner_options.x_goal = end_pt_;
  rrt_planner_options.start_position = start_pt_;
  rrt_planner_options.init_search = true;
  rrt_planner_options.dynamic = true;
  rrt_planner_options.kino_options = kino_ops;
  rrt_planner_options.lengths_of_edges = Q;
  rrt_planner_options.max_samples = max_samples;
  rrt_planner_options.resolution = r; 
  rrt_planner_options.pro = proc;
  rrt_planner_options.horizon = horizon_;
  rrt_planner_options.origin_ = origin_;
  rrt_planner_options.map_size_3d_ = map_size_3d_;
  int times = (number_of_paths > 0) ? number_of_paths: 1;
  rrt_avoidance_dist = (rrt_avoidance_dist > 0) ? rrt_avoidance_dist: 0.6;

  double rrt_avoidance_dist_mod = rrt_avoidance_dist + increase_cleareance;
  rrt_avoidance_dist_mod = (rrt_avoidance_dist_mod < 1.0 ) ? rrt_avoidance_dist_mod: 1.0;

  int number_of_random_points_in_search_space = 200;
  Eigen::VectorXd x_dimentions(6);
  x_dimentions << curr_range[0][0], curr_range[1][0], curr_range[0][1],curr_range[1][1], curr_range[0][2], curr_range[1][2];
  std::cout<< "Dimention of map "<< x_dimentions.transpose() << ", avoidance distance"<< rrt_avoidance_dist_mod << std::endl;
  Eigen::MatrixXd covmat;
  Eigen::Vector3d center;
  Eigen::Matrix3d rotation_matrix;
  Eigen::Vector3d radious;
  Eigen::Quaternion<double> q;
  is_using_whole_space = false;
  int secs = (int)ros::Time::now().toSec();


  
  for (int i = 0; i < times; i++) {
      kamaz::hagen::SearchSpace X;
      X.init_search_space(x_dimentions, number_of_random_points_in_search_space, rrt_avoidance_dist_mod, 10);
      X.use_whole_search_sapce = is_using_whole_space;
      X.setEnvironment(this->edt_env_);
      // create_map(this->edt_env_->get_obs_map());
      if(!X.use_whole_search_sapce){
                // std::cout<< "======1" << end_pt_.state.head(3) << std::endl;
                // std::cout<< "======1" << start_pt_.state.head(3) << std::endl;
                center = (end_pt_.state.head(3) - start_pt_.state.head(3));
                // Eigen::Vector3d new_center_point(4);
                // std::cout<< "======2"<< space_min_z << std::endl;
                // covmat = Eigen::MatrixXd::Zero(3,3);
                radious[0] = (std::abs(center[0]) < 4.0) ? 4.0 : std::abs(center[0]);
                radious[1] = (std::abs(center[1]) < 4.0) ? 4.0 : std::abs(center[1]);
                radious[2] = (std::abs(center[2]) < space_min_z) ? space_min_z: std::abs(center[2]);
                // std::cout<< "======3" << std::endl;
                center = (end_pt + start_pt)/2.0;
                Eigen::Vector3d a(0,0,1);
                Eigen::Vector3d b = end_pt_.state.head(3) - start_pt_.state.head(3);
                // rotation_matrix = Eigen::MatrixXd::Identity(3,3);
                common_utils.get_roration_matrix(a, b, rotation_matrix);
                // // int max_tries = 3;
                // // int try_index = 0;
                X.generate_points(order_of_search_space, radious, center, rotation_matrix, curr_range[0][2], curr_range[1][2]);
      }
      kamaz::hagen::RRTStar3D* rrtstart3d;
      rrt_planner_options.search_space = X;
      rrtstart3d = new kamaz::hagen::RRTStar3D();
      rrtstart3d->rrt_init(rewrite_count, rrt_planner_options, common_utils, secs+i);
      push_job(rrtstart3d, is_allowed_to_run);
  }

  if(!is_using_whole_space){
    q = rotation_matrix;
    // create_marker(center, radious, q);
  }
  boost::wait_for_all((pending_data).begin(), (pending_data).end());
  kamaz::hagen::RRTStar3D rrtstart3d_procesor;
  kamaz::hagen::SearchSpace X;
  X.init_search_space(x_dimentions, max_samples, rrt_avoidance_dist_mod, 15);
  X.use_whole_search_sapce = true;
  X.setEnvironment(this->edt_env_);
  rrt_planner_options.search_space = X;
  rrtstart3d_procesor.rrt_init(rewrite_count, rrt_planner_options, common_utils, save_data_index);
  smoothed_paths.clear();
  paths_costs.clear();
  paths_costs_end_found.clear();
  double lowerst_cost_horizon = 1000000;
  index_of_loweres_cost = -1;
  index_of_alternative_cost = -1;
  int index_counter = 0;
  for(auto result : pending_data){
    std::vector<kamaz::hagen::PathNode> _path = result.get();
    if(_path.size()>1){
          smoothed_paths.push_back(_path);
          auto cost = get_distance(_path);
          bool is_horizon = _path.back().is_horizon;
          double dis_to_goal = 0;
          if(is_horizon){
            dis_to_goal = (_path.back().state.head(3) - end_pt_.state.head(3)).norm();
            cost = dis_to_goal + (_path.back().state.head(3) - start_pt_.state.head(3)).norm();
            // Eigen::Vector3d p1 = end_pt_.state.head(3) - start_pt_.state.head(3);
            // Eigen::Vector3d p2 = _path.back().state.head(3) - start_pt_.state.head(3);
            // double dot =  p1.dot(p2);
            // double angle = std::acos(dot/(p1.norm()*p2.norm()))*10.0;
            // cost = angle;
            std::tuple<double, bool, int> cost_index(cost, is_horizon, index_counter);
            paths_costs.push_back(cost_index);
          }else{
            std::tuple<double, bool, int> cost_index(cost, is_horizon, index_counter);
            paths_costs_end_found.push_back(cost_index);
          }   
          index_counter++;  
    }   
  }
  int which_desk = -1;
  int actual_index = 0;
  std::cout<< "Path costs: to the end" << std::endl;
  path_cost_indices_horizon = sort_indexes(paths_costs);
  path_cost_indices  = sort_indexes(paths_costs_end_found);
  for (auto i: path_cost_indices) {
      std::cout << std::get<0>(paths_costs_end_found[i]) << std::endl;
  }
  std::cout<< "Path costs: to the horizon" << std::endl;
  for (auto i: path_cost_indices_horizon) {
      std::cout << std::get<0>(paths_costs[i]) << std::endl;
  }
  std::cout<<  "path index .." << path_index << std::endl;
  if(path_cost_indices.size() > 0){
    if(path_index >= path_cost_indices.size()){
      path_index = path_cost_indices.size()-1;
    }
    actual_index = (int)path_cost_indices[path_index];
    index_of_loweres_cost = std::get<2>(paths_costs_end_found[actual_index]);
    which_desk = 0;
  }else if(path_cost_indices_horizon.size() > 0){
    // if(path_index >= path_cost_indices.size()){
    //   path_index = path_cost_indices.size()-1;
    // }
    actual_index = 0;
    which_desk = 1;
    index_of_loweres_cost = std::get<2>(paths_costs[(int)path_cost_indices_horizon[0]]);
  }
  std::cout<< "index_of_loweres_cost " << index_of_loweres_cost << std::endl;
  pending_data.clear();
  if(smoothed_paths.size() > 0 && index_of_loweres_cost>-1){
    std::vector<kamaz::hagen::PathNode> smoothed_path;
    rrtstart3d_procesor.get_smoothed_waypoints(smoothed_paths[index_of_loweres_cost], smoothed_path);
    smoothed_paths[index_of_loweres_cost] = smoothed_path;
    bool is_horizon = true;
    if(which_desk == 1){
      is_horizon = std::get<1>(paths_costs[actual_index]);
    }else if(which_desk == 0){
      is_horizon = std::get<1>(paths_costs_end_found[actual_index]);
    }
    if(is_horizon){
      return REACH_HORIZON;
    }
    return REACH_END;
  }else{
    std::cout<< "No path found..." << std::endl;
    return NO_PATH;
  }
}

bool KinodynamicRRTstar::get_search_space(visualization_msgs::Marker& marker){
  if(!is_using_whole_space){
    marker = search_space_marker;
    return true;
  }
  return false;
}

 void KinodynamicRRTstar::create_marker(Eigen::Vector3d center, Eigen::Vector3d radiuos
                , Eigen::Quaternion<double> q){
    search_space_marker.type = visualization_msgs::Marker::SPHERE;
    search_space_marker.action = visualization_msgs::Marker::ADD;
    search_space_marker.pose.position.x = center[0];
    search_space_marker.pose.position.y = center[1];
    search_space_marker.pose.position.z = center[2];
    search_space_marker.pose.orientation.x = q.x();
    search_space_marker.pose.orientation.y = q.y();
    search_space_marker.pose.orientation.z = q.z();
    search_space_marker.pose.orientation.w = q.w();
    search_space_marker.scale.x = radiuos[0];
    search_space_marker.scale.y = radiuos[1];
    search_space_marker.scale.z = radiuos[2];
    search_space_marker.color.a = 0.5;
    search_space_marker.color.r = 0.0;
    search_space_marker.color.g = 0.0;
    search_space_marker.color.b = 0.8;
    search_space_marker.lifetime = ros::Duration();
    return;
}

void KinodynamicRRTstar::create_map(std::vector<std::array<double, 6>> obs_map){
    obs_map_poses.markers.clear();
    int i =0;
    for(auto obs: obs_map){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = obs[0];
        marker.pose.position.y = obs[1];
        marker.pose.position.z = obs[2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = std::abs(obs[3]-obs[0]);
        marker.scale.y = std::abs(obs[3]-obs[0]);
        marker.scale.z = std::abs(obs[3]-obs[0]);
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        obs_map_poses.markers.push_back(marker);
        i++;
    }
    std::cout<< "obs_map_poses......size"<< obs_map_poses.markers.size() << std::endl;
    return;
}

bool KinodynamicRRTstar::get_obs_space(visualization_msgs::MarkerArray& marker_array){
    marker_array = obs_map_poses;
    return true;
}

double  KinodynamicRRTstar::get_distance(std::vector<kamaz::hagen::PathNode> trajectory_){
			double distance = 0.0f;
      if(trajectory_.size() < 1){
          return distance;
      }
      Eigen::Vector3d previous = trajectory_[0].state.head(3);
      for (int i = 1; (unsigned)i < trajectory_.size(); i++){
          double dis = std::abs((previous.head(3) - trajectory_[i].state.head(3)).norm());
          previous = trajectory_[i].state.head(3);
          distance += dis;
      }
      return distance;
}

void KinodynamicRRTstar::setParam(ros::NodeHandle& nh)
{
  nh.param("search/max_tau", max_tau_, -1.0);
  nh.param("search/init_max_tau", init_max_tau_, -1.0);
  nh.param("search/max_vel", max_vel_, -1.0);
  nh.param("search/max_acc", max_acc_, -1.0);
  nh.param("search/w_time", w_time_, -1.0);
  nh.param("search/horizon", horizon_, -1.0);
  nh.param("search/resolution_astar", resolution_, -1.0);
  nh.param("search/time_resolution", time_resolution_, -1.0);
  nh.param("search/lambda_heu", lambda_heu_, -1.0);
  nh.param("search/margin", margin_, -1.0);
  nh.param("search/allocate_num", allocate_num_, -1);
  nh.param("search/check_num", check_num_, -1);
  nh.param("search/number_of_paths", number_of_paths, -1);
  nh.param("search/rrt_avoidance_dist", rrt_avoidance_dist, -1.0);
  nh.param("search/lqr_feasibility_max_vel", lqr_feasibility_max_vel, -1.0);
  nh.param("search/lqr_min_dis", lqr_min_dis, -1.0);
  nh.param("search/lqr_min_dt", lqr_min_dt, -1.0);
  nh.param("search/lqr_num_of_iteration", lqr_num_of_iteration, -1);
  nh.param("search/rrt_star_steer_min", rrt_star_steer_min, -1.0);
  nh.param("search/rrt_order_of_search_space", order_of_search_space, -1);
  nh.param("search/lqr_min_dis_points", _min_dis_points, -1.0);
  
  nh.param("search/rrt_search_space_min_z", space_min_z, -1.0);
  nh.param("search/rrt_star_steer_max", rrt_star_steer_max, -1.0);
  nh.param("search/lqr_obs_radius", obstacle_radios, -1.0);
  nh.param("search/lqr_consider_obs", consider_obs, false);
  nh.param("search/lqr_number_of_closest_obs", number_of_closest_obs, -1);
  cout << "margin:" << margin_ << endl;
}

void KinodynamicRRTstar::init()
{
  this->inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;
  edt_env_->getMapRegion(origin_, map_size_3d_);
  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_3d_.transpose() << endl;
  phi_ = Eigen::MatrixXd::Identity(6, 6);
  service_work = boost::make_unique<boost::asio::io_service::work>(io_service);
  std::cout<< "Number of threads that can support this system: " << boost::thread::hardware_concurrency() << std::endl;
  for (int i = 0; i < boost::thread::hardware_concurrency(); ++i)
  {
    threads.create_thread(boost::bind(&boost::asio::io_service::run,
      &io_service));
  }
  std::cout<< "Thread pool has been initialized..." << boost::thread::hardware_concurrency() << std::endl;
}

void KinodynamicRRTstar::setEnvironment(const EDTEnvironment::Ptr& env)
{
  this->edt_env_ = env;
}

void KinodynamicRRTstar::reset()
{
}

std::vector<std::vector<Eigen::Vector3d>> KinodynamicRRTstar::getRRTTrajS(double delta_t){
  std::vector<std::vector<Eigen::Vector3d>> path_list;
  for(auto path_i : smoothed_paths){
    path_list.push_back(getRRTTraj(0, path_i));
  }
  return path_list;
}

std::vector<Eigen::Vector3d> KinodynamicRRTstar::getRRTTraj(double delta_t, std::vector<kamaz::hagen::PathNode> smoothed_path){
    std::vector<Eigen::Vector3d> state_list;
    for(auto pose : smoothed_path){
      state_list.push_back(pose.state.head(3));
    }
    if(smoothed_path.size()<3){
      for(auto pose : smoothed_path){
        state_list.push_back(pose.state.head(3));
      }
    }else{
        Eigen::MatrixXd points(3, smoothed_path.size());
        int row_index = 0;
        for(auto const way_point : smoothed_path){
            points.col(row_index) << way_point.state[0], way_point.state[1], way_point.state[2];
            row_index++;
        }
        Spline3d spline = Eigen::SplineFitting<Spline3d>::Interpolate(points, 2);
        float time_ = 0;
        int _number_of_steps = smoothed_path.size() + 100;
        for(int i=0; i<_number_of_steps; i++){
            time_ += 1.0/(_number_of_steps*1.0);
            Eigen::VectorXd values = spline(time_);
            // std::cout<< values << std::endl;
            state_list.push_back(values);
        }
    }
    return state_list;
}

Eigen::MatrixXd KinodynamicRRTstar::getSamplesRRT(double& ts, int& K)
{
  //  kamaz::hagen::TrajectoryPlanning trajectory_planner(2);
  /* ---------- final trajectory time ---------- */
  if(smoothed_paths.size() == 0 || index_of_loweres_cost < 0){
    Eigen::MatrixXd samples(3, 1);
    return samples;
  }
  auto selected_path = smoothed_paths[index_of_loweres_cost];
 
  std::vector<kamaz::hagen::PathNode> path_list;
  Eigen::Vector3d previuos_point = selected_path[0].state.head(3);
  path_list.push_back(selected_path[0]);
  for(int i=1; i<selected_path.size(); i++){
    double dis = (selected_path[i].state.head(3) - previuos_point).norm();
    if(dis > _min_dis_points){
      path_list.push_back(selected_path[i]);
    }
    previuos_point = selected_path[i].state.head(3);
  }
  int ki = path_list.size();
  Eigen::MatrixXd samples(3, ki+3);
  if(ki<2){
    return samples;
  }
  Eigen::VectorXd sx(ki), sy(ki), sz(ki);
  int sample_num = 0;
  for(auto knok : path_list){
    sx(sample_num) = knok.state[0], sy(sample_num) = knok.state[1], sz(sample_num) = knok.state[2];
    sample_num++;
  }
  K = ki;
  ts = 0.25;
  samples.block(0, 0, 1, ki) = sx.transpose();
  samples.block(1, 0, 1, ki) = sy.transpose();
  samples.block(2, 0, 1, ki) = sz.transpose();
  samples.col(ki) = start_vel_;
  samples.col(ki+1) = end_vel_;
  //TODO try to fix this
  samples.col(ki+2) << 0 , 0 , 0;
  return samples;
}

Eigen::MatrixXd KinodynamicRRTstar::getSamplesRRTAlternative(double& ts, int& K, bool& is_exist)
{
  //  kamaz::hagen::TrajectoryPlanning trajectory_planner(2);
  /* ---------- final trajectory time ---------- */
  if(smoothed_paths.size() == 0 || index_of_alternative_cost < 0){
    Eigen::MatrixXd samples(3, 1);
    is_exist = false;
    return samples;
  }
  auto selected_path = smoothed_paths[index_of_alternative_cost];
  int ki = selected_path.size();
  Eigen::MatrixXd samples(3, ki+3);
  if(ki<2){
     is_exist = false;
    return samples;
  }
  Eigen::VectorXd sx(ki), sy(ki), sz(ki);
  int sample_num = 0;
  for(auto knok : selected_path){
    sx(sample_num) = knok.state[0], sy(sample_num) = knok.state[1], sz(sample_num) = knok.state[2];
    sample_num++;
  }
  K = ki;
  ts = 0.25;
  samples.block(0, 0, 1, ki) = sx.transpose();
  samples.block(1, 0, 1, ki) = sy.transpose();
  samples.block(2, 0, 1, ki) = sz.transpose();
  samples.col(ki) = start_vel_;
  samples.col(ki+1) = end_vel_;
  //TODO try to fix this
  samples.col(ki+2) << 0 , 0 , 0;
  is_exist = true;
  return samples;
}

Eigen::Vector3i KinodynamicRRTstar::posToIndex(Eigen::Vector3d pt)
{
  Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
  return idx;
}

int KinodynamicRRTstar::timeToIndex(double time)
{
  int idx = floor((time - time_origin_) * inv_time_resolution_);
  return idx;
}

} 