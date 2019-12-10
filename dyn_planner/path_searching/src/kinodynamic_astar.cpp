#include <path_searching/kinodynamic_astar.h>
#include <sstream>

using namespace std;
using namespace Eigen;

namespace dyn_planner
{
KinodynamicAstar::~KinodynamicAstar()
{
  for (int i = 0; i < allocate_num_; i++)
  {
    delete path_node_pool_[i];
  }
}

int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                             Eigen::Vector3d end_pt, Eigen::Vector3d end_v, bool init, bool dynamic, double time_start)
{
  
  Eigen::VectorXd x_dimentions(6);
  std::vector<Eigen::Vector3d> curr_range = this->edt_env_->getMapCurrentRange();
  std::cout<< "x_dimentions:size " << curr_range.size() << std::endl;
  // x_dimentions << origin_[0], map_size_3d_[0], origin_[1], map_size_3d_[1], origin_[2], map_size_3d_[2];
  x_dimentions << curr_range[0][0], curr_range[1][0], curr_range[0][1],curr_range[1][1], curr_range[0][2], curr_range[1][2];
  std::cout<< "Map dimention " << x_dimentions << std::endl;
  
  int max_samples = 1000;
  float avoidance_width = 0.6;
  kamaz::hagen::SearchSpace X;
  int rewrite_count = 32;
  X.init_search_space(x_dimentions, max_samples, avoidance_width, 200);
  X.setEnvironment(this->edt_env_);
  // X.update_obstacles_map(obstacles);
  rrtstart3d.rrt_init(rewrite_count);
  
  Eigen::Vector3d ccc = (end_pt - start_pt).head(3);
  Eigen::MatrixXd covmat = Eigen::MatrixXd::Zero(3,3);

  covmat(0,0) = 3;
  covmat(1,1) = 3;
  covmat(2,2) = 3;
  
  Eigen::Vector3d center = (end_pt + start_pt)/2;
  Eigen::Vector3d a(1,0,0);
  Eigen::Vector3d b = ccc;

  Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity(3,3);
  int ndims = covmat.rows();       
  int save_data_index = 0;
  X.use_whole_search_sapce = true;
  X.generate_search_sapce(covmat, rotation_matrix, center, max_samples);

  kamaz::hagen::PathNode start_pt_;
  start_pt_.state.head(3) = start_pt;
  start_pt_.state.tail(3) = start_v;
  
  if (dynamic)
  {
    start_pt_.time = time_start;
    start_pt_.time_idx = timeToIndex(time_start);
  }

  kamaz::hagen::PathNode end_pt_;
  end_pt_.state.head(3) = end_pt;
  end_pt_.state.tail(3) = end_v;

  kamaz::hagen::RRTKinoDynamicsOptions kino_ops;
  kamaz::hagen::RRTPlannerOptions rrt_planner_options;

  kino_ops.init_max_tau = init_max_tau_;
  kino_ops.max_vel = 0.25;
  kino_ops.max_fes_vel = 0.25;
  kino_ops.max_acc = max_acc_;
  kino_ops.w_time = w_time_;
  kino_ops.horizon = horizon_;
  kino_ops.lambda_heu = lambda_heu_;
  kino_ops.time_resolution = time_resolution_;
  kino_ops.margin = margin_;
  kino_ops.allocate_num = allocate_num_;
  kino_ops.check_num = check_num_;
  kino_ops.start_vel_ = start_v;
  kino_ops.start_acc_ = start_a;
  kino_ops.max_tau = max_tau_;
  kino_ops.dt = 0.5;
  kino_ops.max_itter = 30;
  kino_ops.ell = 20;
  kino_ops.initdt = 0.05;
  kino_ops.min_dis = 1.0;
  
  std::atomic_bool planner_status;
  planner_status = ATOMIC_VAR_INIT(true);
  std::vector<Eigen::Vector2d> Q;
  Eigen::Vector2d dim_in;
  dim_in << 4, 8;
  Q.push_back(dim_in);
  int r = 1;
  float proc = 0.1;

  start_vel_ = start_v;
  start_acc_ = start_a;
  
  rrt_planner_options.search_space = X;
  rrt_planner_options.x_init = start_pt_;
  rrt_planner_options.x_goal = end_pt_;
  rrt_planner_options.start_position = start_pt_;
  rrt_planner_options.obstacle_fail_safe_distance = 0.5;
  rrt_planner_options.min_angle_allows_obs = 0.5;
  rrt_planner_options.init_search = true;
  rrt_planner_options.dynamic = true;
  rrt_planner_options.kino_options = kino_ops;
  rrt_planner_options.lengths_of_edges = Q;
  rrt_planner_options.max_samples = max_samples;
  rrt_planner_options.resolution = r; 
  rrt_planner_options.pro = proc;
  rrt_planner_options.origin_ = origin_;
  rrt_planner_options.map_size_3d_ = map_size_3d_;

  rrtstart3d.rrt_generate_paths(rrt_planner_options, common_utils
  , std::ref(planner_status), save_data_index, 6);

  start_vel_ = start_v;
  start_acc_ = start_a;
  std::cout<< "rrtstart3d.smoothed_paths size " << rrtstart3d.smoothed_paths.size() << std::endl;
  std::cout<< "rrtstart3d.index_of_loweres_cost " << rrtstart3d.index_of_loweres_cost << std::endl;

  if(rrtstart3d.smoothed_paths.size() >= 3 && rrtstart3d.smoothed_paths[rrtstart3d.index_of_loweres_cost].size()>0){
    return REACH_HORIZON;
  }else{
    std::cout<< "No path found..." << std::endl;
    return NO_PATH;
  }
}

void KinodynamicAstar::setParam(ros::NodeHandle& nh)
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

  cout << "margin:" << margin_ << endl;
}

void KinodynamicAstar::init()
{
  /* ---------- map params ---------- */
  this->inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;
  edt_env_->getMapRegion(origin_, map_size_3d_);

  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_3d_.transpose() << endl;

  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++)
  {
    path_node_pool_[i] = new PathNode;
  }

  phi_ = Eigen::MatrixXd::Identity(6, 6);
  use_node_num_ = 0;
  iter_num_ = 0;
}

void KinodynamicAstar::setEnvironment(const EDTEnvironment::Ptr& env)
{
  this->edt_env_ = env;
}

void KinodynamicAstar::reset()
{
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++)
  {
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
}

std::vector<std::vector<Eigen::Vector3d>> KinodynamicAstar::getRRTTrajS(double delta_t){
  std::vector<std::vector<Eigen::Vector3d>> path_list;
  for(auto path_i : rrtstart3d.smoothed_paths){
    rrtstart3d.smoothed_path = path_i;
    path_list.push_back(getRRTTraj(0));
  }
  return path_list;
}

std::vector<Eigen::Vector3d> KinodynamicAstar::getRRTTraj(double delta_t){
  std::vector<Eigen::Vector3d> state_list;
  // for(auto pose : rrtstart3d.smoothed_path){
  //    state_list.push_back(pose.state.head(3));
  // }
  if(rrtstart3d.smoothed_path.size()<3){
    for(auto pose : rrtstart3d.smoothed_path){
      state_list.push_back(pose.state.head(3));
    }
  }else{
      Eigen::MatrixXd points(3, rrtstart3d.smoothed_path.size());
      int row_index = 0;
      for(auto const way_point : rrtstart3d.smoothed_path){
          points.col(row_index) << way_point.state[0], way_point.state[1], way_point.state[2];
          row_index++;
      }
      Spline3d spline = Eigen::SplineFitting<Spline3d>::Interpolate(points, 2);
      float time_ = 0;
      int _number_of_steps = rrtstart3d.smoothed_path.size() + 100;
      for(int i=0; i<_number_of_steps; i++){
          time_ += 1.0/(_number_of_steps*1.0);
          Eigen::VectorXd values = spline(time_);
          // std::cout<< values << std::endl;
          state_list.push_back(values);
      }
  }
  return state_list;
}

Eigen::MatrixXd KinodynamicAstar::getSamplesRRT(double& ts, int& K)
{
  //  kamaz::hagen::TrajectoryPlanning trajectory_planner(2);
  /* ---------- final trajectory time ---------- */
  if(rrtstart3d.smoothed_paths.size() == 0 || rrtstart3d.index_of_loweres_cost < 0){
    Eigen::MatrixXd samples(3, 1);
    return samples;
  }
  auto selected_path = rrtstart3d.smoothed_paths[rrtstart3d.index_of_loweres_cost];
  int ki = selected_path.size();
  Eigen::MatrixXd samples(3, ki+3);
  if(ki<2){
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
  return samples;
}

Eigen::Vector3i KinodynamicAstar::posToIndex(Eigen::Vector3d pt)
{
  Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
  // idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) - origin_(1)) * inv_resolution_),
  // floor((pt(2) - origin_(2)) * inv_resolution_);
  return idx;
}

int KinodynamicAstar::timeToIndex(double time)
{
  int idx = floor((time - time_origin_) * inv_time_resolution_);
}

}  // namespace dyn_planner
