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

void KinodynamicAstar::push_job(kamaz::hagen::RRTStar3D* worker) {
        ptask_t task = boost::make_shared<task_t>(boost::bind(&kamaz::hagen::RRTStar3D::rrt_planner_and_save
                    , worker));
        boost::shared_future<std::vector<kamaz::hagen::PathNode>> fut(task->get_future());
        pending_data.push_back(fut);
        // std::cout<< "Thread has been sumitted..." << std::endl;
        io_service.post(boost::bind(&task_t::operator(), task));
}

int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                             Eigen::Vector3d end_pt, Eigen::Vector3d end_v, bool init, bool dynamic, double time_start)
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
  kino_ops.dt = 0.5;
  kino_ops.max_itter = 30;
  kino_ops.ell = 20;
  kino_ops.initdt = 0.05;
  kino_ops.min_dis = 1.0;

  std::vector<Eigen::Vector2d> Q;
  Eigen::Vector2d dim_in;
  dim_in << 4, 8;
  Q.push_back(dim_in);

  start_vel_ = start_v;
  start_acc_ = start_a;

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
  rrt_planner_options.origin_ = origin_;
  rrt_planner_options.map_size_3d_ = map_size_3d_;
  int times = (number_of_paths > 0) ? number_of_paths: 1;
  rrt_avoidance_dist = (rrt_avoidance_dist > 0) ? rrt_avoidance_dist: 0.6;
  for (int i = 0; i < times; i++) {
      Eigen::VectorXd x_dimentions(6);
      x_dimentions << curr_range[0][0], curr_range[1][0], curr_range[0][1],curr_range[1][1], curr_range[0][2], curr_range[1][2];
      kamaz::hagen::SearchSpace X;
      X.init_search_space(x_dimentions, max_samples, rrt_avoidance_dist, 200);
      X.use_whole_search_sapce = true;
      X.setEnvironment(this->edt_env_);
      kamaz::hagen::RRTStar3D* rrtstart3d;
      rrt_planner_options.search_space = X;
      rrtstart3d = new kamaz::hagen::RRTStar3D();
      rrtstart3d->rrt_init(rewrite_count, rrt_planner_options, common_utils, save_data_index);
      push_job(rrtstart3d);
  }

  boost::wait_for_all((pending_data).begin(), (pending_data).end());
  bool is_path_found = false;
  smoothed_paths.clear();
  path_costs.clear();
  double lowerst_cost = 1000000;
  index_of_loweres_cost = -1;
  for(auto result : pending_data){
    std::vector<kamaz::hagen::PathNode> smoothed_path = result.get();
    int counter=0;
    if(smoothed_path.size()>2){
          smoothed_paths.push_back(smoothed_path);
          auto cost = get_distance(smoothed_path);
          path_costs.push_back(cost);
          if(lowerst_cost > cost){
              lowerst_cost = cost;
              index_of_loweres_cost = counter;
          }
          counter++;
      }   
  }
  std::cout<< "smoothed_paths size " << smoothed_paths.size() << std::endl;
  std::cout<< "index_of_loweres_cost " << index_of_loweres_cost << std::endl;
  pending_data.clear();
  if(smoothed_paths.size() > 0 && index_of_loweres_cost>-1){
    return REACH_HORIZON;
  }else{
    std::cout<< "No path found..." << std::endl;
    return NO_PATH;
  }
}

 double  KinodynamicAstar::get_distance(std::vector<kamaz::hagen::PathNode> trajectory_){
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
  nh.param("search/number_of_paths", number_of_paths, -1);
  nh.param("search/rrt_avoidance_dist", rrt_avoidance_dist, -1.0);
  nh.param("search/lqr_feasibility_max_vel", lqr_feasibility_max_vel, -1.0);
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
  
  service_work = boost::make_unique<boost::asio::io_service::work>(io_service);
  std::cout<< "Number of threads that can support this system: " << boost::thread::hardware_concurrency() << std::endl;
  for (int i = 0; i < boost::thread::hardware_concurrency(); ++i)
  {
    threads.create_thread(boost::bind(&boost::asio::io_service::run,
      &io_service));
  }
  std::cout<< "Thread pool has been initialized..." << boost::thread::hardware_concurrency() << std::endl;

  
  // std::cout<< "Number of threads that can support this system: " << boost::thread::hardware_concurrency() << std::endl;
  // for (int i = 0; i < boost::thread::hardware_concurrency(); ++i)
  // {
  //   threads.create_thread(boost::bind(&boost::asio::io_service::run,
  //     &io_service));
  // }
  // std::cout<< "Thread pool has been initialized..." << boost::thread::hardware_concurrency() << std::endl;
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
  for(auto path_i : smoothed_paths){
    path_list.push_back(getRRTTraj(0, path_i));
  }
  return path_list;
}

std::vector<Eigen::Vector3d> KinodynamicAstar::getRRTTraj(double delta_t, std::vector<kamaz::hagen::PathNode> smoothed_path){
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

Eigen::MatrixXd KinodynamicAstar::getSamplesRRT(double& ts, int& K)
{
  //  kamaz::hagen::TrajectoryPlanning trajectory_planner(2);
  /* ---------- final trajectory time ---------- */
  if(smoothed_paths.size() == 0 || index_of_loweres_cost < 0){
    Eigen::MatrixXd samples(3, 1);
    return samples;
  }
  auto selected_path = smoothed_paths[index_of_loweres_cost];
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
