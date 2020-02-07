#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
// #include "grad_spline/sdf_map.h"
#include "plan_env/edt_environment.h"
#include <boost/functional/hash.hpp>
#include "../../src/rrt_star/rrt_star_3d.h"
#include "../../src/common/search_space.h"
// #include "../../src/common/trajectory_planning.h"
#include "../../src/utils/common_utils.h"
#include <queue>
#include <Eigen/Core>
#include <unsupported/Eigen/Splines>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/make_unique.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/move/move.hpp>
#include <iostream>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <numeric>   
#include <algorithm> 

#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

using namespace visualization_msgs;
using namespace geometry_msgs;
namespace asio = boost::asio; 
using std::cout;
using std::endl;
using std::list;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace dyn_planner
{
// #define REACH_HORIZON 1
// #define REACH_END 2
// #define NO_PATH 3
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

class PathNode
{
public:
  /* -------------------- */
  Eigen::Vector3i index;
  Eigen::Matrix<double, 6, 1> state;
  double g_score, f_score;
  Eigen::Vector3d input;
  double duration;
  double time;  // dyn
  int time_idx;
  PathNode* parent;
  char node_state;
  // kamaz::hagen::SearchSpace X;
  /* -------------------- */
  PathNode()
  {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~PathNode(){};
};
typedef PathNode* PathNodePtr;

class NodeComparator
{
public:
  bool operator()(PathNodePtr node1, PathNodePtr node2) { return node1->f_score > node2->f_score; }
};

template <typename T>
struct matrix_hash : std::unary_function<T, size_t>
{
  std::size_t operator()(T const& matrix) const
  {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i)
    {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable
{
private:
  /* data */
  std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>> data_3d_;
  std::unordered_map<Eigen::Vector4i, PathNodePtr, matrix_hash<Eigen::Vector4i>> data_4d_;

public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}
  void insert(Eigen::Vector3i idx, PathNodePtr node) { data_3d_.insert(std::make_pair(idx, node)); }
  void insert(Eigen::Vector3i idx, int time_idx, PathNodePtr node)
  {
    data_4d_.insert(std::make_pair(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
  }

  PathNodePtr find(Eigen::Vector3i idx)
  {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }
  PathNodePtr find(Eigen::Vector3i idx, int time_idx)
  {
    auto iter = data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  void clear()
  {
    data_3d_.clear();
    data_4d_.clear();
  }
};

typedef Eigen::Spline<double, 3> Spline3d;
class KinodynamicRRTstar
{
private:

  
  /* ---------- main data structure ---------- */
  vector<PathNodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable expanded_nodes_;
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_set_;
  std::vector<PathNodePtr> path_nodes_;
  std::vector<kamaz::hagen::PathNode> path_rrt_;
  
  kamaz::hagen::CommonUtils common_utils;

  /* ---------- record data ---------- */
  Eigen::Vector3d start_vel_, end_vel_, start_acc_;
  Eigen::Matrix<double, 6, 6> phi_;  // state transit matrix
  // shared_ptr<SDFMap> sdf_map;
  dyn_planner::EDTEnvironment::Ptr edt_env_;
  bool is_shot_succ_ = false;
  Eigen::MatrixXd coef_shot_;
  double t_shot_;
  bool has_path_ = false;

  /* ---------- parameter ---------- */
  /* search */
  double max_tau_ = 0.25;
  double init_max_tau_ = 0.8;
  double max_vel_ = 3.0;
  double max_acc_ = 3.0;
  double w_time_ = 10.0;
  double horizon_;
  double lambda_heu_;
  double margin_;
  int allocate_num_;
  int check_num_;
  double tie_breaker_ = 1.0 + 1.0 / 10000;
  int number_of_paths = 4;
  int r = 1;
  int max_samples = 1000;
  int rewrite_count = 32;
  double proc = 0.1;
  int save_data_index = 0;
  double rrt_avoidance_dist = 0.6;
  double lqr_min_dis = 1.0;
  double lqr_min_dt = 0.1;
  double lqr_feasibility_max_vel = 0.25;
  double space_min_z = 1.0;
  int order_of_search_space = 4;
  /* map */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  Eigen::Vector3d origin_, map_size_3d_;
  double time_origin_;

  /* helper */
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
  int timeToIndex(double time);

  void push_job(kamaz::hagen::RRTStar3D* worker);

  typedef boost::packaged_task<std::vector<kamaz::hagen::PathNode>> task_t;
  typedef boost::shared_ptr<task_t> ptask_t;
  std::vector<boost::shared_future<std::vector<kamaz::hagen::PathNode>>> pending_data;
  boost::asio::io_service io_service;
  boost::thread_group threads;
  std::unique_ptr<boost::asio::io_service::work> service_work;

public:
  KinodynamicRRTstar(){};
  ~KinodynamicRRTstar();

  enum
  {
    REACH_HORIZON = 1,
    REACH_END = 2,
    NO_PATH = 3
  };

  /* main API */
  void setParam(ros::NodeHandle& nh);
  void init();
  void reset();
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
             Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool init, bool dynamic = false,
             double time_start = -1.0, double increase_cleareance = 0.0, int path_index = 0);

  void setEnvironment(const EDTEnvironment::Ptr& env);
  std::vector<Eigen::Vector3d> getRRTTraj(double delta_t, std::vector<kamaz::hagen::PathNode> smoothed_path);
  double get_distance(std::vector<kamaz::hagen::PathNode> trajectory_);
  std::vector<std::vector<Eigen::Vector3d>> getRRTTrajS(double delta_t);
  Eigen::MatrixXd getSamplesRRT(double& ts, int& K);
  std::vector<PathNodePtr> getVisitedNodes();
  bool get_search_space(visualization_msgs::Marker& marker);
  void create_marker(Eigen::Vector3d center, Eigen::Vector3d radiuos
            , Eigen::Quaternion<double> q);
  Eigen::MatrixXd getSamplesRRTAlternative(double& ts, int& K, bool& is_exist);
  std::vector<std::vector<kamaz::hagen::PathNode>> smoothed_paths;

  template <typename T> std::vector<size_t> sort_indexes(const std::vector<T> &v) {
    // initialize original index locations
    std::vector<size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);
    // sort indexes based on comparing values in v
    // using std::stable_sort instead of std::sort
    // to avoid unnecessary index re-orderings
    // when v contains elements of equal values 
    std::stable_sort(idx.begin(), idx.end(),[&v](size_t i1, size_t i2) {return v[i1] < v[i2];});
    return idx;
  }

  std::vector<double> paths_costs;
  std::vector<size_t> path_cost_indices;
  int index_of_loweres_cost = -1;
  int index_of_alternative_cost = -1;
  visualization_msgs::Marker search_space_marker;
  typedef shared_ptr<KinodynamicRRTstar> Ptr;
  bool is_using_whole_space = false;
  double rrt_star_steer_min = 4;
  double  rrt_star_steer_max = 6;
  int lqr_num_of_iteration = 20;
  double obstacle_radios = 0.4;
  bool consider_obs = true;
  int number_of_closest_obs = 10;
};

}  // namespace dyn_planner

#endif