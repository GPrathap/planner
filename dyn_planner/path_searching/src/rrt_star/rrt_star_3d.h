#ifndef PATH_PLANNER_RRT_TREE_STAR3D_H_
#define PATH_PLANNER_RRT_TREE_STAR3D_H_

#include <stack>
#include <vector>
#include <array>
#include <iostream>
#include <cmath>
#include <set>
#include <cstdlib>
#include <climits>
#include <cmath>
#include <cerrno>
#include <cfenv>
#include <cstring>
#include <cnpy.h>
#include<complex>
#include<ctime>
#include<cstdlib>
#include<iostream>
#include<map>
#include<string>
#include "../utils/common_utils.h"
#include "rrtstar.h"
#include "rrtbase.h"
#include "../common/ExtendedLQR.h"

namespace kamaz {
namespace hagen {
        class RRTStar3D {
            public:
               EIGEN_MAKE_ALIGNED_OPERATOR_NEW
               RRTStar3D() = default;
               ~RRTStar3D() = default;

               std::vector<PathNode> rrt_planner(std::atomic_bool &is_allowed_to_run);
               std::vector<PathNode> rrt_planner_and_save();

               void rrt_init(int rewrite_count, RRTPlannerOptions planner_options
                ,CommonUtils& common_utils, int index);
                    
               Eigen::Vector3d get_search_space_dim(Eigen::Vector3d dim);
               std::vector<SearchSpace::Rect> get_obstacles();
               std::vector<SearchSpace::Rect> get_random_obstacles(int number_of_obstacles
               , Eigen::VectorXd x_dimentions, PathNode x_init, PathNode x_goal);
               void save_edges(std::map<int, Tree> trees, std::string file_name);
               void save_obstacle(std::vector<SearchSpace::Rect> obstacles, std::string file_name);
               void save_poses(PathNode start, PathNode end, std::string file_name);
               void save_path(std::vector<PathNode> path, std::string file_name);
               void save_long_path(std::vector<PathNode> path, std::string file_name);
               void save_trajectory(std::vector<PathNode> trajectory_of_drone);
               double get_distance(std::vector<PathNode> trajectory_);
               Eigen::VectorXd proceding_position(Eigen::VectorXd start_position, Eigen::VectorXd end_position
                , double distance);
               Eigen::VectorXd next_position(Eigen::VectorXd start_position, Eigen::VectorXd end_position
                , double distance);
               void apply_dynamics_smoothing(Eigen::VectorXd x_start, Eigen::VectorXd x_goal
                                                            , std::vector<PathNode>& smoothed_path);
               void get_smoothed_waypoints(std::vector<PathNode> path
                                                    , std::vector<PathNode>& smoothed_path);
               std::vector<Eigen::Vector3d> next_poses(Eigen::VectorXd start_position, Eigen::VectorXd end_position
                        , double distance);

               void add_waypoints_on_straight_line(Eigen::VectorXd x_start, Eigen::VectorXd x_goal
                                                            , std::vector<PathNode>& smoothed_path);

            private:
               std::vector<Eigen::Vector2d> lengths_of_edges;
               int _max_samples;
               int resolution; 
               double pro;
               int _rewrite_count;
               std::string stotage_location;
               RRTPlannerOptions planner_opts;
               CommonUtils common_utils;
               int index;
        };
    }
}
#endif