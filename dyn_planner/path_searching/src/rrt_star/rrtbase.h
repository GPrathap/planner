#ifndef PATH_PLANNER_RRT_THREE_UTILS_H_
#define PATH_PLANNER_RRT_THREE_UTILS_H_

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
#include <atomic>
#include "../common/search_space.h"
#include "../utils/common_utils.h"
#include "tree.h"
#include <unordered_map>
#include <stddef.h>

namespace kamaz {
namespace hagen {
        struct RRTPlannerOptions {
            SearchSpace search_space;
            int sample_taken;
            int max_samples;
            std::vector<Eigen::Vector2d> lengths_of_edges;
            int resolution;
            double pro;
            double voxel_side_length;
            Eigen::Vector3d init_min_point;
            PathNode x_init;
            PathNode x_goal;
            PathNode start_position;
            double obstacle_fail_safe_distance;
            double min_acceptable;
            double min_angle_allows_obs;
        };

        template <typename C> struct Hasher{
                    typedef typename C::value_type value_type;
                    inline std::size_t operator() (const C &c) const{
                        std::size_t seed = 0;
                        // size_t h = std::hash<double>()(v[0]);
                        // hash_combine(h, v[1]);
                        // hash_combine(h, v[2]);
                        for(typename C::const_iterator it = c.begin(); it != c.end(); ++it){
                            // hash_combine<value_type>(seed, *it);
                            std::hash<value_type> hasher;
                            seed ^= hasher(*it) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                        }
                        return seed;
                    }
        };

        
        class RRTBase {
            public:
                RRTBase(RRTPlannerOptions options, 
                    CommonUtils& common_util, std::atomic_bool &is_allowed_to_run);
                ~RRTBase() = default; 

                SearchSpace X;
                CommonUtils common_utils;
                int sample_taken;
                int max_samples;
                std::vector<Eigen::Vector2d> Q;
                int r;
                double prc;
                double voxel_side_length;
                Eigen::Vector3d init_min_point;
                std::atomic_bool& till_auto_mode;
                PathNode x_init;
                PathNode x_goal;
                PathNode start_position;
                std::map<int, Tree> trees;
                Eigen::MatrixXd covmat_sub_search;
                Eigen::Vector3d center_sub_search;
                double _obstacle_fail_safe_distance = 0.5f;
                double min_acceptable = 0.01;
                double min_angle_allows_obs = 5.0f;
                std::unordered_map<std::array<double, 3>
                                    , PathNode, Hasher<std::array<double, 3>>> V_indices;

                void add_tree();
                void add_vertex(int tree, PathNode v);
                void add_edge(int tree, PathNode child, PathNode parent);
                std::vector<Eigen::Vector3d> nearby_vertices(int tree, PathNode x
                                , int max_neighbors);
                std::vector<Eigen::Vector3d> nearby_waypoints(int tree, PathNode x
                                , int max_neighbors);
                PathNode get_nearest(int tree, PathNode x);
                PathNode steer(PathNode start, PathNode goal, double distance);
                std::vector<PathNode> new_and_near(int tree, Eigen::Vector2d q);
                bool connect_to_point(int tree, PathNode x_a, PathNode x_b);
                bool can_connect_to_goal(int tree);
                bool isEdge(PathNode point, int tree);
                PathNode getEdge(PathNode point, int tree);
                std::vector<PathNode> reconstruct_path(int tree
                                        , PathNode x_init, PathNode x_goal);
                bool check_none_vector(Eigen::Vector3d  vector);
                void setEdge(PathNode key, PathNode value, int tree);
                void connect_to_the_goal(int tree);
                std::vector<PathNode> get_path();
                bool check_solution(std::vector<PathNode>& path);
                int sizeOfEdge(int tree);
                void printEdge(int tree);
                bool is_equal_vectors(PathNode a, PathNode b);
                double cost_to_go(PathNode a, PathNode b);
                double path_cost(PathNode a, PathNode b, int tree);
                double segment_cost(PathNode a, PathNode b);
        };
    }
}
#endif