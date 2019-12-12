#ifndef PATH_PLANNER_RRT_TREE_RRTSTAR_H_
#define PATH_PLANNER_RRT_TREE_RRTSTAR_H_

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
#include <limits>
#include <algorithm>
#include <functional>
#include <array>
#include "rrt.h"

namespace kamaz {
namespace hagen {
        class RRTStar: public RRT{
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                RRTStar(RRTPlannerOptions options,  int rewrite_count, CommonUtils& common_utils, std::atomic_bool &is_allowed_to_run);
                ~RRTStar() = default;

                std::vector<std::tuple<double, PathNode>> get_nearby_vertices(int tree, PathNode x_init, PathNode x_new);
                int current_rewrite_count(int tree);
                void rewrite(int tree, PathNode x_new, std::vector<std::tuple<double, PathNode>> L_near);
                void connect_shortest_valid(int tree, PathNode x_new, std::vector<std::tuple<double, PathNode>> L_near);
                std::vector<PathNode> rrt_star();
                int rewrite_count;
                double c_best;
                
        };
    }
}
#endif