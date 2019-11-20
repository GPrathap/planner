#ifndef PATH_PLANNER_RRT_H_
#define PATH_PLANNER_RRT_H_

#include "rrtbase.h"

namespace kamaz {
namespace hagen {
        class RRT : public RRTBase {
            public:
                RRT(RRTPlannerOptions options, 
                    CommonUtils& common_util, std::atomic_bool &is_allowed_to_run);
                
                ~RRT() = default;
                std::vector<PathNode> rrt_search();
        };
    }
}
#endif