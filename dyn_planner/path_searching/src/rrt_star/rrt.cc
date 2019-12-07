#include "rrt.h"

namespace kamaz {
namespace hagen {
    
    RRT::RRT(RRTPlannerOptions options, 
                    CommonUtils& common_utils, std::atomic_bool &is_allowed_to_run)
        :RRTBase(options, common_utils, is_allowed_to_run){ }
    
    std::vector<PathNode> RRT::rrt_search(){
        add_vertex(0, x_init);
        PathNode none_pose;
        none_pose.state.head(3) << -1, -1, -1;
        add_edge(0, x_init, none_pose); //TODO need to handle this proper way setting null pointer
        std::vector<PathNode> path;
        while(true){
            for(auto const q : Q){
                for(int i=0; i<q[1]; i++){
                   auto new_and_next = new_and_near(0, q);
                //    if(check_none_vector(new_and_next[0])){
                //        continue;
                //    }
                   connect_to_point(0, new_and_next[1], new_and_next[0]);
                   auto solution = check_solution(path);
                   if(solution){
                       return path;
                   }
                }
            }
        }
    }
}
}