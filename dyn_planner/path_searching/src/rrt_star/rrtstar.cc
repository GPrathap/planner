#include "rrtstar.h"

namespace kamaz {
namespace hagen {

    RRTStar::RRTStar(RRTPlannerOptions options, int _rewrite_count,
                 CommonUtils& common_utils, std::atomic_bool &is_allowed_to_run) 
                : RRT(options, common_utils, is_allowed_to_run){
            rewrite_count = _rewrite_count;
            c_best = std::numeric_limits<double>::infinity();
    }
    
    std::vector<std::tuple<double, PathNode>> RRTStar::get_nearby_vertices(int tree, PathNode x_init
                                            , PathNode x_new){
        auto X_near = nearby_vertices(tree, x_new, current_rewrite_count(tree));
        std::cout<< "RRTStar::get_nearby_vertices" << X_near.size() << std::endl;
        std::vector<std::tuple<double, PathNode>> L_near;
        for(auto const x_near : X_near){
            PathNode x_near_node;
            x_near_node.state<< x_near[0], x_near[1], x_near[2], 0, 0, 0;
            auto new_s = segment_cost(x_near_node, x_new);
            auto cost = path_cost(x_init, x_near_node, tree) + new_s;
            auto pose = x_near_node;
            std::tuple<double, PathNode> a(cost, pose);
            L_near.push_back(a);
        }
        std::sort(L_near.begin(), L_near.end(),
            [](const std::tuple<double, PathNode> a, const std::tuple<double, PathNode> b){
                return std::get<0>(a) < std::get<0>(b); 
        });
        return L_near;
    }

    int RRTStar::current_rewrite_count(int tree){
        if(rewrite_count == 0){
            return trees[tree].v_count;
        }
        // std::cout<< "RRTStar::current_rewrite_count: "<< trees[tree].v_count << std::endl;
        return std::min(trees[tree].v_count, rewrite_count);
    }

    void RRTStar::rewrite(int tree, PathNode x_new, std::vector<std::tuple<double, PathNode>> L_near){
        for (auto const l_near : L_near){
            auto x_near = std::get<1>(l_near);
            auto curr_cost = path_cost(x_init, x_near, tree);
            auto tent_cost = path_cost(x_init, x_new, tree) + segment_cost(x_new, x_near);
            if((tent_cost < curr_cost) && (X.collision_free(x_near.state.head(3), x_new.state.head(3), r))){
                // std::cout<< "========RRTStar::rewrite======"<< std::endl;
                // std::cout<< x_near << std::endl;
                // std::cout<< x_new << std::endl;
                setEdge(x_near, x_new, tree);
            }
        }
    }

    void RRTStar::connect_shortest_valid(int tree, PathNode x_new, std::vector<std::tuple<double, PathNode>> L_near){
        // std::cout<< "RRTStar::connect_shortest_valid : L_near size: "<< L_near.size() << std::endl;
        for (auto const l_near : L_near){
            auto c_near = std::get<0>(l_near);
            // std::cout<<"c_near: " << c_near << std::endl;
            auto x_near = std::get<1>(l_near);
            auto cost_go = cost_to_go(x_near, x_goal);
            // std::cout<<"cost_go: " << cost_go << std::endl;
            auto is_connected = connect_to_point(tree, x_near, x_new);
            if((c_near+cost_go < c_best) && is_connected){
                break;
            }
        }
    }

    std::vector<PathNode> RRTStar::rrt_star(){
        add_vertex(0, x_init);
        PathNode none_pose;
        none_pose.state << -1, -1, -1, 0, 0, 0;
        add_edge(0, x_init, none_pose);
        std::vector<PathNode> path;
        while(true){
            for(auto const q : Q){
                for(int i=0; i<q[1]; i++){
                   if(!till_auto_mode){
                        BOOST_LOG_TRIVIAL(warning) << FYEL("Since drone is moved into manuval mode, stop finding trajectory");
                        return path;   
                   }
                   auto new_and_next = new_and_near(0, q);
                //    std::cout<< "rstar loop...." << new_and_next.size() << std::endl;
                   if(new_and_next.size()==0){
                       continue;
                   }
                   auto x_new = new_and_next[0];
                //    std::cout<< "rstar loop.... x_new" << x_new.transpose() << std::endl;
                //    if(check_none_vector(x_new)){
                //        continue;
                //    }
                //    std::cout<< "rstar loop...." << std::endl;
                   auto l_near = get_nearby_vertices(0, x_init, x_new);
                   connect_shortest_valid(0, x_new, l_near);
                   if (isEdge(x_new, 0)){
                       rewrite(0, x_new, l_near);
                   }
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