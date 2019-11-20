#include "rrtbase.h"

namespace kamaz {
namespace hagen {

    RRTBase::RRTBase(RRTPlannerOptions options, CommonUtils& common_util
    , std::atomic_bool &is_allowed_to_run): till_auto_mode(is_allowed_to_run){
            
            X = options.search_space;
            Q = options.lengths_of_edges;
            sample_taken = 0;
            max_samples = options.max_samples;
            r = options.resolution;
            prc = options.pro;
            x_init = options.x_init;
            x_goal = options.x_goal;
            start_position = options.start_position;
            min_angle_allows_obs = options.min_angle_allows_obs;
            add_tree();
            _obstacle_fail_safe_distance = options.obstacle_fail_safe_distance;
            common_utils = common_util;
    }

    void RRTBase::add_tree(){
        Tree tree;
        tree.init(X);
        trees[0] = tree;
    }

    void RRTBase::add_vertex(int tree, PathNode v){
        std::array<double, 3> vertex_ = {v.state[0], v.state[1], v.state[2]};
        trees[tree].V.insert_vertex(v.state.head(3));
        trees[tree].v_count += 1;
        sample_taken += 1;
        V_indices[vertex_] = v;
    }

    void RRTBase::add_edge(int tree, PathNode child, PathNode parent){
        std::array<double, 3> child_ ={child.state[0], child.state[1], child.state[2]};
        trees[tree].E[child_] = parent;
    }

    void RRTBase::printEdge(int tree){
        std::cout<<"RRTBase::printEdge===="<< std::endl;
        for(auto const&item : trees[tree].E){
            std::cout<< item.second.state.transpose() << std::endl;
            std::cout<< item.second.state.transpose() << std::endl;
        }
        std::cout<<"RRTBase::printEdge===="<< std::endl;
    }

    std::vector<Eigen::Vector3d> RRTBase::nearby_vertices(int tree, PathNode x
                                            , int max_neighbors){
        return trees[tree].V.nearest_veties(x.state.head(3), max_neighbors);
    }

    std::vector<Eigen::Vector3d> RRTBase::nearby_waypoints(int tree, PathNode x
                                            , int max_neighbors){
        return trees[tree].V.nearest_point_on_trajectory(x.state.head(3), max_neighbors);
    }

    PathNode RRTBase::get_nearest(int tree, PathNode x){
        auto veties = trees[tree].V.nearest_veties(x.state.head(3), 1);
        if(veties.size()==0){
            BOOST_LOG_TRIVIAL(warning) << FYEL("There is no any neighbors");
            PathNode temp;
            temp.state << -1, -1, -1, 0 , 0 ,0;
            return temp;
        }
        std::array<double, 3> _key = {veties[0][0], veties[0][1], veties[0][2]};
        return V_indices[_key];
    }

    bool RRTBase::check_none_vector(Eigen::Vector3d  vector){
        return (vector[0]<0)? true: false;
    }

    std::vector<PathNode> RRTBase::new_and_near(int tree, Eigen::Vector2d q){
        std::vector<PathNode> new_and_near_vec;
        auto x_ran = X.sample_free();
        PathNode x_rand;
        x_rand.state<< x_ran[0], x_ran[1], x_ran[2], 0, 0, 0;
        auto x_nearest = get_nearest(tree, x_rand);
        auto x_new = steer(x_nearest, x_rand, q[0]);
        // std::cout<<"RRTBase::new_and_near: x_rand: " << x_rand.transpose() << std::endl;
        // std::cout<<"RRTBase::new_and_near: x_nearest: " << x_nearest.transpose() << std::endl;
        // std::cout<<"RRTBase::new_and_near: x_new " << x_new.transpose() << std::endl;
        // printEdge(0);
        auto g1 = trees[0].V.obstacle_free(x_new.state.head(3));
        auto g2 = X.obstacle_free(x_new.state.head(3));
        // std::cout<<"RRTBase::new_and_near: x_new g1 g2 " << g1 << "  "<< g2 << std::endl;
        if((!g1) && (!g2)){
            return new_and_near_vec;
        }
        sample_taken += 1;
        new_and_near_vec.push_back(x_new);
        new_and_near_vec.push_back(x_nearest);
        // std::cout<<"RRTBase::new_and_near: new_and_near_vec "<< new_and_near_vec.size() <<std::endl;
        return new_and_near_vec;
    }

   PathNode RRTBase::steer(PathNode start, PathNode goal, double distance){
        Eigen::Vector3d ab = goal.state.head(3) - start.state.head(3);
        double ba_length = ab.norm();
        Eigen::Vector3d unit_vector = ab/ba_length;
        Eigen::Vector3d scaled_vector = unit_vector*distance;
        Eigen::Vector3d steered_point = start.state.head(3) + scaled_vector.head(3);
        int j = 0;
        for(int i=0; i<6; i+=2){
            if(steered_point[j] < X.dim_lengths[i]){
                steered_point[j] = X.dim_lengths[i];
            }
            if (steered_point[j] >= X.dim_lengths[i+1]){
                steered_point[j] = X.dim_lengths[i+1];
            }
            j++;
        }
        PathNode steer_point;
        steer_point.state<< steered_point[0], steered_point[1], steered_point[2], 0, 0, 0;
        return steer_point;
    }

    bool RRTBase::connect_to_point(int tree, PathNode x_a, PathNode x_b){
        // std::cout<< "RRTBase::connect_to_point: "<< x_b.transpose() << std::endl;
        // std::cout<< "RRTBase::connect_to_point: "<< x_a.transpose() << std::endl;
        auto g1 = trees[tree].V.obstacle_free(x_b.state.head(3));
        auto g2 = X.collision_free(x_a.state.head(3), x_b.state.head(3), r);
        // std::cout<< "RRTBase::connect_to_point: "<< g1 << " " << g2 << std::endl;
        if(( g1 == 1) && ( g2 == 1)){
            add_vertex(tree, x_b);
            add_edge(tree, x_b, x_a);
            return true;
        }
        // std::cout<< "RRTBase::connect_to_point not connected" << std::endl;
        return false;
    }

    bool RRTBase::can_connect_to_goal(int tree){
        auto x_nearest = get_nearest(tree, x_goal);
        // std::cout<< "RRTBase::can_connect_to_goal:x_nearest: "<< x_nearest.transpose() << std::endl;
        // std::cout<< "RRTBase::can_connect_to_goal:x_goal: "<< x_goal.transpose() << std::endl;

        auto f1 = isEdge(x_goal, tree);
        auto f2 = isEdge(x_nearest, tree);

        // std::cout<< "RRTBase::can_connect_to_goal: "<< f1 << " "<< f2 << " " << std::endl;
       
        if( f1 && f2){
            return true;
        }
        if(X.collision_free(x_nearest.state.head(3), x_goal.state.head(3), r)){
            //  std::cout<< "RRTBase::can_connect_to_goal: collision_free true"<< std::endl;
            return true;
        }
        return false;
    }

    void RRTBase::connect_to_the_goal(int tree){
        auto x_nearest = get_nearest(tree, x_goal);
        // std::cout<< "RRTBase::connect_to_the_goal" << std::endl;
        // std::cout<< x_nearest.transpose() << std::endl;
        setEdge(x_goal, x_nearest, tree);
    }

    std::vector<PathNode> RRTBase::get_path(){
        std::vector<PathNode> path;
        if(can_connect_to_goal(0)){
             BOOST_LOG_TRIVIAL(info) << FCYN("Can connect to goal");
            connect_to_the_goal(0);
            return reconstruct_path(0, x_init, x_goal);
        }
        BOOST_LOG_TRIVIAL(info) << FCYN("Could not connect to goal");
        return path;
    }

    bool RRTBase::isEdge(PathNode point, int tree){
        std::array<double, 3> _key = {point.state[0], point.state[1], point.state[2]};
        return (trees[tree].E.count(_key)) > 0 ? true : false;
    }

    PathNode RRTBase::getEdge(PathNode point, int tree){
        std::array<double, 3> _key = {point.state[0], point.state[1], point.state[2]};
        return trees[tree].E[_key];
    }

    void RRTBase::setEdge(PathNode key, PathNode value, int tree){
        std::array<double, 3> _key = {key.state[0], key.state[1], key.state[2]};
        trees[tree].E[_key] = value;
    }

    int RRTBase::sizeOfEdge(int tree){
        return trees[tree].E.size();
    }

    std::vector<PathNode> RRTBase::reconstruct_path(int tree, PathNode x_init, PathNode x_goal){
        std::vector<PathNode> path;
        path.push_back(x_goal);
        auto current = x_goal;
        // std::cout<< "RRTBase::reconstruct_path: current"<< current.transpose() << std::endl;
        // std::cout<< "RRTBase::reconstruct_path: current"<< current.transpose() << std::endl;
        if(is_equal_vectors(x_goal, x_init)){
            return path;
        }
        // printEdge(tree);
        if(isEdge(current, tree)){
            auto current_parent = getEdge(current, tree);
            // std::cout<< "RRTBase::reconstruct_path: current 1"<< current_parent.transpose() << std::endl;
            while(!is_equal_vectors(current_parent, x_init)){
                path.push_back(current_parent);
                // std::cout<< "RRTBase::reconstruct_path: current 2"<< current_parent.transpose() << std::endl;

                if(isEdge(current_parent, tree)){
                    current_parent = getEdge(current_parent, tree);
                    // std::cout<< "RRTBase::reconstruct_path: current is edge 3"<< current_parent.transpose() << std::endl;
                }else{
                    // std::cout<< "RRTBase::reconstruct_path: current: something wrong with edges"<< current_parent.transpose() << std::endl;
                    // return path;
                    break;
                }
            }
            path.push_back(x_init);
            std::reverse(path.begin(), path.end());
        }else{
            BOOST_LOG_TRIVIAL(fatal) << FRED("Something wrong with map, need to change it");
        }

        BOOST_LOG_TRIVIAL(info) << FCYN("RRTBase::reconstruct_path: size_of the path: ")<< path.size();
        return path; 
    }

    bool RRTBase::is_equal_vectors(PathNode a, PathNode b){
        return ((a.state.head(3) - b.state.head(3)).norm()< min_acceptable) ? true : false;
    }

    bool RRTBase::check_solution(std::vector<PathNode>& path){
        if ( prc > std::rand() %1 ){
            BOOST_LOG_TRIVIAL(info) << FCYN("Checking if can connect to the goal at ")<< sample_taken << FCYN(" samples");
            path = get_path();
            if(path.size()>0){
                return true;
            }
        }
        if(sample_taken >= max_samples){
           path = get_path();
           return true; 
        }
        return false;
    }

    double RRTBase::cost_to_go(PathNode a, PathNode b){
        return (a.state.head(3)-b.state.head(3)).norm();
    }

    double RRTBase::path_cost(PathNode a, PathNode b, int tree){
        double cost = 0;
        // std::cout<< "RRTBase::path_cost" << std::endl;
        // std::cout<< "RRTBase::path_cost: a" << a.transpose() << std::endl;
        // std::cout<< "RRTBase::path_cost: b" << b.transpose() << std::endl;
        auto edges = trees[tree].E;
        while(!is_equal_vectors(a, b)){
            // std::cout<< "RRTBase::path_cost:a "<< a.transpose() << "  b: "<< b.transpose() << std::endl;
            std::array<double, 3> _key = {b.state[0], b.state[1], b.state[2]};
            if(edges.count(_key)<1){
                // std::cout<< "RRTBase::path_cost:empty edge " << std::endl;
                break;
            }
            auto p = edges[_key];
            // std::cout<< "-----423:"<< p.transpose() << std::endl;
            cost += (b.state.head(3)-p.state.head(3)).norm();
            // std::cout<< "-----424"<< a << "  "<< b << std::endl;
            b = p;
        }
        // std::cout<< "RRTBase::path_cost: cost: " << cost << std::endl;
        return cost;
    }

    double RRTBase::segment_cost(PathNode a, PathNode b){
        return (a.state.head(3)-b.state.head(3)).norm();
    }

}
}