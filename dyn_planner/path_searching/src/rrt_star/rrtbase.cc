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
            add_tree();
            common_utils = common_util;
            dynamic = options.dynamic;
            search_init = options.init_search;
            opt = options;
            phi_ = Eigen::MatrixXd::Identity(6, 6);
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

    PathNode RRTBase::get_vertex(Eigen::Vector3d v){
        std::array<double, 3> vertex_ = {v[0], v[1], v[2]};
        if(V_indices.count(vertex_) > 0){
             return  V_indices[vertex_];
        }
        PathNode tmp;
        tmp.is_valid = false;
        return tmp;
    }

    bool RRTBase::set_seq(PathNode parent, std::vector<Eigen::MatrixXd> state_seq){
        std::array<double, 3> vertex_ = {parent.state[0], parent.state[1], parent.state[2]};
        // if(V_indices.count(vertex_) > 0){
        //     V_indices[vertex_].state_seq = state_seq;
        //     return true;
        // }else{
        //     std::cout<< "Vertex is not found: " << parent.state.head(3).transpose() << std::endl;
        // }
        return false;
    }

    std::vector<Eigen::MatrixXd> RRTBase::get_seq(PathNode parent){
        std::vector<Eigen::MatrixXd> seq_tmp;
        std::array<double, 3> vertex_ = {parent.state[0], parent.state[1], parent.state[2]};
        // if(V_indices.count(vertex_) > 0){
        //     return V_indices[vertex_].state_seq;
        // }
        return seq_tmp;
    }

    void RRTBase::add_edge(int tree, PathNode child, PathNode parent){
        std::array<double, 3> child_ ={child.state[0], child.state[1], child.state[2]};
        trees[tree].E[child_] = parent;
        // if(parent.state_seq.size()>0){
        //     set_seq(parent, parent.state_seq);
        // }
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
        // std::cout<< "RRTBase::setEdge: "<< key.state.head(3).transpose() << std::endl;
        // std::cout<< "RRTBase::setEdge: "<< key.state_seq.size() << std::endl;
        // std::cout<< "RRTBase::setEdge: val "<< value.state.head(3).transpose() << std::endl;
        // if(key.state_seq.size() <= 0){
        //     set_seq(key, key.state_seq);
        // }else{
            //TODO fix this

            // std::vector<Eigen::MatrixXd> xHit;
            // double distance = (value.state.head(3) - key.state.head(3)).norm();
            // apply_dynamics(key, value, distance, xHit);
            // set_seq(key, xHit);
            // std::cout<< "------------------seting the edge..."<< key.state.head(3).transpose() << " " << key.state_seq.size() << std::endl;
        // }
    }

    int RRTBase::sizeOfEdge(int tree){
        return trees[tree].E.size();
    }

    void RRTBase::printEdge(int tree){
        std::cout<<"RRTBase::printEdge===="<< std::endl;
        for(auto const&item : trees[tree].E){
            std::cout<< item.second.state.transpose() << std::endl;
            std::cout<< item.second.state.transpose() << std::endl;
        }
        std::cout<<"RRTBase::printEdge===="<< std::endl;
    }

    std::vector<PathNode> RRTBase::nearby_vertices(int tree, PathNode x
                                            , int max_neighbors){
        std::vector<PathNode> vertices;
        std::vector<Eigen::Vector3d> vertices_keys
                                = trees[tree].V.nearest_veties(x.state.head(3), max_neighbors);
        for(auto vertex : vertices_keys){
            auto node = get_vertex(vertex);
            if(node.is_valid){
                vertices.push_back(node);
            }
        }
        return vertices;
    }

    std::vector<Eigen::Vector3d> RRTBase::nearby_waypoints(int tree, PathNode x
                                            , int max_neighbors){
        return trees[tree].V.nearest_point_on_trajectory(x.state.head(3), max_neighbors);
    }

    PathNode RRTBase::get_nearest(int tree, PathNode x){
         std::vector<Eigen::Vector3d> veties = trees[tree].V.nearest_veties(x.state.head(3), 1);
        if(veties.size()==0){
            BOOST_LOG_TRIVIAL(warning) << FYEL("There is no any neighbors");
            PathNode temp;
            temp.state.head(3) << -1, -1, -1;
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
        x_rand.state.head(3)<< x_ran[0], x_ran[1], x_ran[2];
        // x_rand.control_input = drone_dynamics.uNominal;
        auto x_nearest = get_nearest(tree, x_rand);
        auto x_new = steer(x_nearest, x_rand, q[0]);
        std::cout<<"RRTBase::new_and_near: x_rand: " << x_rand.state.head(3).transpose() << std::endl;
        std::cout<<"RRTBase::new_and_near: x_nearest: " << x_nearest.state.head(3).transpose() << std::endl;
        std::cout<<"RRTBase::new_and_near: x_new " << x_new.state.head(3).transpose() << std::endl;
        // printEdge(0);
        if(x_new.is_valid){
            sample_taken += 1;
            new_and_near_vec.push_back(x_new);
            new_and_near_vec.push_back(x_nearest);
        }
        // std::cout<<"RRTBase::new_and_near: new_and_near_vec "<< new_and_near_vec.size() <<std::endl;
        return new_and_near_vec;
    }

    // void RRTBase::apply_dynamics(PathNode cur_node, PathNode goal, double distance, std::vector<Eigen::MatrixXd>& xHit){
    //     Eigen::Vector3d ab = goal.state.head(3) - cur_node.state.head(3);
    //     double ba_length = ab.norm();
    //     Eigen::Vector3d unit_vector = ab/ba_length;
    //     Eigen::Vector3d scaled_vector = unit_vector*distance;
    //     Eigen::Vector3d steered_point = cur_node.state.head(3) + scaled_vector.head(3);
    //     std::vector<Eigen::MatrixXd> L;
    //     std::vector<Eigen::MatrixXd> l;
    //     // std::vector<Eigen::MatrixXd> xHit;
    //     Eigen::Vector3d velocity = opt.kino_options.max_fes_vel*unit_vector;
    //     cur_node.state.block<3,1>(3,0) = velocity;
    //     cur_node.state.block<6,1>(6,0) = Eigen::MatrixXd::Zero(6,1);
    //     cur_node.state(12) = log(drone_dynamics.dt);
    //     drone_dynamics.extendedLQR(cur_node.state, drone_dynamics.uNominal, L, l, xHit, steered_point);
    // }

   PathNode RRTBase::steer(PathNode cur_node, PathNode goal, double distance){
        // Eigen::Vector3d ab = goal.state.head(3) - cur_node.state.head(3);
        // double ba_length = ab.norm();
        // Eigen::Vector3d unit_vector = ab/ba_length;
        // Eigen::Vector3d scaled_vector = unit_vector*distance;
        // Eigen::Vector3d steered_point = cur_node.state.head(3) + scaled_vector.head(3);
        // std::vector<Eigen::MatrixXd> L;
        // std::vector<Eigen::MatrixXd> l;
        // std::vector<Eigen::MatrixXd> xHit;

        // Eigen::Vector3d velocity = opt.kino_options.max_vel*unit_vector;
        // cur_node.state.block<3,1>(3,0) = velocity;
        // cur_node.state.block<6,1>(6,0) = Eigen::MatrixXd::Zero(6,1);
        // cur_node.state(12) = log(drone_dynamics.dt);
        // drone_dynamics.extendedLQR(cur_node.state, drone_dynamics.uNominal, L, l, xHit, steered_point);

        // std::vector<Eigen::MatrixXd> xHit;
        // apply_dynamics(cur_node, goal, distance, xHit);
        Eigen::Vector3d ab = goal.state.head(3) - cur_node.state.head(3);
        double ba_length = ab.norm();
        Eigen::Vector3d unit_vector = ab/ba_length;
        Eigen::Vector3d scaled_vector = unit_vector*distance;
        Eigen::Vector3d steered_point = cur_node.state.head(3) + scaled_vector.head(3);
        PathNode steer_point;
        steer_point.state.head(3)<< steered_point[0], steered_point[1], steered_point[2];
        // steer_point.state.head(3)<< (xHit.back())(0), (xHit.back())(1), (xHit.back())(2);
        int j = 0;
        for(int i=0; i<6; i+=2){
            if(steer_point.state(j) < X.dim_lengths[i]){
                steer_point.state(j) = X.dim_lengths[i];
            }
            if (steer_point.state(j) >= X.dim_lengths[i+1]){
                steer_point.state(j) = X.dim_lengths[i+1];
            }
            j++;
        }
        // steer_point.state_seq = xHit;
        // steer_point.input_seq = xHit;
        // bool is_set = set_seq(cur_node, xHit);
        // std::cout<< "state sequence is set for " << cur_node.state.head(3).transpose() << " " << cur_node.state_seq.size() << std::endl;
        auto g1 = trees[0].V.obstacle_free(steer_point.state.head(3), -1);
        auto g2 = X.obstacle_free(steer_point.state.head(3), -1);
        // std::cout<<"RRTBase::steer: current " <<  cur_node.state.transpose() << std::endl;
        // std::cout<<"RRTBase::steer: xHit " <<  xHit.size() << std::endl;
        // std::cout<<"RRTBase::steer: steered " <<  steered_point.transpose() << std::endl;
        if((!g1) && (!g2)){
            steer_point.is_valid = false;
        }
        return steer_point;
    }

    void RRTBase::stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1,
                                    Eigen::Vector3d um, double tau)
    {
        for (int i = 0; i < 3; ++i)
            phi_(i, i + 3) = tau;

        Eigen::Matrix<double, 6, 1> integral;
        integral.head(3) = 0.5 * pow(tau, 2) * um;
        integral.tail(3) = tau * um;
        state1 = phi_ * state0 + integral;
    }

    bool RRTBase::connect_to_point(int tree, PathNode x_a, PathNode x_b){
        // std::cout<< "RRTBase::connect_to_point: "<< x_b.transpose() << std::endl;
        // std::cout<< "RRTBase::connect_to_point: "<< x_a.state.head(3).transpose() << std::endl;
        auto dis = (x_a.state.head(3)-x_b.state.head(3)).norm();
        // std::cout<< "RRTBase::connect_to_point distance: " <<  dis << std::endl;
        if(isEdge(x_b, 0)){
            // BOOST_LOG_TRIVIAL(info) << FRED("RRTBase::connect_to_point: acrylic is not allowed");
            return false;
        }
        if(dis < 0.05){
            // BOOST_LOG_TRIVIAL(info) << FRED("RRTBase::connect_to_point: two points are in the same location");
            return false;
        }

        auto g1 = trees[tree].V.obstacle_free(x_b.state.head(3), -1);
        auto g2 = X.collision_free(x_a.state.head(3), x_b.state.head(3), r, -1);
        // std::cout<< "RRTBase::connect_to_point: "<< g1 << " " << g2 << std::endl;

        if((g1 == 1) && (g2 == 1)){
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
        // std::vector<Eigen::MatrixXd> xHit;
        // double distance = (x_goal.state.head(3) - x_nearest.state.head(3)).norm();

        if( f1 && f2){
            // apply_dynamics(x_nearest, x_goal, distance, xHit);
            // set_seq(x_nearest, xHit);
            return true;
        }
        if(X.collision_free(x_nearest.state.head(3), x_goal.state.head(3), r, -1.0)){
            //  std::cout<< "RRTBase::can_connect_to_goal: collision_free true"<< std::endl;
            return true;
        }
        if((x_init.state.head(3)-x_nearest.state.head(3)).norm() > opt.horizon){
            BOOST_LOG_TRIVIAL(info) << FRED("Horizon is met, not going to plan rest of it");
            x_goal.is_horizon = true;
            connect_to_the_goal(0);
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
        // BOOST_LOG_TRIVIAL(info) << FCYN("Could not connect to goal");
        return path;
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
                // current_parent.state_seq = get_seq(current_parent);
                path.push_back(current_parent);
                // std::cout<< "RRTBase::reconstruct_path: "<< current_parent.state.head(3).transpose() << std::endl;
                // std::cout<< "RRTBase::reconstruct_path: seq"<< seq.size() << std::endl;
                if(isEdge(current_parent, tree)){
                    current_parent = getEdge(current_parent, tree);
                    // std::cout<< "RRTBase::reconstruct_path: current is edge 3"<< current_parent.transpose() << std::endl;
                }else{
                    // std::cout<< "RRTBase::reconstruct_path: current: something wrong with edges"<< current_parent.transpose() << std::endl;
                    // return path;
                    break;
                }
            }
            // auto seq = get_seq(x_init);
            // x_init.state_seq = get_seq(x_init);
            // std::cout<< "RRTBase::reconstruct_path: "<< x_init.state.head(3).transpose() << std::endl;
            // std::cout<< "RRTBase::reconstruct_path: seq"<< seq.size() << std::endl;
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
            // BOOST_LOG_TRIVIAL(info) << FCYN("Checking if can connect to the goal at ")<< sample_taken << FCYN(" samples");
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
        // std::cout<< "RRTBase::path_cost: a" << a.state.head(3).transpose() << std::endl;
        // std::cout<< "RRTBase::path_cost: b" << b.state.head(3).transpose() << std::endl;
        auto edges = trees[tree].E;
        while(!is_equal_vectors(a, b)){
            // std::cout<< "RRTBase::path_cost:a "<< a.state.head(3).transpose() << "  b: "<< b.state.head(3).transpose() << std::endl;
            std::array<double, 3> _key = {b.state[0], b.state[1], b.state[2]};
            if(edges.count(_key)<1){
                // std::cout<< "RRTBase::path_cost:empty edge " << std::endl;
                break;
            }
            auto p = edges[_key];

            // std::cout<< "-----423:"<< p.state.head(3).transpose() << std::endl;
            auto current_cost = (b.state.head(3)-p.state.head(3)).norm();
            // std::cout<< "-----423:current_cost: "<< current_cost << std::endl;
            if(current_cost < 0.05){
                // std::cout<< "Two edges are on the same location"<< std::endl;
                break;
            }
            cost += current_cost;
            b = p;
        }
        // std::cout<< "RRTBase::path_cost: cost: " << cost << std::endl;
        return cost;
    }

    double RRTBase::segment_cost(PathNode a, PathNode b){
        return (a.state.head(3)-b.state.head(3)).norm();
    }

    double RRTBase::get_cost_of_path(std::vector<PathNode> path1){
        int size_of_path = path1.size();
        Eigen::Vector3d path1_dis(size_of_path);
        for(int i=0; i< path1.size(); i++){
            path1_dis[i] = path1[i].state.head(3).norm();
        }
        Eigen::MatrixXd smoothed_map  = Eigen::MatrixXd::Zero(size_of_path, size_of_path);
        for(int i=0; i<size_of_path-1; i++){
        smoothed_map(i,i) = 2;
        smoothed_map(i,i+1) = smoothed_map(i+1,i) = -1;
        }
        smoothed_map(size_of_path-1, size_of_path-1) = 2;
        return path1_dis.transpose()*smoothed_map*path1_dis;
    }

}
}