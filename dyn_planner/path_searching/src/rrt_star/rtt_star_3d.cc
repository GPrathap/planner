#include "rrt_star_3d.h"

namespace kamaz {
namespace hagen {

    std::vector<PathNode> RRTStar3D::rrt_planner(std::atomic_bool &is_allowed_to_run){
        
        auto rrtstar = RRTStar(planner_opts, _rewrite_count
                                            , common_utils, is_allowed_to_run);
        return rrtstar.rrt_star();
    }

   std::vector<PathNode> RRTStar3D::rrt_planner_and_save(std::atomic_bool &is_allowed_to_run){
        // std::atomic_bool planner_status;
        // planner_status = ATOMIC_VAR_INIT(true);
        RRTStar rrtstar(planner_opts, _rewrite_count, common_utils, is_allowed_to_run);
        // std::ofstream outfile;
        // std::vector<PathNode> path;
        // std::cout<< "========================" << std::endl;
        // outfile.open("/dataset/rrt_old/time_stamps.txt", std::ios_base::app);
        // std::cout<< "========================" << std::endl;
        // const clock_t begin_time = clock();
        std::vector<PathNode> path = rrtstar.rrt_star();
        if(path.size()<2){
            std::cout<< "Path can not be found" << std::endl;
            return path;
        }
        // std::vector<PathNode> smoothed_path;
        // get_smoothed_waypoints(path, smoothed_path);
        // // double time_diff =  double( clock () - begin_time ) /  CLOCKS_PER_SEC;

        // std::cout<< "Size of smoothed path..."<< smoothed_path.size() << std::endl;
        // stotage_location = "/dataset/rrt_old/" + std::to_string(index) + "_";
        // save_edges(rrtstar.trees, stotage_location + "edges.npy");
        // save_obstacle(planner_opts.search_space.edt_env_->get_obs_map(), stotage_location + "obstacles.npy");
        // save_poses(planner_opts.x_init, planner_opts.x_goal, stotage_location + "start_and_end_pose.npy");
        // std::cout<< "Size of smoothed path..."<< smoothed_path.size() << std::endl;
        // save_path(smoothed_path, stotage_location + "rrt_star_dynamics_path.npy");
        // save_path(planner_opts.search_space, stotage_location + "rrt_search_space.npy");
        // save_path(path, stotage_location + "rrt_star_path.npy");
        // save_long_path(smoothed_path, stotage_location + "rrt_star_dynamics_path.npy");
        // std::cout<< "Path has been calculated..." << smoothed_path.size() << std::endl;
        // return smoothed_path;
        return path;
    }

    void RRTStar3D::save_path(SearchSpace search_space, std::string file_name){
       std::vector<double> projected_path;
       BOOST_LOG_TRIVIAL(info) << FCYN("RRTStar3D::save search space trajectory size: ") << search_space.number_of_points_in_random_tank;
       for(int i=0; i<search_space.number_of_points_in_random_tank; i++){
           projected_path.push_back((*search_space.random_points_tank).row(i)[0]);
           projected_path.push_back((*search_space.random_points_tank).row(i)[1]);
           projected_path.push_back((*search_space.random_points_tank).row(i)[2]);
       }
       cnpy::npy_save(file_name, &projected_path[0], {search_space.number_of_points_in_random_tank, 3}, "w");
    }

    double  RRTStar3D::get_distance(std::vector<PathNode> trajectory_){
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

    void RRTStar3D::get_smoothed_waypoints(std::vector<PathNode> path
                                                    , std::vector<PathNode>& smoothed_path){
        auto opts = planner_opts.kino_options;
        if(path.size() >= 3){
            Eigen::VectorXd previous_node = path[0].state.head(3);
            Eigen::VectorXd current_node = path[1].state.head(3);
            Eigen::VectorXd next_node = path[2].state.head(3);

            PathNode next_pose_node;
            next_pose_node.state.head(3) << previous_node[0], previous_node[1], previous_node[2];
            smoothed_path.push_back(next_pose_node);
            Eigen::VectorXd expected_preceding = next_position(previous_node, current_node
                            , opts.min_dis);
             Eigen::VectorXd expected_next = proceding_position(current_node, next_node
                            , opts.min_dis);
            if(path.size() == 3){
                add_waypoints_on_straight_line(previous_node, expected_preceding, smoothed_path);
                apply_dynamics_smoothing(expected_preceding, expected_next, smoothed_path);
                add_waypoints_on_straight_line(expected_next, next_node, smoothed_path);
            }else{
                for(int i=3; i <= (int)path.size(); i++){
                    // std::cout<< "previous: " << previous_node.transpose() << std::endl;
                    // std::cout<< "current_node: " << current_node.transpose() << std::endl;
                    // std::cout<< "next_node: " << next_node.transpose() << std::endl;
                    add_waypoints_on_straight_line(previous_node, expected_preceding, smoothed_path);
                    // std::cout<< "expected_next: " << expected_next.transpose() << std::endl;
                    // std::cout<< "expected_preceding: " << expected_preceding.transpose() << std::endl;
                    apply_dynamics_smoothing(expected_preceding, expected_next, smoothed_path);
                    previous_node = expected_next;
                    current_node = next_node;
                    if(i == (int)path.size()){
                        add_waypoints_on_straight_line(expected_next, path.back().state.head(3), smoothed_path);
                        break;
                    }
                    next_node = path[i].state.head(3);
                    expected_preceding = next_position(previous_node, current_node
                                , opts.min_dis);
                    expected_next = proceding_position(current_node, next_node
                                , opts.min_dis);
                }
            }
            PathNode last_node = path.back();
            smoothed_path.push_back(last_node);
        }else{
            smoothed_path.push_back(path[0]);
            add_waypoints_on_straight_line(path[0].state.head(3), path[1].state.head(3), smoothed_path);
            smoothed_path.push_back(path[1]);
        }
    }


    void RRTStar3D::add_waypoints_on_straight_line(Eigen::VectorXd x_start, Eigen::VectorXd x_goal
                                                            , std::vector<PathNode>& smoothed_path){
        auto opts = planner_opts.kino_options;
        std::vector<Eigen::Vector3d> poses = common_utils.next_poses(x_start, x_goal, opts.dt*opts.max_fes_vel);
        for(auto po : poses){
            PathNode next_pose_node;
            next_pose_node.state.head(3) << po[0], po[1], po[2];
            smoothed_path.push_back(next_pose_node);
        }
    }

    void RRTStar3D::apply_dynamics_smoothing(Eigen::VectorXd x_start, Eigen::VectorXd x_goal
                                                            , std::vector<PathNode>& smoothed_path){
        auto opts = planner_opts.kino_options;
        auto search_space = planner_opts.search_space;
        std::vector<Eigen::Vector3d> poses = common_utils.next_poses(x_start, x_goal, 0.2);
        loto::hagen::ExtendedLQR extendedLQR;
        if(opts.consider_obs){
            for(auto pose : poses){
                std::vector<Eigen::Vector3d> obs_poses;
                search_space.get_free_space(pose, obs_poses, opts.number_of_closest_obs);
                for(auto obs : obs_poses){
                    loto::hagen::Obstacle obs_pose;
                    double dis_to_obs = obs.norm();
                    if(dis_to_obs < 0.03){
                        continue;
                    }
                    obs = (obs/dis_to_obs)*(dis_to_obs + opts.obstacle_radios);
                    obs_pose.pos[0] = obs[0];
                    obs_pose.pos[1] = obs[1];
                    obs_pose.pos[2] = obs[2];
                    obs_pose.radius = 0.4;
                    obs_pose.dim = 2;
                    extendedLQR.obstacles.push_back(obs_pose);
                }
            }
        }
        
        std::cout<< "Number of obstacles in the space: " << extendedLQR.obstacles.size() << std::endl;
        std::vector<loto::hagen::Matrix<U_DIM, X_DIM>> L;
        std::vector<loto::hagen::Matrix<U_DIM>> l;

        extendedLQR.xGoal = loto::hagen::zero<X_DIM>();
        extendedLQR.xGoal[0] = x_goal[0];
        extendedLQR.xGoal[1] = x_goal[1];
        extendedLQR.xGoal[2] = x_goal[2];

        extendedLQR.xGoal[3] = 0;
        extendedLQR.xGoal[4] = 0;
        extendedLQR.xGoal[5] = 0;

        Eigen::Vector3d ab = x_goal.head(3) - x_start.head(3);
        double ba_length = ab.norm();
        Eigen::Vector3d unit_vector = ab/ba_length;
        Eigen::Vector3d velocity = unit_vector*opts.max_fes_vel;

        extendedLQR.xStart[0] = x_start[0];
        extendedLQR.xStart[1] = x_start[1];
        extendedLQR.xStart[2] = x_start[2];
        extendedLQR.xStart[3] = velocity[0];
        extendedLQR.xStart[4] = velocity[1];
        extendedLQR.xStart[5] = velocity[2];

        loto::hagen::Matrix<X_DIM> xStartInit = extendedLQR.xStart;
        xStartInit[X_DIM-1] = log(opts.initdt);
        double dt_lqr;
        bool dynamics_present = true;
        try{
            dt_lqr = extendedLQR.extendedLQRItr(opts.ell, xStartInit, extendedLQR.uNominal, L, l
                            , opts.max_itter);
        }catch(const std::runtime_error& error){
            dynamics_present = false;
            BOOST_LOG_TRIVIAL(warning) << FYEL("Using same waypoints...");
            
        }
        if(dynamics_present){
            loto::hagen::Matrix<X_DIM> x = extendedLQR.xStart;
            x[X_DIM-1] = log(dt_lqr);
            std::cout << "========================================" << std::endl;
            for (int t = 0; t < opts.ell; ++t) {
                x = extendedLQR.g(x, L[t]*x + l[t]);
                // std::cout << x << std::endl;
                PathNode next_pose;
                next_pose.state.head(3) << x[0], x[1], x[2];
                if(ba_length*1.0 < (next_pose.state.head(3)-x_start.head(3)).norm()){
                    // For stopping the overshoot
                    PathNode next_pose;
                    next_pose.state.head(3) = x_goal;
                    smoothed_path.push_back(next_pose);
                    std::cout<< "Stoping the overshotting .....==============>" << std::endl;
                    break;
                }
                smoothed_path.push_back(next_pose);
            }
        }else{
            // TODO list...
            PathNode next_pose;
            next_pose.state.head(3) = x_start;
            smoothed_path.push_back(next_pose);

            next_pose.state.head(3) = x_goal;
            smoothed_path.push_back(next_pose);
        }
    }

    void RRTStar3D::save_trajectory(std::vector<PathNode> trajectory_of_drone){
       std::string file_name = stotage_location + "smoothed_rrt_path.npy";
       std::vector<double> quad_status; 
       for(auto sector: trajectory_of_drone){
            // std::cout<< status.size() << std::endl;
            quad_status.push_back(sector.state[0]);
            quad_status.push_back(sector.state[1]);
            quad_status.push_back(sector.state[2]);
       }
       cnpy::npy_save(file_name, &quad_status[0], {quad_status.size()}, "w");
    }

    void RRTStar3D::rrt_init(int rewrite_count, RRTPlannerOptions planner_options
                ,CommonUtils& _common_utils, int _index){
        _rewrite_count = rewrite_count;
        planner_opts = planner_options;
        common_utils = _common_utils;
        index = _index;

    }

    Eigen::Vector3d RRTStar3D::get_search_space_dim(Eigen::Vector3d dimensions){
        Eigen::Vector3d dim_ = dimensions;
        return dim_;
    }

    std::vector<SearchSpace::Rect> RRTStar3D::get_obstacles(){
        std::vector<SearchSpace::Rect> _objects;
        _objects.push_back(SearchSpace::Rect(20, 20, 20, 40, 40, 40));
        _objects.push_back(SearchSpace::Rect(20, 20, 60, 40, 40, 80));
        _objects.push_back(SearchSpace::Rect(20, 60, 20, 40, 80, 40));
        _objects.push_back(SearchSpace::Rect(60, 60, 20, 80, 80, 40));
        _objects.push_back(SearchSpace::Rect(60, 20, 20, 80, 40, 40));
        _objects.push_back(SearchSpace::Rect(60, 20, 60, 80, 40, 80));
        _objects.push_back(SearchSpace::Rect(20, 60, 60, 40, 80, 80));
        _objects.push_back(SearchSpace::Rect(60, 60, 60, 80, 80, 80));
        return _objects;
    }

    void RRTStar3D::save_edges(std::map<int, Tree> trees, std::string file_name){
       std::vector<double> edges; 
       int count = 0;
       for(auto const&item : trees[0].E){
            auto sp = item.first;
            auto ep = item.second;
            edges.push_back(std::get<0>(sp));
            edges.push_back(std::get<1>(sp));
            edges.push_back(std::get<2>(sp));
            edges.push_back(ep.state[0]);
            edges.push_back(ep.state[1]);
            edges.push_back(ep.state[2]);
            count += 1;
        }

        cnpy::npy_save(file_name, &edges[0],{(unsigned int)1, (unsigned int)count, (unsigned int)6},"w");
    }

    void RRTStar3D::save_obstacle(std::vector<SearchSpace::Rect> obstacles, std::string file_name){
        std::vector<double> obstacles_pose; 
        int count = 0;
        for(auto const& rect : obstacles){
                obstacles_pose.push_back(rect.min[0]);
                obstacles_pose.push_back(rect.min[1]);
                obstacles_pose.push_back(rect.min[2]);
                obstacles_pose.push_back(rect.max[0]);
                obstacles_pose.push_back(rect.max[1]);
                obstacles_pose.push_back(rect.max[2]);
                count += 1;
        }
        cnpy::npy_save(file_name, &obstacles_pose[0],{(unsigned int)1, (unsigned int)count, (unsigned int)6},"w");
    }

    void RRTStar3D::save_obstacle(std::vector<std::array<double, 6>> obstacles, std::string file_name){
        std::vector<double> obstacles_pose; 
        int count = 0;
        for(auto const& rect : obstacles){
                obstacles_pose.push_back(rect[0]);
                obstacles_pose.push_back(rect[1]);
                obstacles_pose.push_back(rect[2]);
                obstacles_pose.push_back(rect[3]);
                obstacles_pose.push_back(rect[4]);
                obstacles_pose.push_back(rect[5]);
                count += 1;
        }
        cnpy::npy_save(file_name, &obstacles_pose[0],{(unsigned int)1, (unsigned int)count, (unsigned int)6},"w");
    }

    void RRTStar3D::save_poses(PathNode start, PathNode end, std::string file_name){
       std::vector<double> obstacles_pose(6); 
       obstacles_pose[0] = start.state[0];
       obstacles_pose[1] = start.state[1];
       obstacles_pose[2] = start.state[2];
       obstacles_pose[3] = end.state[0];
       obstacles_pose[4] = end.state[1];
       obstacles_pose[5] = end.state[2];
       cnpy::npy_save(file_name, &obstacles_pose[0], {obstacles_pose.size()}, "w");
    }

    void RRTStar3D::save_path(std::vector<PathNode> path, std::string file_name){
       std::vector<double> projected_path;
        BOOST_LOG_TRIVIAL(info) << FCYN("RRTStar3D::save_path trajectory size: ") << path.size();
       for(auto const& way_point : path){
           projected_path.push_back(way_point.state[0]);
           projected_path.push_back(way_point.state[1]);
           projected_path.push_back(way_point.state[2]);
       }
       cnpy::npy_save(file_name, &projected_path[0], {path.size(), 3}, "w");
    }

    void RRTStar3D::save_long_path(std::vector<PathNode> path, std::string file_name){
    //    std::vector<double> projected_path;
    //    int waypoints = 0;
    //    BOOST_LOG_TRIVIAL(info) << FCYN("RRTStar3D::save_path trajectory size: smoothed path") << path.size();
    //    for(auto const& way_point : path){
    //             projected_path.push_back(point(0));
    //             projected_path.push_back(point(1));
    //             projected_path.push_back(point(2));

    //    }
    //    cnpy::npy_save(file_name, &projected_path[0], {waypoints, 3}, "w");
    }

    Eigen::VectorXd RRTStar3D::proceding_position(Eigen::VectorXd start_position, Eigen::VectorXd end_position
        , double distance)
    {
        Eigen::VectorXd next_pose(3);
        auto position_vector = end_position - start_position;
        auto x = position_vector[0];
        auto y = position_vector[1];
        auto z = position_vector[2];
        double diff = position_vector.norm();
        if( diff <= 0.0){
            BOOST_LOG_TRIVIAL(warning) << FYEL("Next pose of the cant be equal or less than zero...") << next_pose;
        }
        if( diff < distance){
            return start_position;
        }
        auto theta = std::atan2(y, x);
        auto phi = std::atan2(std::sqrt(x*x + y*y), z);
        // std::cout<< "theta: "<< theta << " phi: " << phi << std::endl;
        // double projected_dis = (diff - distance);
        auto target_z = distance*std::cos(phi) + start_position[2];
        auto target_x = distance*std::sin(phi)*std::cos(theta) + start_position[0];
        auto target_y = distance*std::sin(phi)*std::sin(theta) + start_position[1];
        next_pose<<target_x, target_y, target_z;
        // std::cout<< "Next pose:: inside:: not"<< next_pose << std::endl;
        return next_pose;
    }

    Eigen::VectorXd RRTStar3D::next_position(Eigen::VectorXd start_position, Eigen::VectorXd end_position
      , double distance)
    {
        Eigen::VectorXd next_pose(3);
        auto position_vector = end_position - start_position;
        auto x = position_vector[0];
        auto y = position_vector[1];
        auto z = position_vector[2];
        double diff = position_vector.norm();
        if( diff <= 0.0){
          BOOST_LOG_TRIVIAL(warning) << FYEL("Next pose of the cant be equal or less than zero...") << next_pose;
        }
        if( diff < distance){
         return end_position;
        }
        auto theta = std::atan2(y, x);
        auto phi = std::atan2(std::sqrt(x*x + y*y), z);
        // std::cout<< "theta: "<< theta << " phi: " << phi << std::endl;
        double projected_dis = (diff - distance);
        auto target_z = projected_dis*std::cos(phi) + start_position[2];
        auto target_x = projected_dis*std::sin(phi)*std::cos(theta) + start_position[0];
        auto target_y = projected_dis*std::sin(phi)*std::sin(theta) + start_position[1];
        next_pose<<target_x, target_y, target_z;
        // std::cout<< "Next pose:: inside:: not"<< next_pose << std::endl;
        return next_pose;

    }

    
}
}