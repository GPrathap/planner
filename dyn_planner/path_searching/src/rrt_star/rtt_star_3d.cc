#include "rrt_star_3d.h"

namespace kamaz {
namespace hagen {

    std::vector<PathNode> RRTStar3D::rrt_planner(SearchSpace search_space
                , PathNode start_pose, PathNode goal_pose
                , PathNode start_position, double obstacle_fail_safe_distance
                , double min_angle_allows_obs, CommonUtils& common_utils
                , std::atomic_bool &is_allowed_to_run){
        
        rrt_planner_options.search_space = search_space;
        rrt_planner_options.x_init = start_pose;
        rrt_planner_options.x_goal = goal_pose;
        rrt_planner_options.start_position = start_position;
        rrt_planner_options.obstacle_fail_safe_distance = obstacle_fail_safe_distance;
        rrt_planner_options.min_angle_allows_obs = min_angle_allows_obs;
        auto rrtstar = RRTStar(rrt_planner_options, _rewrite_count
                                            , common_utils, is_allowed_to_run);
        return rrtstar.rrt_star();
    }

    std::vector<PathNode> RRTStar3D::rrt_planner_and_save(SearchSpace search_space
                , PathNode start_pose, PathNode goal_pose
                , PathNode start_position, double obstacle_fail_safe_distance
                , double min_angle_allows_obs, CommonUtils& common_utils
                , std::atomic_bool &is_allowed_to_run, int index){
        
        rrt_planner_options.search_space = search_space;
        rrt_planner_options.x_init = start_pose;
        rrt_planner_options.x_goal = goal_pose;
        rrt_planner_options.start_position = start_position;
        rrt_planner_options.min_angle_allows_obs = min_angle_allows_obs;
        rrt_planner_options.obstacle_fail_safe_distance = obstacle_fail_safe_distance;

        auto rrtstar =  RRTStar(rrt_planner_options, _rewrite_count, common_utils
                        , is_allowed_to_run);
        std::ofstream outfile;
        // std::cout<< " ========================4444444" << std::endl;
        // outfile.open("/dataset/rrt_old/time_stamps.txt", std::ios_base::app);
        // std::cout<< " ========================4444444" << std::endl;
        // const clock_t begin_time = clock();
        // std::cout<< " ========================4444444" << std::endl;
        auto path = rrtstar.rrt_star();
        // double time_diff =  double( clock () - begin_time ) /  CLOCKS_PER_SEC;
        // if(path.size()>1){
            //  outfile << "rrt,"<<  time_diff <<","<< path.size() << "," << get_distance(path) << "," <<  common_utils.get_cost_of_path(path) << "\n";
        // }
        // std::cout<< " ========================4444444" << std::endl;
        stotage_location = "/dataset/rrt_old/"+ std::to_string(index) + "_";
        save_edges(rrtstar.trees, stotage_location + "edges.npy");
        save_obstacle(search_space.random_objects, stotage_location + "obstacles.npy");
        save_poses(start_pose, goal_pose, stotage_location + "start_and_end_pose.npy");
        if(path.size()>0){
            save_path(path, stotage_location + "rrt_star_path.npy");
        }
        
        return path;
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

    void RRTStar3D::rrt_init(std::vector<Eigen::Vector2d> _lengths_of_edges
            , int max_samples, int _resolution, double _pro, int rewrite_count){
        rrt_planner_options.lengths_of_edges = _lengths_of_edges;
        rrt_planner_options.max_samples = max_samples;
        rrt_planner_options.resolution = _resolution; 
        rrt_planner_options.pro = _pro;
        _rewrite_count = rewrite_count;
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

    std::vector<SearchSpace::Rect> RRTStar3D::get_random_obstacles(int number_of_obstacles
    , Eigen::VectorXd x_dimentions, PathNode x_init, PathNode x_goal){
        std::vector<SearchSpace::Rect> _objects;
        srand(time(NULL));
        for(int i=0; i< number_of_obstacles; i++){
            
            double x_plus = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) *x_dimentions[1];
            double y_plus = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) *x_dimentions[3];
            double z_plus = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) *x_dimentions[5];

            double x_minus = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) *x_dimentions[0];
            double y_minus = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) *x_dimentions[2];
            double z_minus = static_cast <double> (rand()) / static_cast <double> (RAND_MAX) *x_dimentions[4];

            double x = x_plus + x_minus;
            double y = y_plus + y_minus;
            double z = z_plus + z_minus;
            double width_x = x + rand()%5;
            double width_y = y + rand()%5;
            double width_z = z + rand()%5;
            // std::cout<< x << " " <<  y << " "<< z << " " << width_x << " " << width_y << " " << width_z << std::endl;
            if(width_x >= x_dimentions[1] || width_y >= x_dimentions[3] || width_z >= x_dimentions[5]){
                continue;
            }
            if(x <= x_dimentions[0]  || y <= x_dimentions[2]  || z <= x_dimentions[4] ){
                continue;
            }
            
            _objects.push_back(SearchSpace::Rect(x, y, z, width_x, width_y, width_z));
        }
        std::cout<< "RRTStar3D::get_random_obstacles: Size of the objects "<< _objects.size() << std::endl;
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
}
}