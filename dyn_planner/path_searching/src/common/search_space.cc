#include "search_space.h"

namespace kamaz {
namespace hagen {
    SearchSpace::SearchSpace() {
         random_points_tank = std::make_shared<Eigen::MatrixXd>();
    }

    void SearchSpace::init_search_space(Eigen::VectorXd dimension_lengths
                , int num_of_rand_points, double _avoidance_width
                , int number_of_tries_at_time){
        dim_lengths = dimension_lengths;
        std::uniform_real_distribution<> distribution_x(dimension_lengths[0], dimension_lengths[1]);
        std::uniform_real_distribution<> distribution_y(dimension_lengths[2], dimension_lengths[3]);
        std::uniform_real_distribution<> distribution_z(dimension_lengths[4], dimension_lengths[5]);
        uni_dis_vector.push_back(distribution_x);
        uni_dis_vector.push_back(distribution_y);
        uni_dis_vector.push_back(distribution_z);
        number_of_rand_points = num_of_rand_points;
        number_of_max_attempts = number_of_tries_at_time;
        random_call = new Random_call(std::chrono::system_clock::now().time_since_epoch().count(), num_of_rand_points);
        obstacle_counter = 0;
        avoidance_width = _avoidance_width;
    }

    void SearchSpace::setEnvironment(const dyn_planner::EDTEnvironment::Ptr& env){
        edt_env_ = env;
    }

    // bool SearchSpace::is_inside_map(Eigen::Vector3d pos){
    //     edt_env_->is_inside_map
    // }

    void SearchSpace::generate_random_objects(int num_of_objects){
        for (int i = 0 ; i < num_of_objects ; ++i)
        {
            double min_x = rand() % 1000;
            double min_y = rand() % 1000;
            double min_z = rand() % 1000;
            double w = 1 + rand() % 100;
            double h = 1 + rand() % 100;
            double d = 1 + rand() % 100;
            random_objects.push_back(Rect(min_x, min_y, min_z, min_x+w, min_y+h, min_z+d));
        }
    }

    void SearchSpace::insert_obstacles(std::vector<Rect> obstacles){
        // if(obstacles.size() == 0){
        //     std::cout<< "No obstacle to be inserted" << std::endl;
        // }
        // for(size_t i = 0; i < obstacles.size(); i++){
        //     Rect const& r = obstacles[i];
        //     box_t b(point_t(r.min[0], r.min[1], r.min[2]), point_t(r.max[0], r.max[1], r.max[2]));
        //     bg_tree.insert(value_t(b, i));
        // }
        // random_objects = obstacles;
    }

    void SearchSpace::insert_trajectory(std::vector<Rect> way_points){
        if(way_points.size() == 0){
            std::cout<< "No waypoints to be inserted" << std::endl;
        }
        for(size_t i = 0; i < way_points.size(); i++){
            Rect const& r = way_points[i];
            box_t b(point_t(r.min[0], r.min[1], r.min[2]), point_t(r.max[0], r.max[1], r.max[2]));
            current_trajectory.insert(value_t(b, i));
        }
    }

    void SearchSpace::update_obstacles_map(std::vector<Rect> way_points){
        if(way_points.size() == 0){
            std::cout<< "No waypoints to be inserted" << std::endl;
        }
        for(size_t i = 0; i < way_points.size(); i++){
            Rect const& r = way_points[i];
            box_t b(point_t(r.min[0], r.min[1], r.min[2]), point_t(r.max[0]
                            , r.max[1], r.max[2]));
            obs_tree.insert(value_t(b, i));
        }
        random_objects = way_points;
        obstacle_counter = way_points.size();
    }

    std::vector<Eigen::Vector3d> SearchSpace::nearest_obstacles_to_current_pose(Eigen::Vector3d x
                , int max_neighbours){
        // // std::vector<value_t> returned_values;
        // std::vector<Eigen::Vector3d> neighbour_points;
        // for ( RTree::const_query_iterator it = obs_tree.qbegin(bgi::nearest(point_t(x[0], x[1], x[2]), max_neighbours)) ;
        //         it != obs_tree.qend() ; ++it )
        // {
        //     Eigen::Vector3d pose(3);
        //     auto cube = (*it).first;
        //     double min_x = bg::get<bg::min_corner, 0>(cube);
        //     double min_y = bg::get<bg::min_corner, 1>(cube);
        //     double min_z = bg::get<bg::min_corner, 2>(cube);
        //     pose << min_x, min_y, min_z;
        //     neighbour_points.push_back(pose);
        // }
        
        return edt_env_->nearest_obstacles_to_current_pose(x, max_neighbours);
    }

    double SearchSpace::get_free_space(Eigen::Vector3d pose, std::vector<Eigen::Vector3d>& obs_poses, int num_of_obs){
        auto obstacles = nearest_obstacles_to_current_pose(pose, num_of_obs);
        if(obstacles.size() <= 0){
            return 3.0; //Since no onstacles on the obstacles map
        }
        obs_poses = obstacles;
        return (obstacles[0]-pose).norm();
    }

    double SearchSpace::get_free_space(Eigen::Vector3d pose){
        auto obstacles = nearest_obstacles_to_current_pose(pose, 1);
        if(obstacles.size() <= 0){
            return 10.0; //Since no onstacles on the obstacles map
        }
        return (obstacles[0]-pose).norm();
    }

    void SearchSpace::insert_obstacle(Eigen::Vector3d index){
        //TODO not sure this method
        box_t b(point_t(index[0], index[1], index[2])
        , point_t(index[0]+avoidance_width, index[1]+avoidance_width
        , index[2]+avoidance_width));
        obs_tree.insert(value_t(b, obstacle_counter));
    }

    void SearchSpace::insert_vertex(Eigen::Vector3d index){
        box_t b(point_t(index[0], index[1], index[2])
        , point_t(index[0]+avoidance_width, index[1]+avoidance_width
        , index[2]+avoidance_width));
        bg_tree.insert(value_t(b, obstacle_counter));
        obstacle_counter++;
    }
    
     std::vector<Eigen::Vector3d> SearchSpace::nearest_obstacles(Eigen::Vector3d x, int max_neighbours){
        // std::vector<value_t> returned_values;
        
        std::vector<Eigen::Vector3d> neighbour_points;
        for ( RTree::const_query_iterator it = obs_tree.qbegin(bgi::nearest(point_t(x[0], x[1], x[2]), max_neighbours)) ;
                it != obs_tree.qend() ; ++it )
        {
            Eigen::Vector3d pose(3);
            auto cube = (*it).first;

            // std::cout<< "SearchSpace::nearest_obstacles:  distance: " << bg::distance(cube, point_t(x[0], x[1], x[2])) << std::endl;

            double min_x = bg::get<bg::min_corner, 0>(cube);
            double min_y = bg::get<bg::min_corner, 1>(cube);
            double min_z = bg::get<bg::min_corner, 2>(cube);

            // double max_x = bg::get<bg::max_corner, 0>(cube);
            // double max_y = bg::get<bg::max_corner, 1>(cube);
            // double max_z = bg::get<bg::max_corner, 2>(cube);

            // std::cout<< "SearchSpace::nearest_obstacles: "<< min_x << ","<<min_y << ", "<< min_z << ", " << max_x << ", "<< max_y << ", " << max_z << std::endl;

            // pose << (min_x+max_x)/2, (min_y+max_y)/2, (min_z+max_z)/2;
            pose << min_x, min_y, min_z;
            neighbour_points.push_back(pose);
        
        }
        return neighbour_points;        
     }
    std::vector<Eigen::Vector3d> SearchSpace::nearest_veties(Eigen::Vector3d x, int max_neighbours){
        // std::vector<value_t> returned_values;
        
        std::vector<Eigen::Vector3d> neighbour_points;
        for ( RTree::const_query_iterator it = bg_tree.qbegin(bgi::nearest(point_t(x[0], x[1], x[2]), max_neighbours)) ;
                it != bg_tree.qend() ; ++it )
        {
            Eigen::Vector3d pose(3);
            auto cube = (*it).first;
            // std::cout<< "SearchSpace::nearest_obstacles:  distance: " << bg::distance(cube, point_t(x[0], x[1], x[2])) << std::endl;
            double min_x = bg::get<bg::min_corner, 0>(cube);
            double min_y = bg::get<bg::min_corner, 1>(cube);
            double min_z = bg::get<bg::min_corner, 2>(cube);
            // double max_x = bg::get<bg::max_corner, 0>(cube);
            // double max_y = bg::get<bg::max_corner, 1>(cube);
            // double max_z = bg::get<bg::max_corner, 2>(cube);

            // std::cout<< "SearchSpace::nearest_obstacles: "<< min_x << ","<<min_y << ", "<< min_z << ", " << max_x << ", "<< max_y << ", " << max_z << std::endl;

            // pose << (min_x+max_x)/2, (min_y+max_y)/2, (min_z+max_z)/2;
            pose << min_x, min_y, min_z;
            neighbour_points.push_back(pose);
        }
        return neighbour_points;        
    }

    std::vector<Eigen::Vector3d> SearchSpace::nearest_point_on_trajectory(Eigen::Vector3d x, int max_neighbours){
        // std::vector<value_t> returned_values;
        std::vector<Eigen::Vector3d> neighbour_points;
        for ( RTree::const_query_iterator it = current_trajectory.qbegin(bgi::nearest(point_t(x[0], x[1], x[2]), max_neighbours)) ;
                it != current_trajectory.qend() ; ++it )
        {
            Eigen::Vector3d pose(3);
            auto cube = (*it).first;
            double min_x = bg::get<bg::min_corner, 0>(cube);
            double min_y = bg::get<bg::min_corner, 1>(cube);
            double min_z = bg::get<bg::min_corner, 2>(cube);
            pose << min_x, min_y, min_z;
            neighbour_points.push_back(pose);
        }
        return neighbour_points;        
    }

    std::vector<double> SearchSpace::arange(double start, double stop, double step) {
        std::vector<double> values;
        for (double value = start; value < stop; value += step)
            values.push_back(value);
        return values;
    }

    void SearchSpace::generate_search_sapce(Eigen::MatrixXd covmat, Eigen::Matrix3d rotation_mat,
            Eigen::Vector3d cent, int npts){
        // int ndims = covmat.rows();
        // number_of_points_in_random_tank = npts;
        // // std::cout<< "======6" << std::endl;
        // *random_points_tank = Eigen::MatrixXd::Zero(npts, ndims);
        // // std::cout<< "======8" << std::endl;
        // generate_samples_from_ellipsoid(covmat, rotation_mat, cent);
        // // std::cout<< "======9" << std::endl;
        // is_random_tank_is_ready = true;
        return;
    }

    std::vector<SearchSpace::Rect> SearchSpace::get_random_obstacles(int number_of_obstacles
            , Eigen::VectorXd x_dimentions){
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

    void SearchSpace::save_search_space(int index){
        std::vector<double> sample_pose;
        for(int i=0; i< random_objects.size(); i++){
                sample_pose.push_back(random_objects[i].min[0]);
                sample_pose.push_back(random_objects[i].min[1]);
                sample_pose.push_back(random_objects[i].min[2]);
                sample_pose.push_back(random_objects[i].max[0]);
                sample_pose.push_back(random_objects[i].max[1]);
                sample_pose.push_back(random_objects[i].max[2]);
        }
        std::string file_name = "/dataset/" + std::to_string(index)+ "_search_space.npy";
        cnpy::npy_save(file_name, &sample_pose[0],{(unsigned int)1, (unsigned int)random_objects.size(), (unsigned int)6},"w");
    }

    void SearchSpace::save_samples(int index){
        // std::vector<double> sample_pose;
        // for(int i=0; i< (*random_points_tank).rows(); i++){
        //     for(int j=0; j< (*random_points_tank).cols(); j++){
        //         sample_pose.push_back((*random_points_tank)(i,j));
        //     }
        // }
        // std::string file_name = "/dataset/" + std::to_string(index)+ "_random_samples.npy";
        // cnpy::npy_save(file_name, &sample_pose[0],{(unsigned int)1, (unsigned int)(*random_points_tank).rows(), (unsigned int)(*random_points_tank).cols()},"w");
    }

    // bool SearchSpace::obstacle_free(Rect search_rect){
    //     box_t search_box(
    //     point_t(search_rect.min[0], search_rect.min[1], search_rect.min[2]),
    //     point_t(search_rect.max[0], search_rect.max[1], search_rect.max[2]));
    //     size_t sum = 0;
    //     // boost::timer t;
    //     // res.clear();
    //     sum += obs_tree.query(bgi::intersects(search_box)
    //     , boost::make_function_output_iterator(geometry_rtree_callback));
    //     // double s = t.elapsed();
    //     // std::cout << search_rect.min[0] << " " << search_rect.min[1] << " " << search_rect.min[2] << std::endl;
    //     // std::cout << search_rect.max[0] << " " << search_rect.max[1] << " " << search_rect.max[2] << std::endl;

    //     // std::cout << "sum up..." << sum << std::endl;
    //     return sum > 0 ? false : true;
    // }

    void SearchSpace::generate_samples_from_ellipsoid(Eigen::MatrixXd covmat, Eigen::Matrix3d rotation_mat, Eigen::Vector3d cent){
        // std::cout<< "======11" << std::endl;

        int ndims = (*random_points_tank).cols();
        int npts = (*random_points_tank).rows();
        Eigen::EigenSolver<Eigen::MatrixXd> eigensolver;
        // std::cout<< "======12" << std::endl;
        eigensolver.compute(covmat);
        // std::cout<< "======13" << std::endl;
        Eigen::Vector3d eigen_values = eigensolver.eigenvalues().real();
        Eigen::MatrixXd eigen_vectors = eigensolver.eigenvectors().real();
        std::vector<std::tuple<double, Eigen::Vector3d>> eigen_vectors_and_values;
        // std::cout<< "======14" << std::endl;
        for(int i=0; i<eigen_values.size(); i++){
            std::tuple<double, Eigen::Vector3d> vec_and_val(eigen_values[i], eigen_vectors.row(i));
            eigen_vectors_and_values.push_back(vec_and_val);
        }
        std::sort(eigen_vectors_and_values.begin(), eigen_vectors_and_values.end(),
            [&](const std::tuple<double, Eigen::Vector3d>& a, const std::tuple<double, Eigen::Vector3d>& b) -> bool{
                return std::get<0>(a) <= std::get<0>(b);
        });
        int index = 0;
        for(auto const vect : eigen_vectors_and_values){
            eigen_values(index) = std::get<0>(vect);
            eigen_vectors.row(index) = std::get<1>(vect);
            index++;
        }
        //  std::cout<< "======15" << std::endl;
        Eigen::MatrixXd eigen_values_as_matrix = eigen_values.asDiagonal();

        std::random_device rd{};
        std::mt19937 gen{rd()};
        std::uniform_real_distribution<double> dis(0, 1);
        std::normal_distribution<double> normal_dis{0.0f, 1.0f};
        // std::cout<< "======16" << std::endl;
        Eigen::MatrixXd pt = Eigen::MatrixXd::Zero(npts, ndims).unaryExpr([&](double dummy){return (double)normal_dis(gen);});
        // std::cout<< "======17" << std::endl;
        Eigen::VectorXd rs = Eigen::VectorXd::Zero(npts).unaryExpr([&](double dummy){return dis(gen);});
        // std::cout<< "======18" << std::endl;
        Eigen::VectorXd fac = pt.array().pow(2).rowwise().sum();
        // std::cout<< "======19" << std::endl;
        Eigen::VectorXd fac_sqrt = fac.array().sqrt();
        // std::cout<< "======211" << std::endl;
        Eigen::VectorXd rs_pow = rs.array().pow(1.0/ndims);
        // std::cout<< "======221" << std::endl;
        fac = rs_pow.array()/fac_sqrt.array();
        // std::cout<< "======221" << std::endl;
        Eigen::VectorXd d = eigen_values_as_matrix.diagonal().array().sqrt();
        // std::cout << "============================================start" << npts << std::endl;
        for(auto i(0); i<npts; i++){
            // std::cout << "============================================>>>>>>"<< i << std::endl;
            (*random_points_tank).row(i) = fac(i)*pt.row(i).array();
            // std::cout << "=======4====" << std::endl;
            Eigen::MatrixXd  fff = ((*random_points_tank).row(i).array()*d.transpose().array());
            // std::cout << "=======4====" << std::endl;
            Eigen::VectorXd bn = rotation_mat*fff.transpose();
            // std::cout << "=======12====" << std::endl;
            (*random_points_tank).row(i) = bn.array() + cent.head(3).array();
        }
        // std::cout << "============================================end" << npts << std::endl;
        // std::cout << "points: " << (*random_points_tank) << std::endl;
    }


    // bool SearchSpace::obstacle_free(Eigen::Vector3d search_rect){
    //     box_t search_box(
    //     point_t(search_rect[0], search_rect[1], search_rect[2]),
    //     point_t(search_rect[0]+avoidance_width, search_rect[1]+avoidance_width, search_rect[2]+avoidance_width));
    //     size_t sum = 0;
    //     // boost::timer t;
    //     // res.clear();
    //     sum += obs_tree.query(bgi::intersects(search_box)
    //     , boost::make_function_output_iterator(geometry_rtree_callback));
    //     // double s = t.elapsed();
    //     // std::cout <<" SearchSpace::obstacle_free: sum: " << sum << std::endl;
    //     // std::cout << search_rect.transpose() << std::endl;
    //     // // std::cout << search_rect.max[0] << " " << search_rect.max[1] << " " << search_rect.max[2] << std::endl;

    //     // std::cout << "sum up..." << sum << std::endl;
  
    //     return sum > 0 ? false : true;
    // }

    bool SearchSpace::obstacle_free(Eigen::Vector3d search_rect, double optimal_time){
    //    std::cout<< "=========1" << std::endl;
    //    double dis = edt_env_->evaluateCoarseEDT(search_rect, optimal_time);
        double dis = edt_env_->get_free_distance(search_rect);
    //    std::cout<< "=============>> "<< dis << "  " << avoidance_width << std::endl;
       if((avoidance_width < dis)){
           return true;
       }
       return false;
    }

    double SearchSpace::get_free_space(Eigen::Vector3d search_rect, double optimal_time){
    //  double dis = edt_env_->evaluateCoarseEDT(search_rect, optimal_time);
        double dis = edt_env_->get_free_distance(search_rect);
       return dis;
    }


    // Eigen::Vector3d SearchSpace::get_sample_with_the_space(Eigen::Vector3d sample_){
    //      while(true){
             
    //          edt_env_->is_inside_map(sample_)
    //      }
    // }

    Eigen::Vector3d SearchSpace::sample(){
        Eigen::Vector3d random_pose(3);
        // std::default_random_engine generator_on_x;
        // generator_on_x.seed(std::chrono::system_clock::now().time_since_epoch().count());
        // auto x_on = uni_dis_vector[0](generator_on_x);
        // generator_on_x.seed(std::chrono::system_clock::now().time_since_epoch().count());
        // auto y_on = uni_dis_vector[1](generator_on_x);
        // generator_on_x.seed(std::chrono::system_clock::now().time_since_epoch().count());
        // auto z_on = uni_dis_vector[2](generator_on_x);
        // random_pose << x_on, y_on, z_on ;
       
                if(use_whole_search_sapce){
                std::default_random_engine generator_on_x;
                generator_on_x.seed(std::chrono::system_clock::now().time_since_epoch().count());
                auto x_on = uni_dis_vector[0](generator_on_x);
                generator_on_x.seed(std::chrono::system_clock::now().time_since_epoch().count());
                auto y_on = uni_dis_vector[1](generator_on_x);
                generator_on_x.seed(std::chrono::system_clock::now().time_since_epoch().count());
                auto z_on = uni_dis_vector[2](generator_on_x);
                random_pose << x_on, y_on, z_on ;
               
                }
                else{
                    while(true){
                        int index = *(random_call);
                        // std::cout<< "index: " << index << std::endl;
                        if((index < (*random_points_tank).rows()) && (index>0)){
                            // std::cout<< "========================1114"<< index << "===" << (*random_points_tank).rows() << std::endl;
                            // std::cout<< "========================1114"<< index << "===" << (*random_points_tank).cols() << std::endl;
                            if(is_random_tank_is_ready){
                                random_pose = (*random_points_tank).row(index);
                                // std::cout<< "========================1115" << std::endl;
                            }
                            break;
                        }
                    }
                }
        // }
        return random_pose;
    }

    Eigen::Vector3d SearchSpace::sample_free(){
        static int number_of_attempts = 0;
        while(true){
            number_of_attempts++;
            if(number_of_attempts>number_of_max_attempts){
                BOOST_LOG_TRIVIAL(info) << FRED("Giving whole space for searching...");
                use_whole_search_sapce = true;
            }
            Eigen::Vector3d x = sample();
            if(!edt_env_->is_inside_map(x)){
                 std::cout<< "Sample is out of the search space..." << x.transpose() <<std::endl;
            }
            else if(obstacle_free(x, -1.0)){
                // std::cout<< "free sample--->" << x.transpose() <<std::endl;
                number_of_attempts = 0;
                return x;
            }
        }
    }

    bool SearchSpace::collision_free(Eigen::Vector3d start, Eigen::Vector3d end, int r){
        auto dist = (start - end).norm();
        double resolution = std::ceil(dist/r);
        std::vector<double> res_on_x = linspace(start[0], end[0], resolution);
        std::vector<double> res_on_y = linspace(start[1], end[1], resolution);
        std::vector<double> res_on_z = linspace(start[2], end[2], resolution);
        // std::cout<< "===================:collision_free" << std::endl;
        // std::cout<<  res_on_x.size() << " " << res_on_y.size() <<" "<< res_on_z.size() << std::endl;
        int len = std::min({res_on_x.size(), res_on_y.size(), res_on_z.size()}
        , [](const int s1, const int s2) -> bool{
                return s1 < s2;
        });
        std::cout<< "SearchSpace::collision_free:: len: " << len << std::endl;
        for(int i=0; i<len; i++){
            Eigen::Vector3d search_rect(3);
            search_rect<< res_on_x[i], res_on_y[i], res_on_z[i];
            // std::cout<<" collision_free  " << search_rect.transpose() << std::endl;
            if(!obstacle_free(search_rect, -1.0)){
                return false;
            }
        }
        return true;
    }


    bool SearchSpace::collision_free(Eigen::Vector3d start, Eigen::Vector3d end, int r, double optimal_time){
        auto dist = (start - end).norm();
        double resolution = std::ceil(dist/r);
        std::vector<double> res_on_x = linspace(start[0], end[0], resolution);
        std::vector<double> res_on_y = linspace(start[1], end[1], resolution);
        std::vector<double> res_on_z = linspace(start[2], end[2], resolution);
        // std::cout<< "+===================:collision_free" << std::endl;
        // std::cout<<  res_on_x.size() << " " << res_on_y.size() <<" "<< res_on_z.size() << std::endl;
        int len = std::min({res_on_x.size(), res_on_y.size(), res_on_z.size()}
        , [](const int s1, const int s2) -> bool{
                return s1 < s2;
        });
        // std::cout<< "SearchSpace::collision_free:: len: " << len << std::endl;
        for(int i=0; i<len; i++){
            Eigen::Vector3d search_rect(3);
            search_rect<< res_on_x[i], res_on_y[i], res_on_z[i];
            // std::cout<<" collision_free  " << search_rect.transpose() << std::endl;
            if(!obstacle_free(search_rect, optimal_time)){
                return false;
            }
        }
        return true;
    }

        std::vector<double> SearchSpace::linspace(double start_in, double end_in, double step_size)
        {
            std::vector<double> linspaced;
            double start = start_in;
            double end = end_in;
            double num = step_size;
            if (num == 0) {
                 return linspaced; 
            }
            if (num == 1)
            {
                linspaced.push_back(start);
                return linspaced;
            }
            double delta = (end - start) / (num - 1);
            for(int i=0; i < num-1; ++i)
            {
                linspaced.push_back(start + delta * i);
            }
            linspaced.push_back(end);
            return linspaced;
        }

        double*  SearchSpace::ellipsoid_grid ( int n, int ng ){
            double h;
            int ii;
            int i;
            int j;
            int k;
            int m;
            int ng2;
            int ni;
            int nj;
            int nk;
            int np;
            double p[3*8];
            double rmin;
            double x;
            double *xyz;
            double y;
            double z;
            ng2 = 0;
            xyz = new double[3*ng];
            rmin = r8vec_min ( 3, r );
            if ( r[0] == rmin )
            {
                h = 2.0 * r[0] / ( double ) ( 2 * n + 1 );
                ni = n;
                nj = i4_ceiling ( r[1] / r[0] ) * ( double ) ( n );
                nk = i4_ceiling ( r[2] / r[0] ) * ( double ) ( n );
            }
            else if ( r[1] == rmin )
            {
                h = 2.0 * r[1] / ( double ) ( 2 * n + 1 );
                nj = n;
                ni = i4_ceiling ( r[0] / r[1] ) * ( double ) ( n );
                nk = i4_ceiling ( r[2] / r[1] ) * ( double ) ( n );
            }
            else
            {
                h = 2.0 * r[2] / ( double ) ( 2 * n + 1 );
                nk = n;
                ni = i4_ceiling ( r[0] / r[2] ) * ( double ) ( n );
                nj = i4_ceiling ( r[1] / r[2] ) * ( double ) ( n );
            }

            for ( k = 0; k <= nk; k++ )
            {
                z = c[2] + ( double ) ( k ) * h;
                for ( j = 0; j <= nj; j++ )
                {
                y = c[1] + ( double ) ( j ) * h;
                for ( i = 0; i <= ni; i++ )
                {
                    x = c[0] + ( double ) ( i ) * h;
                    if ( 1.0 < pow ( ( x - c[0] ) / r[0], 2 )
                            + pow ( ( y - c[1] ) / r[1], 2 )
                            + pow ( ( z - c[2] ) / r[2], 2 ) )
                    {
                        break;
                    }
                    np = 0;
                    p[0+np*3] = x;
                    p[1+np*3] = y;
                    p[2+np*3] = z;
                    np = 1;

                    if ( 0 < i )
                    {
                        for ( m = 0; m < np; m++ )
                        {
                            p[0+(np+m)*3] = 2.0 * c[0] - p[0+m*3];
                            p[1+(np+m)*3] = p[1+m*3];
                            p[2+(np+m)*3] = p[2+m*3];
                        }
                        np = 2 * np;
                    }

                    if ( 0 < j )
                    {
                        for ( m = 0; m < np; m++ )
                        {
                            p[0+(np+m)*3] = p[0+m*3];
                            p[1+(np+m)*3] = 2.0 * c[1] - p[1+m*3];
                            p[2+(np+m)*3] = p[2+m*3];
                        }
                        np = 2 * np;
                    }

                    if ( 0 < k )
                    {
                        for ( m = 0; m < np; m++ )
                        {
                            p[0+(np+m)*3] = p[0+m*3];
                            p[1+(np+m)*3] = p[1+m*3];
                            p[2+(np+m)*3] = 2.0 * c[2] - p[2+m*3];
                        }
                        np = 2 * np;
                    }

                    for ( m = 0; m < np; m++ )
                    {
                        for ( ii = 0; ii < 3; ii++ )
                        {
                            xyz[ii+(ng2+m)*3] = p[ii+m*3];
                        }
                    }
                    ng2 = ng2 + np;
                }
            }
        }
        return xyz;
    }

    int SearchSpace::ellipsoid_grid_count( int n, Eigen::Vector3d radios, Eigen::Vector3d center){

            r[0] = radios[0];
            r[1] = radios[1];
            r[2] = radios[2];
            c[0] = center[0];
            c[1] = center[1];
            c[2] = center[2];
            double h;
            int i;
            int j;
            int k;
            int m;
            int ng;
            int ni;
            int nj;
            int nk;
            int np;
            double rmin;
            double x;
            double y;
            double z;

            ng = 0;

            rmin = r8vec_min ( 3, r );

            if ( r[0] == rmin )
            {
                h = 2.0 * r[0] / ( double ) ( 2 * n + 1 );
                ni = n;
                nj = i4_ceiling ( r[1] / r[0] ) * ( double ) ( n );
                nk = i4_ceiling ( r[2] / r[0] ) * ( double ) ( n );
            }
            else if ( r[1] == rmin )
            {
                h = 2.0 * r[1] / ( double ) ( 2 * n + 1 );
                nj = n;
                ni = i4_ceiling ( r[0] / r[1] ) * ( double ) ( n );
                nk = i4_ceiling ( r[2] / r[1] ) * ( double ) ( n );
            }
            else
            {
                h = 2.0 * r[2] / ( double ) ( 2 * n + 1 );
                nk = n;
                ni = i4_ceiling ( r[0] / r[2] ) * ( double ) ( n );
                nj = i4_ceiling ( r[1] / r[2] ) * ( double ) ( n );
            }

            for ( k = 0; k <= nk; k++ )
            {
                z = c[2] + ( double ) ( k ) * h;
                for ( j = 0; j <= nj; j++ )
                {
                y = c[1] + ( double ) ( j ) * h;
                for ( i = 0; i <= ni; i++ )
                {
                    x = c[0] + ( double ) ( i ) * h;
                    if ( 1.0 < pow ( ( x - c[0] ) / r[0], 2 )
                            + pow ( ( y - c[1] ) / r[1], 2 )
                            + pow ( ( z - c[2] ) / r[2], 2 ) )
                    {
                    break;
                    }

                    np = 1;
                    if ( 0 < i )
                    {
                    np = 2 * np;
                    }
                    if ( 0 < j )
                    {
                    np = 2 * np;
                    }
                    if ( 0 < k )
                    {
                    np = 2 * np;
                    }
                    ng = ng + np;
                }
                }
            }
            return ng;
    }

    int SearchSpace::i4_ceiling( double x )
    {
        int value;

        value = ( int ) x;

        if ( value < x )
        {
            value = value + 1;
        }

        return value;
    }

    void SearchSpace::r83vec_print_part( int n, double a[], Eigen::Vector3d center_pose, Eigen::Matrix3d rotation_matrix, std::string file_name )
    {
        int i;
        if ( n <= 0 )
        {
            return;
        }
        std::vector<double> edges;
        int count = 0;
        for ( i = 0; i < n - 2; i++ )
        {
            Eigen::Vector3d point;
            point<< a[0+i*3], a[1+i*3], a[2+i*3];
            Eigen::Vector3d ff = point.transpose()*rotation_matrix;
            std::cout<< center_pose << std::endl;
            std::cout<< ff << std::endl;
            ff = ff + center_pose;
            std::cout<< ff << std::endl;
            edges.push_back(ff[0]);
            edges.push_back(ff[1]);
            edges.push_back(ff[2]);
            count +=1;
        }
        cnpy::npy_save(file_name, &edges[0],{(unsigned int)1, (unsigned int)count, (unsigned int)3},"w");
    return;
    }

    void SearchSpace::generate_points( int n, Eigen::Vector3d radios, Eigen::Vector3d center_pose
            , Eigen::Matrix3d rotation_matrix)
    {
        Eigen::Vector3d c(0,0,0);
        int ng = ellipsoid_grid_count(n, radios, c);
        number_of_points_in_random_tank = ng-2;
        // std::cout << "\n";
        // std::cout << "  Number of grid points will be " << ng << "\n";
        double* a = ellipsoid_grid(n, ng);
        *random_points_tank = Eigen::MatrixXd::Zero(ng-2, 3);
        int i;
        if ( n <= 0 )
        {
            return;
        }

        int count = 0;
        for ( i = 0; i < ng - 2; i++ )
        {
            Eigen::Vector3d point;
            point<< a[0+i*3], a[1+i*3], a[2+i*3];
            Eigen::Vector3d ff = point.transpose()*rotation_matrix;
            ff = ff + center_pose;
            (*random_points_tank).row(i) = ff;
            count +=1;
        }
        is_random_tank_is_ready = true;
        return;
    }

    double SearchSpace::r8vec_min( int n, double r8vec[] ){
        int i;
        double value;

        value = r8vec[0];

        for ( i = 1; i < n; i++ )
        {
            if ( r8vec[i] < value )
            {
            value = r8vec[i];
            }
        }
        return value;
    }

    void SearchSpace::timestamp ( ){
        # define TIME_SIZE 40
        static char time_buffer[TIME_SIZE];
        const struct std::tm *tm_ptr;
        size_t len;
        std::time_t now;
        now = std::time ( NULL );
        tm_ptr = std::localtime ( &now );
        len = std::strftime ( time_buffer, TIME_SIZE, "%d %B %Y %I:%M:%S %p", tm_ptr );
        std::cout << time_buffer << "\n";
        return;
        # undef TIME_SIZE
    }
}
}


