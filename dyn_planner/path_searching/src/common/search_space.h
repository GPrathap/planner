#ifndef PATH_PLANNER_SEARCH_SPACE_H
#define PATH_PLANNER_SEARCH_SPACE_H

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
#include <Eigen/Dense>
#include <chrono> 
#include <vector>
#include <cnpy.h>
#include <boost/function_output_iterator.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/timer.hpp>
#include <boost/foreach.hpp>
#include "../utils/common_utils.h"
#include "plan_env/edt_environment.h"
#include "../../include/colours.h"
#include <random>

namespace kamaz {
namespace hagen {
        namespace bg = boost::geometry;
        namespace bgi = boost::geometry::index;
        class SearchSpace {
            typedef bg::model::point<double, 3, bg::cs::cartesian> point_t;
            typedef bg::model::box<point_t> box_t;
            typedef std::pair<box_t, uint64_t> value_t;
            typedef boost::geometry::box_view<box_t> box_view;
            typedef bgi::rtree<value_t, bgi::quadratic<8, 4>> RTree;

            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                SearchSpace();
                ~SearchSpace() = default;

                std::vector<std::uniform_real_distribution<>> uni_dis_vector;

                struct Rect {
                    Rect()  {}
                    Rect(double a_minX, double a_minY, double a_minZ, double a_maxX
                    , double a_maxY, double a_maxZ){
                        min[0] = a_minX;
                        min[1] = a_minY;
                        min[2] = a_minZ;

                        max[0] = a_maxX;
                        max[1] = a_maxY;
                        max[2] = a_maxZ;
                    }
                    double min[3];
                    double max[3];
                };

                struct Random_call
                {
                    Random_call(unsigned seed, int number_of_rand_points)
                    : _mt19937_64(seed)
                    , _uniform_int(0, number_of_rand_points)
                    {}

                    operator int() {
                        return _uniform_int(_mt19937_64);
                    }

                    std::mt19937 _mt19937_64;
                    std::uniform_int_distribution<int> _uniform_int;
                };

                void init_search_space(Eigen::VectorXd dimension_lengths
                        , int number_of_rand_points, double avoidance_width
                        , int number_of_tries_at_time);
                void generate_random_objects(int num_of_objects);
                void insert_obstacles(std::vector<Rect> obstacles);
                void insert_trajectory(std::vector<Rect> trajectory);
                void search_all_obstacles();
                // bool obstacle_free(Rect search_rect);
                void setEnvironment(const dyn_planner::EDTEnvironment::Ptr& env);
                // bool obstacle_free(Eigen::Vector3d search_rect);
                bool obstacle_free(Eigen::Vector3d search_rect, double optimal_time);
                Eigen::Vector3d sample_free();
                Eigen::Vector3d sample();
                std::vector<double> linspace(double start_in, double end_in, double step_size);
                bool collision_free(Eigen::Vector3d start, Eigen::Vector3d end, int r);
                bool collision_free(Eigen::Vector3d start, Eigen::Vector3d end, int r, double optimal_time);
                void insert_obstacle(Eigen::Vector3d index);
                std::vector<Eigen::Vector3d> nearest_obstacles(Eigen::Vector3d x
                                    , int max_neighbours);
                std::vector<Eigen::Vector3d> 
                        nearest_point_on_trajectory(Eigen::Vector3d x
                        , int max_neighbours);
                void generate_samples_from_ellipsoid(Eigen::MatrixXd covmat, Eigen::Matrix3d rotation_mat
                , Eigen::Vector3d cent);
                void generate_search_sapce(Eigen::MatrixXd covmat, Eigen::Matrix3d rotation_mat
                        , Eigen::Vector3d cent, int npts);
                std::vector<SearchSpace::Rect> get_random_obstacles(int number_of_obstacles
                     , Eigen::VectorXd x_dimentions);
                std::vector<double> arange(double start, double stop, double step);
                void save_samples(int index);
                void save_search_space(int index);
                void update_obstacles_map(std::vector<Rect> way_points);
                std::vector<Eigen::Vector3d> nearest_obstacles_to_current_pose(Eigen::Vector3d x
                                , int max_neighbours);
                double get_free_space(Eigen::Vector3d pose, std::vector<Eigen::Vector3d>& obs_poses
                                                    , int num_of_obs);
                double get_free_space(Eigen::Vector3d search_rect, double optimal_time);
                double get_free_space(Eigen::Vector3d pose);
                void insert_vertex(Eigen::Vector3d index);
                std::vector<Eigen::Vector3d> nearest_veties(Eigen::Vector3d x, int max_neighbours);
                
                int dementions = 3;
                Eigen::VectorXd dim_lengths;
                // std::vector<uint64_t> res;
                std::shared_ptr<Eigen::MatrixXd> random_points_tank;
                int number_of_rand_points;
                Random_call* random_call;
                bool use_whole_search_sapce = false;
                double voxel_side_length = 0.1f;
                dyn_planner::EDTEnvironment::Ptr edt_env_;
                struct GeometryRTreeSearchCallback
                {
                    template <typename Value>
                    void operator()(Value const& v)
                    {
                        // return v.is_red();
                        // std::cout<< v.second << std::endl;
                    }
                };

                double *ellipsoid_grid(int n, int ng );
                int ellipsoid_grid_count(int n, Eigen::Vector3d radios,
                                 Eigen::Vector3d center);
                int i4_ceiling (double x);
                void r83vec_print_part(int n, double a[], Eigen::Vector3d center_pose, Eigen::Matrix3d rotation_matrix, std::string file_name);
                void r8mat_write(std::string output_filename, int m, int n, double table[] );
                double r8vec_min(int n, double r8vec[]);
                void generate_points( int n, Eigen::Vector3d radios, Eigen::Vector3d center_pose
                    , Eigen::Matrix3d rotation_matrix);
                void timestamp();

                GeometryRTreeSearchCallback geometry_rtree_callback;
                RTree bg_tree;
                RTree current_trajectory;
                RTree obs_tree;
                std::vector<Rect> random_objects;
                CommonUtils common_utils;
                int number_of_max_attempts;
                int number_of_points_in_random_tank;
                bool is_random_tank_is_ready = false;
                int obstacle_counter = 0;
                double avoidance_width = 0.5;
            private:
                double r[3];
                double c[3];
        };
    }
}
#endif
