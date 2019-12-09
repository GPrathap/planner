#ifndef TRAJECTORY_PLANNING
#define TRAJECTORY_PLANNING

#include <stack>
#include <vector>
#include <array>
#include <iostream>
#include <cmath>
#include <set>
#include <cstdlib>
#include <climits>
#include <cerrno>
#include <cfenv>
#include <cstring>
#include <cnpy.h>
#include <stack> 
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <Eigen/Eigenvalues> 
#include <complex>
#include <fstream>
#include <random>
#include <cmath>
#include <chrono>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>

namespace kamaz {
namespace hagen {
        class TrajectoryPlanning {
            public:
                TrajectoryPlanning(double speed = 2);
                ~TrajectoryPlanning() = default;

                bool generate_ts(std::vector<Eigen::VectorXd> path);
                void traj_opt7();
                void save_status(std::vector<std::vector<Eigen::VectorXd>>
                        , std::string file_name);
                void save_trajectory(std::vector<Eigen::VectorXd> trajectory_of_drone
                ,  std::string file_name);

                void get_desired_state(double time
                        , std::vector<Eigen::VectorXd>& states);
                std::pair<double, int > closest(double value);

                void generate_target_trajectory(std::vector<Eigen::VectorXd>&  target_trajectory
                       , std::string trajectory_to_be_flown_file_name);

                std::vector<double> time_segs;

                Eigen::MatrixXd way_points;
                double total_time;
                double max_speed;
                Eigen::MatrixXd X;
                Eigen::MatrixXd A;
                Eigen::MatrixXd Y;
                bool is_set;
                bool current_time_is_set = false;

            private:
            };
    }
}
#endif