#ifndef PATH_PLANNER_TRAJECTORY_H_
#define PATH_PLANNER_TRAJECTORY_H_

#include <random>
#include <cmath>
#include <chrono>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include "../include/colours.h"
#include <Eigen/Dense>
#include <queue>

namespace kamaz {
namespace hagen {

class Trajectory{
 public:
  Trajectory() = default;
  ~Trajectory() = default;

  void generate_target_trajectory(std::vector<Eigen::Vector3d>&, std::string file_name);
  void print_target_trajectory(std::queue<Eigen::Vector3d>);
  void print_target_trajectory(std::vector<Eigen::Vector3d>);
  std::vector<Eigen::Vector3d> proposed_trajectory;
  std::string trajectory_to_be_flown;
};
} 
} 

#endif 
