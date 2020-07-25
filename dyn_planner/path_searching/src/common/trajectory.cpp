
#include "trajectory.h"


namespace kamaz {
namespace hagen {

  void Trajectory::print_target_trajectory(std::queue<Eigen::Vector3d> target_trajectory){
    const Eigen::IOFormat fmt(4, Eigen::DontAlignCols, "\t", " ", "", "", "");
    while (!target_trajectory.empty()){
      proposed_trajectory.push_back(target_trajectory.front().transpose());
      std::cout << target_trajectory.front().transpose().format(fmt) << std::endl;
      target_trajectory.pop();
	  }
  }

  void Trajectory::print_target_trajectory(std::vector<Eigen::Vector3d> target_trajectory){
    const Eigen::IOFormat fmt(4, Eigen::DontAlignCols, "\t", " ", "", "", "");
    for(auto &item : target_trajectory){
      
      std::cout << item.transpose().format(fmt) << std::endl;
    }
  }

  void Trajectory::generate_target_trajectory(std::vector<Eigen::Vector3d>&  target_trajectory
  , std::string trajectory_to_be_flown_file_name){
    Eigen::Vector3d path_position(4);
    std::fstream infile;
	  infile.open(trajectory_to_be_flown_file_name, std::ios::in);
    std::string line, word, temp;
    std::string delimiter = ",";
    while (std::getline(infile, line)) {
      size_t pos = 0;
      std::string token;
      int index = 0;
      while ((pos = line.find(delimiter)) != std::string::npos) {
          token = line.substr(0, pos);
          path_position(index) = std::atof(token.c_str());
          index++;
          line.erase(0, pos + delimiter.length());
      }
      path_position(index) = std::atof(line.c_str());
      path_position(3) = 1.0;
		  target_trajectory.push_back(path_position);
    }
    return;
  }
}
}

   