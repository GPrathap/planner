#include <traj_utils/planning_saving.h>

using std::cout;
using std::endl;
namespace hagen_planner
{
PlanningSaving::PlanningSaving(ros::NodeHandle& nh)
{
  node = nh;
  nh.param<std::string>("traj_utils/root_dir", root_dir, "");
}

void PlanningSaving::save_double_vector(vector<vector<double>> trees_, std::string file_name, int index){
  std::vector<double> edges; 
  // int count = 0;
  // cout<< trees_.size() << "  " << trees_[0].size() << endl;
  for(int k=0; k< (int)trees_.size(); k++){
    for(int j=0; j< (int)trees_[0].size(); j++){
        edges.push_back(trees_[k][j]);
    }
  }
  std::string storing_location = root_dir + file_name;
  if(edges.size()>0){
    cnpy::npy_save(storing_location, &edges[0],{(unsigned int)trees_.size(), (unsigned int)trees_[0].size()/index, (unsigned int)index},"w");
  }else{
    cout<< "No data to save " << storing_location << endl;
  }
}

void PlanningSaving::save_dm(DM trees, std::string file_name, int index){
        std::vector<double> edges; 
        for(int k=0; k< (int)trees.size1(); k++){
          for(int j=0; j< trees.size2(); j++){
              edges.push_back((double)trees(k,j));
          }
        }
        std::string storing_dir = root_dir + file_name;
        if(edges.size()>0){
            cnpy::npy_save(storing_dir, &edges[0],{(unsigned int)trees.size1(), (unsigned int)trees.size2()/index, (unsigned int)index},"w");
        }else{
            cout<< "no data to save" << storing_dir << endl;
        }
}


void PlanningSaving::save_vector(vector<double> trees, std::string file_name, int index){
        std::vector<double> edges; 
        for(int k=0; k< (int)trees.size(); k++){
              edges.push_back(trees[k]);
        }
        std::string storing_dir = root_dir + file_name;
        if(edges.size()>0){
            cnpy::npy_save(storing_dir, &edges[0],{(unsigned int)1, (unsigned int)trees.size(), (unsigned int)index},"w");
        }else{
            cout << "No data to save" << storing_dir << endl;
        }
}

// PlanningSaving::
}  // namespace hagen_planner