#include <plan_env/edt_environment.h>

namespace dyn_planner
{
/* ============================== edt_environment ============================== */
void EDTEnvironment::init()
{
}

void EDTEnvironment::setMap(shared_ptr<SDFMap> map)
{
  this->sdf_map_ = map;
  resolution_inv_ = 1 / sdf_map_->getResolution();
}

std::vector<Eigen::Vector3d> EDTEnvironment::getMapCurrentRange(){
    return this->sdf_map_->getMapCurrentRange(); 
}

bool EDTEnvironment::is_inside_map(Eigen::Vector3d pos){
  return this->sdf_map_->isInMap(pos);
}

std::vector<std::array<double, 6>> EDTEnvironment::get_obs_map(){
  return this->sdf_map_->getObsMap();
}
std::vector<Eigen::Vector3d> EDTEnvironment::nearest_obstacles_to_current_pose(Eigen::Vector3d x
                , int max_neighbours){
                  return this->sdf_map_->nearest_obstacles_to_current_pose(x, max_neighbours);
}

double EDTEnvironment::get_free_distance(Eigen::Vector3d x){
  return this->sdf_map_->get_free_distance(x);
}

// EDTEnvironment::
}  // namespace dyn_planner