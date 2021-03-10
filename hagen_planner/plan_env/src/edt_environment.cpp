#include <plan_env/edt_environment.h>

namespace hagen_planner
{
void EDTEnvironment::init()
{
}

void EDTEnvironment::setMap(shared_ptr<EDTOctoMap> map)
{
  this->sdf_map_ = map;
  resolution_inv_ = 1 / sdf_map_->getResolution();
}

std::vector<Eigen::Vector3d> EDTEnvironment::getMapCurrentRange(){
    return this->sdf_map_->getMapCurrentRange(); 
}

double EDTEnvironment::getResolution(){
  return this->sdf_map_->getResolution();
}

int EDTEnvironment::getInflateOccupancy(Eigen::Vector3d pos) {
    if(is_inside_map(pos)){
      double dis = get_free_distance(pos);
      if(dis>0.5){
        return 0;
      }else{
        return 1;
      }
    }
    return -1;
}

bool EDTEnvironment::is_inside_map(Eigen::Vector3d pos){
  return this->sdf_map_->isInMap(pos);
}

std::vector<std::array<double, 6>> EDTEnvironment::get_obs_map(){
  return this->sdf_map_->getObsMap();
}

void EDTEnvironment::get_close_obstacles(Eigen::Vector3d pos, std::vector<Eigen::Vector3d>& obs_vec){
  this->sdf_map_->get_close_obstacles(pos, obs_vec);
}

std::vector<Eigen::Vector3d> EDTEnvironment::nearest_obstacles_to_current_pose(Eigen::Vector3d x
                , int max_neighbours){
                  return this->sdf_map_->nearest_obstacles_to_current_pose(x, max_neighbours);
}

void EDTEnvironment::get_close_obstacle(Eigen::Vector3d x, Eigen::Vector3d& close_obs, double& dis){
  this->sdf_map_->get_close_obstacle(x, close_obs, dis);
}

double EDTEnvironment::get_free_distance(Eigen::Vector3d x){
  return this->sdf_map_->get_free_distance(x);
}

bool EDTEnvironment::collision_free(Eigen::Vector3d start, Eigen::Vector3d end, double min_dis, double avoidance_dis){
  std::vector<Eigen::Vector3d> poses = math_util.next_poses(start, end, min_dis);
  for(auto pose : poses){
    if(this->sdf_map_->get_free_distance(pose)< avoidance_dis){
      return false;
    }
  }
  return true;
}

//http://corysimon.github.io/articles/uniformdistn-on-sphere/
bool EDTEnvironment::get_projected_point(Eigen::Vector3d center, double radius, double avoidance_dis,
                Eigen::Vector3d& projected_pose){
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 generator (seed);
    std::uniform_real_distribution<double> uniform01(0.0, 1.0);
    std::cout << "Generating intermediate goal..." << std::endl;
    double min_z = getMapCurrentRange()[0](2);
    double max_z = getMapCurrentRange()[1](2);
    for (int i = 0; i < 2000; i++) {
        // incorrect way
        double theta = 2 * M_PI * uniform01(generator);
        double phi = acos(1 - 1 * uniform01(generator));
        double x = radius*sin(phi) * cos(theta);
        double y = radius*sin(phi) * sin(theta);
        double z = radius*cos(phi);
        Eigen::Vector3d end_pose(x, y, z);
        end_pose = end_pose + center;
        if(collision_free(center, end_pose, 0.2, avoidance_dis)){
          if(min_z > end_pose[2]){
            end_pose[2] =  min_z; 
          }
          if(max_z < end_pose[2]){
            end_pose[2] =  max_z; 
          }
          Eigen::Vector3d end_;
          double dis_obs;
          get_close_obstacle(end_pose, end_, dis_obs);
          if(dis_obs>avoidance_dis){
            projected_pose = end_pose;
            return true;
          }
        }
    }
    for (int i = 0; i < 2000; i++) {
        // incorrect way
        double theta = 2 * M_PI * uniform01(generator);
        double phi = acos(-1 * uniform01(generator));
        double x = radius*sin(phi) * cos(theta);
        double y = radius*sin(phi) * sin(theta);
        double z = radius*cos(phi);
        Eigen::Vector3d end_pose(x, y, z);
        end_pose = end_pose + center;
        if(collision_free(center, end_pose, 0.2, avoidance_dis)){
          if(min_z > end_pose[2]){
            end_pose[2] =  min_z; 
          }
          if(max_z < end_pose[2]){
            end_pose[2] =  max_z; 
          }
          Eigen::Vector3d end_;
          double dis_obs;
          get_close_obstacle(end_pose, end_, dis_obs);
          if(dis_obs>avoidance_dis){
            projected_pose = end_pose;
            return true;
          }
        }
    }
    return false;
}

// https://stackoverflow.com/questions/48703275/3d-truncated-cone-in-python
bool EDTEnvironment::get_projected_point(Eigen::Vector3d p0, Eigen::Vector3d p1, double inner_radius
        , double outer_radius, double avoidance_dis,  double avoidance_dis_max, Eigen::Vector3d& projected_pose){
  Eigen::Vector3d v = p1 - p0;
  double mag = v.norm();
  v = v/mag;
  Eigen::Vector3d not_v(1,1,0);
  if( v==not_v){
    not_v<< 0, 1, 0;
  }
  Eigen::Vector3d n1 = v.cross(not_v);
  n1 = n1/n1.norm();
  Eigen::Vector3d n2 = v.cross(n1);
  Eigen::MatrixXd t = Eigen::VectorXd::LinSpaced(num_of_points, 0, mag).replicate(1, num_of_points);
  Eigen::MatrixXd theta = Eigen::VectorXd::LinSpaced(num_of_points, 0, 2*M_PI).replicate(1, num_of_points).transpose();
  Eigen::VectorXd R;
  R.setLinSpaced(num_of_points, inner_radius, outer_radius);
  Eigen::MatrixXd X(num_of_points, num_of_points);
  Eigen::MatrixXd Y(num_of_points, num_of_points);
  Eigen::MatrixXd Z(num_of_points, num_of_points);

  Eigen::MatrixXd sin_values = theta.array().sin();
  Eigen::MatrixXd cos_values = theta.array().cos();

  int i = 0;
  Eigen::MatrixXd sin_on_vec = sin_values*n1[i];
  Eigen::MatrixXd cos_on_vec = cos_values*n2[i];
  for(int j=0; j<num_of_points; j++){
    X.row(j) = p0[i] + (v[i] * t.col(j).array() + R.array()*sin_on_vec.col(j).array() + R.array()*cos_on_vec.col(j).array()).array();
  }
  i = 1;
  sin_on_vec = sin_values*n1[i];
  cos_on_vec = cos_values*n2[i];
  for(int j=0; j<num_of_points; j++){
    Y.row(j) = p0[i] + (v[i] * t.col(j).array() + R.array()*sin_on_vec.col(j).array() + R.array()*cos_on_vec.col(j).array()).array();
  }
  i = 2;
  sin_on_vec = sin_values*n1[i];
  cos_on_vec = cos_values*n2[i];
  for(int j=0; j<num_of_points; j++){
    Z.row(j) = p0[i] + (v[i] * t.col(j).array() + R.array()*sin_on_vec.col(j).array() + R.array()*cos_on_vec.col(j).array()).array();
  }

  Eigen::MatrixXd search_space_(X.rows(), 3*X.cols());
  search_space_ << X, Y, Z;
  search_space = search_space_;

  double min_z = getMapCurrentRange()[0](2);
  double max_z = getMapCurrentRange()[1](2);

  for (int i = 0; i < (int)X.rows(); i++) {
    for (int j = 0; j < (int)X.cols(); j++) {
        Eigen::Vector3d end_pose(X(i, j), Y(i, j), Z(i, j));
        // end_pose = end_pose + p0;
        if(collision_free(p0, end_pose, 0.2, avoidance_dis)){
          if(min_z > end_pose[2]){
            end_pose[2] =  min_z;
          }
          if(max_z < end_pose[2]){
            end_pose[2] =  max_z;
          }
          Eigen::Vector3d end_;
          double dis_obs;
          get_close_obstacle(end_pose, end_, dis_obs);
          if(dis_obs>avoidance_dis_max){
            projected_pose = end_pose;
            return true;
          }
        }
    }
  }
  return false;
}

void EDTEnvironment::get_constraint_polyhedra(Eigen::Vector3d pose){
  Eigen::VectorXd info(5);
  info(0) = pose(1);
  info(1) = pose(2);
  info(2) = this->sdf_map_->max_avoidance_distance_;
  info(3) = this->sdf_map_->max_avoidance_distance_;
  info(4) = 0.25;
  info(5) = pose(0);
  std::vector<Eigen::Vector3d> poses;
  math_util.get_squar_segments(info, poses);
  // Eigen::Vector3d end_;
  // double dis_obs;
  // get_close_obstacle(pose, end_, dis_obs);
  // double min_dis_on_z = std::abs(end_(2) - pose(2));
}

void EDTEnvironment::get_cone_points(Eigen::MatrixXd& poses){
  poses = search_space;
}

}