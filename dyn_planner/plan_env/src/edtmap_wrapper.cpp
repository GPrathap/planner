#include <plan_env/edtmap_wrapper.h>

namespace dyn_planner
{
/* ============================== edt_environment ============================== */
void EDTMapWrapper::init(ros::NodeHandle& node_)
{
    node_.param("sdf_map/origin_x", origin_(0), -20.0);
    node_.param("sdf_map/origin_y", origin_(1), -20.0);
    node_.param("sdf_map/origin_z", origin_(2), 0.0);

    node_.param("sdf_map/map_size_x", map_size_(0), 40.0);
    node_.param("sdf_map/map_size_y", map_size_(1), 40.0);
    node_.param("sdf_map/map_size_z", map_size_(2), 5.0);

    node_.param("sdf_map/resolution_sdf", resolution_sdf_, 0.2);
    min_range_ = origin_;
    resolution_inv_ = 1/resolution_sdf_;
    max_range_ = min_range_ + map_size_;

    for (int i = 0; i < 3; ++i)
        grid_size_(i) = std::ceil(map_size_(i) / resolution_sdf_);
 
    current_ranges.push_back(min_range_);
    current_ranges.push_back(max_range_);

    std::cout << "edt wrapper : origin_: " << origin_.transpose() << std::endl;
    std::cout << "edt wrapper : map size: " << map_size_.transpose() << std::endl;
    std::cout << "edt wrapper : resolution: " << resolution_sdf_ << std::endl;
    odom_valid_ = false; 
    map_valid_ = false;
    have_odom_ = false;

    // edtmap_sub_ = node_.subscribe("/sdf_map/edtmap", 1, &EDTMapWrapper::edtmapCallback, this);
}

nav_msgs::Odometry EDTMapWrapper::getOdom(){ 
  return odom_; 
}

void EDTMapWrapper::setOdom(const nav_msgs::OdometryConstPtr& msg){ 
  odom_ = *msg;
  odom_.header.frame_id = "world";
  // have_odom_ = true;
}

// void EDTMapWrapper::edtmapCallback(const edtmap_msg::EDTMap::ConstPtr& msg){
//     this->edt_map_ = msg->data;
// }

void EDTMapWrapper::setMap(std::vector<double> map, bool map_valid, bool odom_valid){
  this->edt_map_ = map;
  map_valid_ = map_valid;
  have_odom_ = odom_valid;
}

bool EDTMapWrapper::odomValid(){
  return have_odom_;
}

bool EDTMapWrapper::mapValid(){
  return map_valid_;
}

bool EDTMapWrapper::isInMap(Eigen::Vector3d pos)
{
  if (pos(0) < min_range_(0) + 1e-4 || pos(1) < min_range_(1) + 1e-4 || pos(2) < min_range_(2) + 1e-4)
  {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > max_range_(0) - 1e-4 || pos(1) > max_range_(1) - 1e-4 || pos(2) > max_range_(2) - 1e-4)
  {
    // cout << "larger than max range!" << endl;
    return false;
  }
  return true;
}

void EDTMapWrapper::posToIndex(Eigen::Vector3d pos, Eigen::Vector3i& id)
{
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - origin_(i)) * resolution_inv_);
}


std::vector<Eigen::Vector3d> EDTMapWrapper::getMapCurrentRange(){
    return current_ranges;
}

void EDTMapWrapper::getMapRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size){
    ori = origin_;
    size = map_size_;
}

std::vector<Eigen::Vector3d> EDTMapWrapper::nearest_obstacles_to_current_pose(Eigen::Vector3d x, int max_neighbours){
    std::vector<Eigen::Vector3d> neighbour_points;
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
    return neighbour_points;  
}

double EDTMapWrapper::getDistance(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return -1;
  Eigen::Vector3i id;
  posToIndex(pos, id);
  // (x, y, z) -> x*ny*nz + y*nz + z
  int index = id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2);
  if(edt_map_.size()>index){
    return edt_map_[index];
  }
  return 1000;
}

double EDTMapWrapper::evaluateCoarseEDT(const Eigen::Vector3d& pos, const double& time)
{
  double d1 = getDistance(pos);
  return d1;
}

// EDTEnvironment::
}  // namespace dyn_planner