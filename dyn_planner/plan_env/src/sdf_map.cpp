#include "plan_env/sdf_map.h"

namespace dyn_planner
{
SDFMap::SDFMap(Eigen::Vector3d ori, double resolution, Eigen::Vector3d size)
{
  this->origin_ = ori;
  this->resolution_sdf_ = resolution;
  this->resolution_inv_ = 1 / resolution_sdf_;
  this->map_size_ = size;
  for (int i = 0; i < 3; ++i)
    grid_size_(i) = ceil(map_size_(i) / resolution_sdf_);
  // cout << "grid num:" << grid_size_.transpose() << endl;
  min_range_ = origin_;
  max_range_ = origin_ + map_size_;
  min_vec_ = Eigen::Vector3i::Zero();
  max_vec_ = grid_size_ - Eigen::Vector3i::Ones();
}

void SDFMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
  min_pos(0) = max(min_pos(0), min_range_(0));
  min_pos(1) = max(min_pos(1), min_range_(1));
  min_pos(2) = max(min_pos(2), min_range_(2));
  max_pos(0) = min(max_pos(0), max_range_(0));
  max_pos(1) = min(max_pos(1), max_range_(1));
  max_pos(2) = min(max_pos(2), max_range_(2));
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos - Eigen::Vector3d(resolution_sdf_ / 2, resolution_sdf_ / 2, resolution_sdf_ / 2), max_id);
}

bool SDFMap::isInMap(Eigen::Vector3d pos)
{
  if (pos(0) < min_range_(0) + 1e-4 || pos(1) < min_range_(1) + 1e-4 || pos(2) < min_range_(2) + 1e-4)
  {
    return false;
  }
  if (pos(0) > max_range_(0) - 1e-4 || pos(1) > max_range_(1) - 1e-4 || pos(2) > max_range_(2) - 1e-4)
  {
    return false;
  }
  return true;
}

void SDFMap::posToIndex(Eigen::Vector3d pos, Eigen::Vector3i& id)
{
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - origin_(i)) * resolution_inv_);
}

void SDFMap::indexToPos(Eigen::Vector3i id, Eigen::Vector3d& pos)
{
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * resolution_sdf_ + origin_(i);
}

std::vector<Eigen::Vector3d> SDFMap::getMapCurrentRange(){
  std::vector<Eigen::Vector3d> current_ranges;
  current_ranges.push_back(min_range_);
  current_ranges.push_back(max_range_);
  return current_ranges;
}

void SDFMap::setUpdateRange(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
  min_pos(0) = max(min_pos(0), min_range_(0));
  min_pos(1) = max(min_pos(1), min_range_(1));
  min_pos(2) = max(min_pos(2), min_range_(2));
  max_pos(0) = min(max_pos(0), max_range_(0));
  max_pos(1) = min(max_pos(1), max_range_(1));
  max_pos(2) = min(max_pos(2), max_range_(2));
  posToIndex(min_pos, min_vec_);
  posToIndex(max_pos - Eigen::Vector3d(resolution_sdf_ / 2, resolution_sdf_ / 2, resolution_sdf_ / 2), max_vec_);
}

void SDFMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  /* need odom_ for center radius sensing */
  if (!have_odom_)
  {
    cout << "SDFMap: no odom_" << endl;
    return;
  }
  pcl::fromROSMsg(*msg, latest_cloud_);
  new_map_ = true;
}

void SDFMap::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  if (msg->child_frame_id == "X" || msg->child_frame_id == "O")
    return;
  odom_ = *msg;
  odom_.header.frame_id = "world";
  have_odom_ = true;
}

std::vector<std::array<double, 6>> SDFMap::getObsMap(){
  return _objects_map;
}

void SDFMap::updateCallback(const ros::TimerEvent& e)
{
  int static no_maps = 0;
  if (!new_map_)
  {
    // cout << "no new map." << endl;
    // obs_tree.clear();
    no_maps++;
    if(no_maps>map_clear_duration){
      obs_tree.clear();
      no_maps = 0;
    }
    return;
  }
  map_valid_ = true;
  new_map_ = false;
  no_maps = 0;
  if (latest_cloud_.points.size() == 0)
    return;
  Eigen::Vector3d center(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);
  if (isnan(center(0)) || isnan(center(1)) || isnan(center(2)))
    return;
  /* ---------- inflate cloud and insert to SDFMap ---------- */
  Eigen::Vector3d disp(update_range_, update_range_, update_range_ / 2.0);
  this->resetBuffer(center - disp, center + disp);
  cloud_inflate_vis_.clear();
  pcl::PointXYZ pt, pt_inf;
  Eigen::Vector3d p3d, p3d_inf;
  const int ifn = ceil(inflate_ * resolution_inv_);
  int obs_counter = 0;
  obs_tree_previous.clear();
  _objects_map.clear();
  double totat_time_rtree_buffer = 0;
  auto t1 = std::chrono::high_resolution_clock::now();
  for (size_t i = 0; i < latest_cloud_.points.size(); ++i)
  {
    pt = latest_cloud_.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;
    double dist_to_obs = (center - p3d).norm();
    /* point inside update range */
    if ( dist_to_obs < update_range_)
    {
      /* inflate the point */
      for (int x = -ifn; x <= ifn; ++x){
        for (int y = -ifn; y <= ifn; ++y){
          for (int z = -ifn; z <= ifn; ++z)
          {
            p3d_inf(0) = pt_inf.x = pt.x + x * resolution_sdf_;
            p3d_inf(1) = pt_inf.y = pt.y + y * resolution_sdf_;
            p3d_inf(2) = pt_inf.z = pt.z + 0.5 * z * resolution_sdf_;

            if (pt_inf.z < 2.0)
              cloud_inflate_vis_.push_back(pt_inf);
          }
        }
      }
      box_t b(point_t(pt.x, pt.y, pt.z), point_t(pt.x+resolution_sdf_
                            , pt.y+resolution_sdf_, pt.z+resolution_sdf_));
      obs_tree_previous.insert(value_t(b, obs_counter));
      std::array<double, 6> cube = {pt.x, pt.y, pt.z, pt.x+resolution_sdf_
                                  , pt.y+resolution_sdf_, pt.z+resolution_sdf_};
      _objects_map.push_back(cube);
      obs_counter++;
    }
    if(dist_to_obs<= 0.5){
      // std::cout<< p3d.transpose() << "==========------" << "-----" << p3d << "-----"  << std::endl;
    }
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  totat_time_rtree_buffer = std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count();
  // std::cout << "time----" << std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() << std::endl;
  // std::ofstream outfile;
  // outfile.open("/home/geesara/tmp/data/map_building_time_stamps_rtee.txt", std::ios_base::app);
  // outfile << "rtree,"<<  totat_time_rtree_buffer <<"\n";
  obs_tree = obs_tree_previous;
  cloud_inflate_vis_.width = cloud_inflate_vis_.points.size();
  cloud_inflate_vis_.height = 1;
  cloud_inflate_vis_.is_dense = true;
  cloud_inflate_vis_.header.frame_id = "world";
  cloud_inflate_vis_.header.seq = latest_cloud_.header.seq;
  cloud_inflate_vis_.header.stamp = latest_cloud_.header.stamp;
  sensor_msgs::PointCloud2 map_inflate_vis;
  pcl::toROSMsg(cloud_inflate_vis_, map_inflate_vis);
  inflate_cloud_pub_.publish(map_inflate_vis);
  this->setUpdateRange(center - disp, center + disp);
}

std::vector<Eigen::Vector3d> SDFMap::nearest_obstacles_to_current_pose(Eigen::Vector3d x
                , int max_neighbours){
    // std::vector<value_t> returned_values;
    std::vector<Eigen::Vector3d> neighbour_points;
    for ( RTree::const_query_iterator it = obs_tree.qbegin(bgi::nearest(point_t(x[0], x[1], x[2]), max_neighbours)) ;
            it != obs_tree.qend() ; ++it )
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

double SDFMap::get_free_distance(Eigen::Vector3d x){
  if(!isInMap(x)){
    ROS_WARN_STREAM("Requested pose is not inside the map" << x.transpose());
    return 0.0;
  }
  std::vector<Eigen::Vector3d> poses = nearest_obstacles_to_current_pose(x, 1);
  if(poses.size()>0){
    return (x - poses[0]).norm();
  }else{
    return 1000;
  }
}

void SDFMap::init(ros::NodeHandle& nh)
{
  node_ = nh;

  /* ---------- param ---------- */
  node_.param("sdf_map/origin_x", origin_(0), -20.0);
  node_.param("sdf_map/origin_y", origin_(1), -20.0);
  node_.param("sdf_map/origin_z", origin_(2), 0.0);

  node_.param("sdf_map/map_size_x", map_size_(0), 40.0);
  node_.param("sdf_map/map_size_y", map_size_(1), 40.0);
  node_.param("sdf_map/map_size_z", map_size_(2), 5.0);

  node_.param("sdf_map/resolution_sdf", resolution_sdf_, 0.2);
  node_.param("sdf_map/ceil_height", ceil_height_, 2.0);
  node_.param("sdf_map/update_rate", update_rate_, 10.0);
  node_.param("sdf_map/update_range", update_range_, 5.0);
  node_.param("sdf_map/inflate", inflate_, 0.2);
  node_.param("sdf_map/radius_ignore", radius_ignore_, 0.2);
  node_.param("sdf_map/map_clear_duration", map_clear_duration, 10);

  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_.transpose() << endl;
  cout << "resolution: " << resolution_sdf_ << endl;

  /* ---------- sub and pub ---------- */
  odom_sub_ = node_.subscribe<nav_msgs::Odometry>("/odom_world", 10, &SDFMap::odomCallback, this);
  cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1, &SDFMap::cloudCallback, this);
  update_timer_ = node_.createTimer(ros::Duration(0.1), &SDFMap::updateCallback, this);
  inflate_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/inflate_cloud", 1);

  /* ---------- setting ---------- */
  have_odom_ = false;
  new_map_ = false;
  map_valid_ = false;

  resolution_inv_ = 1 / resolution_sdf_;
  for (int i = 0; i < 3; ++i)
    grid_size_(i) = ceil(map_size_(i) / resolution_sdf_);
  // SETY << "grid num:" << grid_size_.transpose() << REC;
  min_range_ = origin_;
  max_range_ = origin_ + map_size_;
  min_vec_ = Eigen::Vector3i::Zero();
  max_vec_ = grid_size_ - Eigen::Vector3i::Ones();
}
}  // namespace dyn_planner
