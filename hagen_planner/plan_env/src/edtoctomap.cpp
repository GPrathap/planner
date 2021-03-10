#include "plan_env/edtoctomap.h"

namespace hagen_planner
{

  EDTOctoMap::EDTOctoMap(Eigen::Vector3d ori, double resolution, Eigen::Vector3d size)
  {
    this->origin_ = ori;
    this->resolution_sdf_ = resolution;
    this->resolution_inv_ = 1 / resolution_sdf_;
    this->map_size_ = size;
    for (int i = 0; i < 3; ++i){
        grid_size_(i) = ceil(map_size_(i) / resolution_sdf_);
    }
    // cout << "grid num:" << grid_size_.transpose() << endl;
    min_range_ = origin_;
    max_range_ = origin_ + map_size_;
    min_vec_ = Eigen::Vector3i::Zero();
    max_vec_ = grid_size_ - Eigen::Vector3i::Ones();
  }

  void EDTOctoMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
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

  bool EDTOctoMap::isInMap(Eigen::Vector3d pos)
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

  void EDTOctoMap::posToIndex(Eigen::Vector3d pos, Eigen::Vector3i& id)
  {
    for (int i = 0; i < 3; ++i)
      id(i) = floor((pos(i) - origin_(i)) * resolution_inv_);
  }

  void EDTOctoMap::indexToPos(Eigen::Vector3i id, Eigen::Vector3d& pos)
  {
    for (int i = 0; i < 3; ++i)
      pos(i) = (id(i) + 0.5) * resolution_sdf_ + origin_(i);
  }

  std::vector<Eigen::Vector3d> EDTOctoMap::getMapCurrentRange(){
    std::vector<Eigen::Vector3d> current_ranges;
    current_ranges.push_back(min_range_);
    current_ranges.push_back(max_range_);
    return current_ranges;
  }

  void EDTOctoMap::setUpdateRange(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
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

  void EDTOctoMap::loadStaticMap(pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_){
    tree->clear();
    for (int i = 0; i < (int)latest_cloud_->points.size(); i++) {
      octomap::point3d endpoint(latest_cloud_->points[i].x,latest_cloud_->points[i].y, latest_cloud_->points[i].z);
      tree->updateNode(endpoint, true);
    }
    distmap->update(); 
  }

  void EDTOctoMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    /* need odom_ for center radius sensing */
    if (!have_odom_)
    {
      cout << "EDTOctoMap: no odom_" << endl;
      return;
    }
    new_map_ = false;
    pcl::fromROSMsg(*msg, latest_cloud_);

    

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
    tree->clear();
    for (int i = 0; i < (int)latest_cloud_.points.size(); i++) {
      octomap::point3d endpoint(latest_cloud_.points[i].x,latest_cloud_.points[i].y, latest_cloud_.points[i].z);
      tree->updateNode(endpoint, true);
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

              // if (pt_inf.z < 2.0)
              cloud_inflate_vis_.push_back(pt_inf);
            }
          }
        }
      }
      // octocloud.push_back(endpoint);
    }
    distmap->update(); 
    new_map_ = true;
    map_valid_ = true;
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

  void EDTOctoMap::odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    if (msg->child_frame_id == "X" || msg->child_frame_id == "O")
      return;
    odom_ = *msg;
    odom_.header.frame_id = "world";
    have_odom_ = true;
  }

  std::vector<std::array<double, 6>> EDTOctoMap::getObsMap(){
    std::vector<std::array<double, 6>> map;
    return map;
  }

  std::vector<Eigen::Vector3d> EDTOctoMap::nearest_obstacles_to_current_pose(Eigen::Vector3d x
                  , int max_neighbours){
    
  std::vector<Eigen::Vector3d> neighbour_points;
    if(!isInMap(x)){
      std::cout<< "Point outside of the map" << x.transpose() << std::endl;
      return neighbour_points;
    }
    octomap::point3d p(x[0],x[1],x[2]);
    octomap::point3d closestObst;
    float distance;
    distmap->getDistanceAndClosestObstacle(p, distance, closestObst);
    Eigen::Vector3d obs(closestObst.x(), closestObst.y(), closestObst.z());
    neighbour_points.push_back(obs);
    return neighbour_points;
  }

  double EDTOctoMap::get_free_distance(Eigen::Vector3d x){
    octomap::point3d p(x[0],x[1],x[2]);
    double dis = distmap->getDistance(p);
    if(dis <0){
      // cout<< "Pose not inside the map: "<< x.transpose() << endl;
      return 20;
    }
    return dis;
  }

  void EDTOctoMap::get_close_obstacle(Eigen::Vector3d x, Eigen::Vector3d& close_obs, double& dis){
    octomap::point3d p(x[0],x[1],x[2]);
    octomap::point3d closestObst;
    float distance;
    distmap->getDistanceAndClosestObstacle(p, distance, closestObst);
    if(distance <0){
      // cout<< "Pose not inside the map: "<< x.transpose() << endl;
       distance = 20;
       closestObst.x() = x[0]+100;
       closestObst.y() = x[1]+100;
       closestObst.z() = x[2]+100;
    }
    // std::cout<< " dis: " << dis << std::endl;
    // std::cout<< " closestObst" << closestObst << std::endl;
    Eigen::Vector3d obs_pose(closestObst.x(), closestObst.y(), closestObst.z());
    close_obs = obs_pose;
    dis = distance;
  }

  void EDTOctoMap::get_close_obstacles(Eigen::Vector3d pos, std::vector<Eigen::Vector3d>& obs_vec){

  }


  // void EDTOctoMap::get_polyhedral_eql(Eigen::MatrixXd vertices, Eigen::MatrixXd& set_enql){

  //     dd_PolyhedraPtr poly;
  //     dd_MatrixPtr M;
  //     dd_ErrorType err;
  //     dd_MatrixPtr A;
  //     dd_set_global_constants();
  //     dd_rowrange m_input;
  //     dd_colrange d_input;
  //     mytype value;
  //     m_input = vertices.rows();
  //     d_input = vertices.cols();

  //     M=dd_CreateMatrix(m_input, d_input);
  //     M->representation=dd_Generator;
  //     M->numbtype=dd_Real;

  //     // Eigen::VectorXd equl(m_input*d_input);
  //     // // equl << 1.0, 4.0, -1.0, 1.0, 4.0, 5.0, 1.0, 8.0, 3.0;
  //     // equl << 1, 2,-2, 1, 2, 2, 1, -10, 2, 1, -10, -2;

  //     // for(int i=0; i<equl.size(); ++i){
  //     //   dd_set_d(value, equl[i]);
  //     //   int x_ = (int)std::floor(i/d_input);
  //     //   int y_ = (int)i%d_input;
  //     //   dd_set(M->matrix[x_][y_],value);
  //     // }

  //     for(int i=0; i<m_input; i++){
  //       for(int j=0; j< d_input; j++){
  //         dd_set_d(value, vertices(i,j));
  //         dd_set(M->matrix[i][j], value);
  //       }
  //     }

  //     poly=dd_DDMatrix2Poly(M, &err); /* compute the second representation */
  //     A=dd_CopyInequalities(poly);
  //     // set_enql = Eigen::MatrixXd Ab(m_input, d_input);
  //     for(int i=0; i< m_input; i++){
  //       for(int j=0; j< d_input; j++){
  //         set_enql(i,j) = *(A->matrix[i][j]);
  //       }
  //     }
  //     std::cout<< "b:\n " << set_enql.col(0) << std::endl;
  //     std::cout<< "A: \n" << -1*set_enql.block(0,1, m_input, d_input-1) << std::endl;

  //     dd_FreePolyhedra(poly);
  //     dd_FreeMatrix(M);
  //     dd_FreeMatrix(A);
  //     dd_free_global_constants();  /* At the end, this must be called. */
  // }

  void EDTOctoMap::init(ros::NodeHandle& nh)
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
    node_.param("sdf_map/max_avoidance_distance", max_avoidance_distance_, 0.2);

    cout << "origin_: " << origin_.transpose() << endl;
    cout << "map size: " << map_size_.transpose() << endl;
    cout << "resolution: " << resolution_sdf_ << endl;

    /* ---------- sub and pub ---------- */
    odom_sub_ = node_.subscribe<nav_msgs::Odometry>("/odom_world", 10, &EDTOctoMap::odomCallback, this);
    cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1, &EDTOctoMap::cloudCallback, this);
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
    tree = new octomap::OcTree(resolution_sdf_);
    octomap::point3d max_(max_range_[0], max_range_[1], max_range_[2]);
    octomap::point3d min_(min_range_[0], min_range_[1], min_range_[2]);
    tree->setBBXMax(max_);
    tree->setBBXMin(min_);
    distmap = new DynamicEDTOctomap(update_range_, tree, min_, max_, false);
  }
}  // namespace hagen_planner
