#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <atomic>

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
// #include <cnpy.h>
#include<complex>
#include<ctime>
#include<cstdlib>
#include<iostream>
#include<map>
#include <traj_utils/planning_visualization.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <path_searching/kinodynamic_rrt_star.h>
#include <plan_manage/dyn_planner_manager.h>
#include "plan_manage/Bspline.h"
// #include <dyn_planner/obj_predictor.h>
// #include <dyn_planner/planning_visualization.h>
// #include <dyn_planner/edt_environment.h>
#include <plan_manage/planning_fsm.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <octomap_msgs/conversions.h>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <octomap/octomap.h>
#include <plan_manage/backward.hpp>
namespace backward
{
  backward::SignalHandling sh;
}

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace dyn_planner;
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dyn_planner_validator_node");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");
  PlanningFSM fsm;

  int init_sys = false;

  ros::Publisher pub_point_cloud, pub_trajectory, octomap_publisher_;

  dyn_planner::SDFMap::Ptr sdf_map_;
  dyn_planner::EDTEnvironment::Ptr edt_env_;
  dyn_planner::KinodynamicRRTstar::Ptr path_finder_;

  edt_env_.reset(new dyn_planner::EDTEnvironment);
  sdf_map_.reset(new dyn_planner::SDFMap);
  sdf_map_->init(nh);

  path_finder_.reset(new dyn_planner::KinodynamicRRTstar);
  path_finder_->setParam(nh);
 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<std::vector<string>> records;
  ifstream fin1;
  std::vector<string> row;
  string line1;

  pub_point_cloud = nh.advertise<PointCloud> ("/plan_validate/pointcloud", 1);
  pub_trajectory = node.advertise<visualization_msgs::Marker>("/plan_validate/trajectory", 1);
  octomap_publisher_ = node.advertise<octomap_msgs::Octomap>("/plan_validate/octomap", 1);

  // Open an existing file
  fin1.open("/root/catkin_ws/src/planner/Fast-Planner/dyn_planner/plan_manage/data/start_and_end.csv");
  while(!fin1.eof()){
              fin1>>line1;
              string line, word, temp; 
              stringstream s(line1); 
              row.clear(); 
              while (getline(s, word, ',')) { 
                row.push_back(word); 
              }
              records.push_back(row);
  }
    
  int counter_record = 0;
  for(auto record : records){
      
      std::string pcd_path = "/root/catkin_ws/src/forest_gen/octomaps/forest" + record[1] + ".pcd";
      std::string octo_path = "/root/catkin_ws/src/forest_gen/octomaps/forest" + record[1] + ".bt";

      octomap::OcTree* octree = new octomap::OcTree(octo_path);

      octomap_msgs::Octomap map;
       map.header.frame_id = "world";
       octomap_msgs::binaryMapToMsg(*octree, map);
       octomap_publisher_.publish(map);
 
     

      std::cout<< pcd_path << std::endl;
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_path, *cloud) == -1) //* load the file
      {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
      }
      
      PointCloud::Ptr msg (new PointCloud);
      msg->header.frame_id = "world";
      int obs_counter = 0;
      for (std::size_t i = 0; i < cloud->points.size (); ++i){
          auto pt = cloud->points[i];
          // double dist_to_obs = (center - p3d).norm();
          /* point inside update range */
          double half_resolution = sdf_map_->resolution_sdf_/2.0;
          dyn_planner::box_t b(dyn_planner::point_t(pt.x- half_resolution, pt.y- half_resolution, pt.z- half_resolution)
          , dyn_planner::point_t(pt.x + half_resolution, pt.y + half_resolution, pt.z + half_resolution));
          sdf_map_->obs_tree_previous.insert(dyn_planner::value_t(b,  obs_counter));
          std::array<double, 6> cube = {pt.x - half_resolution, pt.y - half_resolution, pt.z - half_resolution, pt.x + half_resolution
                                  , pt.y + half_resolution, pt.z + half_resolution};
          sdf_map_->_objects_map.push_back(cube);
          obs_counter++;
          msg->points.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
      }

      pub_point_cloud.publish(msg);
    
      sdf_map_->obs_tree = sdf_map_->obs_tree_previous;
      edt_env_->setMap(sdf_map_);

      if(!init_sys){
          path_finder_->setEnvironment(edt_env_);
          path_finder_->init();
      }else{
        path_finder_->setEnvironment(edt_env_);
      }
     
      std::cout << std::stod(record[2]) << " " << std::stod(record[3]) << " " << std::stod(record[4]) << " " << std::stod(record[5]) << " " << std::stod(record[6]) << " " << std::stod(record[7]) << " " << std::endl;
      Eigen::Vector3d start_pt(std::stod(record[2]), std::stod(record[3]), std::stod(record[4]));
      Eigen::Vector3d start_vel(0,0,0);
      Eigen::Vector3d end_pt(std::stod(record[5])	, std::stod(record[6])	, std::stod(record[7]));
      Eigen::Vector3d end_vel(0,0,0);
      Eigen::Vector3d start_acc(0,0,0);
      
      std::ofstream outfile;
      outfile.open("/dataset/rrt_old/time_stamps_comparison.txt", std::ios_base::app);
      const clock_t begin_time = clock();
      std::atomic_bool is_allowed_to_run = ATOMIC_VAR_INIT(true); 
      int status = path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true, is_allowed_to_run,  true, 0.0 , 0, counter_record);
      double time_diff =  double( clock () - begin_time ) /  CLOCKS_PER_SEC;
      double dis_ratio = -1;
      if(status <3){
          int K_rrt;
          double ts_rrt = 0.1;
          Eigen::MatrixXd vel_acc;
          Eigen::MatrixXd samples_rrt = path_finder_->getSamplesRRT(ts_rrt, K_rrt);


          visualization_msgs::Marker mk;
          mk.header.frame_id = "world";
          mk.header.stamp = ros::Time::now();
          mk.type = visualization_msgs::Marker::LINE_LIST;
          mk.action = visualization_msgs::Marker::DELETE;
          mk.id = 3;
          mk.action = visualization_msgs::Marker::ADD;
          mk.pose.orientation.x = 0.0, mk.pose.orientation.y = 0.0, mk.pose.orientation.z = 0.0, mk.pose.orientation.w = 1.0;
          mk.color.r = 0.4, mk.color.g = 0.5, mk.color.b = 0.7, mk.color.a = 1;
          mk.scale.x = 0.2, mk.scale.y = 0.2, mk.scale.z = 0.2;
          geometry_msgs::Point pt;
          double distance = 0.0f;
         
          if(samples_rrt.cols() > 1){
            Eigen::VectorXd previous = samples_rrt.col(0);
            pt.x = previous[0], pt.y = previous[1], pt.z = previous[2];
            mk.points.push_back(pt);
            for (int i = 1; (unsigned)i < samples_rrt.cols(); i++){
                double dis = std::abs((previous - samples_rrt.col(i)).norm());
                previous = samples_rrt.col(i);
                if(i < samples_rrt.cols()-3){
                    pt.x = previous[0], pt.y = previous[1], pt.z = previous[2];
                    mk.points.push_back(pt);
                }
                distance += dis;
            }
          }
          pub_trajectory.publish(mk);
          double dis_ratio = (start_pt - end_pt).norm();
          dis_ratio = distance / dis_ratio ;
          outfile << "rtree,"<<  time_diff << "," <<  distance <<"\n";
      } 
      counter_record++;
      std::cout<< "Is path found" << status << std::endl;
  }
  ros::spin();
  return 0;
}
