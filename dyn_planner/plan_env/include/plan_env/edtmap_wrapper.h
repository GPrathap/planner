#ifndef _EDT_ENVIRONMENT_WRAPPER_H_
#define _EDT_ENVIRONMENT_WRAPPER_H_

#include <Eigen/Eigen>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/function_output_iterator.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/timer.hpp>
#include <boost/foreach.hpp>
#include <edtmap_msg/EDTMap.h>
#include <nav_msgs/Odometry.h>

using std::cout;
using std::endl;
using std::list;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;



namespace dyn_planner
{
  namespace bg = boost::geometry;
  namespace bgi = boost::geometry::index;
    class EDTMapWrapper
    {
        private:
            /* data */
            std::vector<double> edt_map_;
            Eigen::Vector3d min_range_, max_range_, map_size_, origin_;
            std::vector<Eigen::Vector3d> current_ranges;
            double resolution_sdf_, resolution_inv_;
            Eigen::Vector3i grid_size_; 
            ros::Subscriber edtmap_sub_;

            typedef bg::model::point<double, 3, bg::cs::cartesian> point_t;
            typedef bg::model::box<point_t> box_t;
            typedef std::pair<box_t, uint64_t> value_t;
            typedef boost::geometry::box_view<box_t> box_view;
            typedef bgi::rtree<value_t, bgi::quadratic<8, 4>> RTree; 
            RTree obs_tree; 

            nav_msgs::Odometry odom_;
            bool odom_valid_; 
            bool map_valid_;
            bool have_odom_;

        public:
            EDTMapWrapper(/* args */) {}
            ~EDTMapWrapper() {}

            void init(ros::NodeHandle& nh);
            void setMap(std::vector<double> map, bool map_valid, bool odom_valid);
            double evaluateCoarseEDT(const Eigen::Vector3d& pos, const double& time);
            void getMapRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
            double getDistance(Eigen::Vector3d pos);
            void posToIndex(Eigen::Vector3d pos, Eigen::Vector3i& id);
            bool isInMap(Eigen::Vector3d pos);
            bool odomValid();
            bool mapValid();
            nav_msgs::Odometry getOdom();
            void setOdom(const nav_msgs::OdometryConstPtr& msg);
            std::vector<Eigen::Vector3d> getMapCurrentRange();
            std::vector<Eigen::Vector3d> nearest_obstacles_to_current_pose(Eigen::Vector3d x
                            , int max_neighbours);
            // void edtmapCallback(const edtmap_msg::EDTMap::ConstPtr& msg);
            typedef shared_ptr<EDTMapWrapper> Ptr;
    };

}  // namespace dyn_planner

#endif