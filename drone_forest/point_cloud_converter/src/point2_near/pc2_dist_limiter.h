#ifndef __PC2_DIST_LIMITER_H__
#define __PC2_DIST_LIMITER_H__

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

namespace point_cloud_converter
{

typedef geometry_msgs::Point32 GeometryPoint;
typedef std::vector<geometry_msgs::Point32> GeometryPointsVec;

class PC2DistLimiter
{
public:
  PC2DistLimiter(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  ~PC2DistLimiter();

  double rate();
  double limitRadius();
  std::string farmeId();
  void process();

private:
  void robotPosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void rawMarkerArrCallback(const visualization_msgs::MarkerArray::ConstPtr &msg);

  void filterPointCloud();
  template <typename T>
  double getDistBetween(const T& point_1, const T& point_2);

  sensor_msgs::PointCloud2 createPointCloud2();
  template <typename T>
  void copyToByteVector(T* val, std::vector<uint8_t>& buffer);
  void copyPointVectorToByteVector(const GeometryPointsVec& p_vec, std::vector<uint8_t>& buffer);

  void pubFilteredPointCloud2(const sensor_msgs::PointCloud2 &msg);
  bool pc2MsgIsEmpty(const sensor_msgs::PointCloud2& msg);

private:
  ros::Subscriber robot_pos_sub_;
  ros::Subscriber raw_marker_arr_sub_;
  ros::Publisher limeted_pc2_pub_;

  std::string frame_id_;
  double limit_radius_;
  double rate_;
  double prev_time_;
  double delay_;
  GeometryPoint robot_point_;

  GeometryPointsVec raw_pc_points_;
  GeometryPointsVec filtered_pc_points_;
};

}

#endif // __PC2_DIST_LIMITER_H__
