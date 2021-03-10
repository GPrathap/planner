#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <bspline_opt/non_uniform_bspline.h>
#include <boost/circular_buffer.hpp>

using std::vector;
namespace hagen_planner
{
class PlanningVisualization
{
private:
  enum DRAW_ID
  {
    GOAL = 1,
    PATH = 200,
    BSPLINE = 300,
    BSPLINE_CTRL_PT = 400,
    PREDICTION = 500
  };

  /* data */
  ros::NodeHandle node;
  ros::Publisher traj_pub, traj_whole_pub, state_pub;
  ros::Publisher search_space_publisher, pub_rviz_markers_;

  void displaySphereList(vector<Eigen::Vector3d> list, double resolution, Eigen::Vector4d color, int id);

public:
  PlanningVisualization(/* args */)
  {
  }
  ~PlanningVisualization()
  {
  }

  PlanningVisualization(ros::NodeHandle& nh);

  void drawPath(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id = 0);

  void drawBspline(NonUniformBspline bspline, double size, Eigen::Vector4d color, bool show_ctrl_pts = false,
                   double size2 = 0.1, Eigen::Vector4d color2 = Eigen::Vector4d(1, 1, 0, 1), int id1 = 0, int id2 = 0);

  void drawGoal(Eigen::Vector3d goal, double resolution, Eigen::Vector4d color, int id = 0);

  void publish_marker(visualization_msgs::Marker, int id_
        , std::string name_space);
  
  void drawObsMap(visualization_msgs::MarkerArray marker_array, double resolution, Eigen::Vector4d color, int id);
  void displayTrajWithColor(boost::circular_buffer<Eigen::Vector3d>& path, double resolution, Eigen::Vector4d color, int id);
  void drawState(Eigen::Vector3d pos, Eigen::Vector3d vec, int id, Eigen::Vector4d color);
  void drawConePoints(Eigen::MatrixXd poses, double resolution, Eigen::Vector4d color, int id);

  typedef std::shared_ptr<PlanningVisualization> Ptr;
};
}  // namespace hagen_planner
#endif