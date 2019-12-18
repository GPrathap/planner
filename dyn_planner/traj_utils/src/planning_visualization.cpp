#include <traj_utils/planning_visualization.h>

using std::cout;
using std::endl;
namespace dyn_planner
{
PlanningVisualization::PlanningVisualization(ros::NodeHandle& nh)
{
  node = nh;
  traj_pub = node.advertise<visualization_msgs::Marker>("/planning_vis/trajectory", 10);
  search_space_publisher = node.advertise<visualization_msgs::Marker>("/planning_vis/search_space", 10);
}

void PlanningVisualization::displaySphereList(vector<Eigen::Vector3d> list, double resolution, Eigen::Vector4d color,
                                              int id)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0, mk.pose.orientation.y = 0.0, mk.pose.orientation.z = 0.0, mk.pose.orientation.w = 1.0;
  mk.color.r = color(0), mk.color.g = color(1), mk.color.b = color(2), mk.color.a = color(3);
  mk.scale.x = resolution, mk.scale.y = resolution, mk.scale.z = resolution;
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++)
  {
    pt.x = list[i](0), pt.y = list[i](1), pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
}

void PlanningVisualization::drawBspline(NonUniformBspline bspline, double size, Eigen::Vector4d color,
                                        bool show_ctrl_pts, double size2, Eigen::Vector4d color2, int id1, int id2)
{
  vector<Eigen::Vector3d> traj_pts;
  double tm, tmp;
  bspline.getTimeSpan(tm, tmp);
  for (double t = tm; t <= tmp; t += 0.01)
  {
    Eigen::Vector3d pt = bspline.evaluateDeBoor(t);
    traj_pts.push_back(pt);
  }
  displaySphereList(traj_pts, size, color, BSPLINE + id1 % 100);

  // draw the control point
  if (!show_ctrl_pts)
    return;


  Eigen::MatrixXd ctrl_pts = bspline.getControlPoint();

  vector<Eigen::Vector3d> ctp;
  for (int i = 0; i < int(ctrl_pts.rows()); ++i)
  {
    Eigen::Vector3d pt = ctrl_pts.row(i).transpose();
    ctp.push_back(pt);
  }
  displaySphereList(ctp, size2, color2, BSPLINE_CTRL_PT + id2 % 100);
}

void PlanningVisualization::create_marker_point(Eigen::Vector3d _point_on_path,
        Eigen::MatrixXd covmat, , Eigen::Quaternion<double> q, int id_, std::string name_space){ 
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = name_space;
        marker.id = id_;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = _point_on_path[0];
        marker.pose.position.y = _point_on_path[1];
        marker.pose.position.z = _point_on_path[2];
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        marker.scale.x = covmat(0,0);
        marker.scale.y = covmat(1,1);
        marker.scale.z = covmat(2,2);
        marker.color.a = 0.2;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.8;
        marker.lifetime = ros::Duration(); 
        search_space_publisher.publish(marker);
        return;
}

void PlanningVisualization::drawGoal(Eigen::Vector3d goal, double resolution, Eigen::Vector4d color, int id)
{
  vector<Eigen::Vector3d> goal_vec = { goal };

  displaySphereList(goal_vec, resolution, color, GOAL + id % 100);
}

void PlanningVisualization::drawPath(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id)
{
  displaySphereList(path, resolution, color, PATH + id % 100);
}

// PlanningVisualization::
}  // namespace dyn_planner