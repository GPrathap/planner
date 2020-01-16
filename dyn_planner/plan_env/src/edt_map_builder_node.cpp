#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_env/sdf_map.h>

using namespace dyn_planner;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "edt_map_builder_node");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");
  SDFMap sdf_map;
  sdf_map.init(nh);
  ros::spin();
  return 0;
}
