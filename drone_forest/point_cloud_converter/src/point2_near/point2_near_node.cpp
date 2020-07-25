#include "pc2_dist_limiter.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point2_near");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  point_cloud_converter::PC2DistLimiter g_node(nh, nh_private);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Rate r(g_node.rate());
  while (ros::ok())
  {
      g_node.process();
      r.sleep();
  }
  ros::waitForShutdown();

  return 0;
}

