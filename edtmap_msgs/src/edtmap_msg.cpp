#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ros/ros.h>
#include <ros/console.h>                                                                                                                                                                           

// messages
#include "edtmap_msg/EDTMap.h"

void echo_messages(const edtmap_msg::EDTMap::ConstPtr& msg)
{
    for (std::vector<double>::const_iterator dp = msg->data.begin(); dp != msg->data.end(); ++dp)
        std::cout << *dp << ' ';
    std::cout << '\n';
}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "edtmap_msg");
    ros::NodeHandle n;
    ros::Subscriber edtmap_msg_sub = n.subscribe ("echo_edtmap_msg", 8, echo_messages);
    ros::spin();
  return 0;
}