#include <ros/ros.h>
#include "frontier.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "frontier_waypoint") ;

  ros::NodeHandle nHandle ;
  
  frontier frontierMap(nHandle) ;
  
  ros::spin();
  return 0;
}
