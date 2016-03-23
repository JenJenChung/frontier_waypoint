#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include <ros/console.h>
#include <actionlib_msgs/GoalID.h>

typedef unsigned int UINT ;

class frontier_costmap
{
  public:
    frontier_costmap(ros::NodeHandle, costmap_2d::Costmap2DROS *) ;
    ~frontier_costmap() {}
    
  private:
    ros::Subscriber subWaypoint ;
    ros::Subscriber subCostmapUpdate ;
    ros::Publisher pubCancelGoal ;
    
    costmap_2d::Costmap2DROS * costmap_ros_ ;
    geometry_msgs::PoseStamped currentWaypoint ;
    bool fWaypoint ;
    unsigned char cellCostThreshold ;
    
    void waypointCallback(const geometry_msgs::PoseStamped&) ;
    void mapUpdateCallback(const map_msgs::OccupancyGridUpdate&) ;
    void cancelNavigation() ;
} ;

frontier_costmap::frontier_costmap(ros::NodeHandle nh, costmap_2d::Costmap2DROS *cm_ros_): costmap_ros_(cm_ros_), fWaypoint(false), cellCostThreshold(50) {
  subWaypoint = nh.subscribe("move_base/current_goal", 10, &frontier_costmap::waypointCallback, this) ;
  subCostmapUpdate = nh.subscribe("frontier_map/frontier_map/costmap_updates", 10, &frontier_costmap::mapUpdateCallback, this) ;
  pubCancelGoal = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 10) ;
}

void frontier_costmap::waypointCallback(const geometry_msgs::PoseStamped& msg){
  currentWaypoint = msg ;
  fWaypoint = true ;
}

void frontier_costmap::mapUpdateCallback(const map_msgs::OccupancyGridUpdate&){
  if (fWaypoint){
    costmap_2d::Costmap2D *costmap_ = costmap_ros_->getCostmap() ;
    double wx = currentWaypoint.pose.position.x ;
    double wy = currentWaypoint.pose.position.y ;
    UINT mx ;
    UINT my ;
    bool convSuccess = costmap_->worldToMap(wx,wy,mx,my) ;
    bool belowThresh = false ;
    if (!convSuccess)
      ROS_INFO_STREAM("Waypoint out of costmap bounds; resetting waypoint.") ;
    else {
      unsigned char wpCellCost = costmap_->getCost(mx,my) ;
      if (wpCellCost < cellCostThreshold)
        belowThresh = true ;
      else
        ROS_INFO_STREAM("Waypoint cell cost greater than allowable threshold; resetting waypoint.") ;
    }
    if (!convSuccess || !belowThresh){
      // Cancel previous
      cancelNavigation() ;
    }
  }
}

void frontier_costmap::cancelNavigation()
{
  actionlib_msgs::GoalID cancelGoal ;
  pubCancelGoal.publish(cancelGoal) ;
}
