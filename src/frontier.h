#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include <move_base_msgs/MoveBaseActionResult.h>
#include <ros/console.h>
#include <vector>
#include <math.h>

using namespace std ;

typedef unsigned int UINT ;

class frontier
{
  public:
    frontier(ros::NodeHandle) ;
    ~frontier() {}
    
  private:
    ros::Subscriber subResult ;
    ros::Subscriber subMap ;
    ros::Subscriber subOdom ;
    ros::Subscriber subCostMap ;
    ros::Publisher pubFrontierMap ;
    ros::Publisher pubWaypoint ;
    bool fWaypoint ;
    bool fMap ;
    bool fOdom ;
    move_base_msgs::MoveBaseActionResult result ;
    nav_msgs::OccupancyGrid slamMap ;
    nav_msgs::OccupancyGrid frontierMap ;
    geometry_msgs::Pose pose ;
    geometry_msgs::Twist waypoint ;
    UINT width ;
    UINT height ;
    UINT frontierThreshold ;
    double cancelThreshold ;
    vector< vector<double> > cancelledWaypoints ;
    vector< vector<double> > sentWaypoints ;
    
    void waypointCallback(const move_base_msgs::MoveBaseActionResult&) ;
    void mapCallback(const nav_msgs::OccupancyGrid&) ;
    void odomCallback(const nav_msgs::Odometry&) ;
    void neighbours(UINT, vector<UINT> &, vector<UINT> &) ;
    vector< vector<UINT> > frontierCentroids() ;
    vector<double> centroidSelection(vector <vector<UINT> >) ;
};

frontier::frontier(ros::NodeHandle nh){
  subResult = nh.subscribe("move_base/result", 10, &frontier::waypointCallback, this) ;
  subMap = nh.subscribe("map", 10, &frontier::mapCallback, this) ;
  subOdom = nh.subscribe("odom", 10, &frontier::odomCallback, this) ;
  pubFrontierMap = nh.advertise<nav_msgs::OccupancyGrid>("frontier_cells", 50, true) ;
  pubWaypoint = nh.advertise<geometry_msgs::Twist>("map_goal", 10) ;
  fWaypoint = false ;
  fMap = false ;
  fOdom = false ;
  double temp = 0.0 ;
  ros::param::get("frontier_waypoint/frontier_threshold", temp) ;
  frontierThreshold = (UINT) temp ;
  ROS_INFO_STREAM("Frontier occupancy threshold: " << frontierThreshold) ;
  ros::param::get("frontier_waypoint/cancel_threshold", temp) ;
  cancelThreshold = temp ;
  ROS_INFO_STREAM("Waypoint cancel threshold: " << cancelThreshold) ;
}

void frontier::waypointCallback(const move_base_msgs::MoveBaseActionResult& msg){
  result = msg ;
  if (fMap == true && fOdom == true){
    if (msg.status.status == 6 || msg.status.status == 2 || msg.status.status == 4 || msg.status.status == 5){ // waypoint was preempted or aborted
      vector<double> wp_old ;
      wp_old.push_back(waypoint.linear.x)  ;
      wp_old.push_back(waypoint.linear.y)  ;
      cancelledWaypoints.push_back(wp_old) ;
      ROS_INFO_STREAM("Number of cancelled waypoints: " << cancelledWaypoints.size()) ;
    }
    // Compute the centroid of all connected frontiers
    vector< vector<UINT> > centroids = frontierCentroids() ;
    vector<double> wp = centroidSelection(centroids) ;
    if (wp.size() == 2){
      waypoint.linear.x = wp[0] ;
      waypoint.linear.y = wp[1] ;
      waypoint.linear.z = 0 ;
      waypoint.angular.x = 0 ;
      waypoint.angular.y = 0 ;
      waypoint.angular.z = 0 ;
      sentWaypoints.push_back(wp) ;
      ROS_INFO_STREAM("Sending new waypoint ("<< waypoint.linear.x << "," << waypoint.linear.y << ")") ;
      pubWaypoint.publish(waypoint) ;
      fWaypoint = true ;
    }
    else
      ROS_INFO_STREAM("No valid frontiers found.") ;
  }
  fOdom = false ;
}


void frontier::mapCallback(const nav_msgs::OccupancyGrid& msg){
  if (fMap == false){
    // Initialise frontierMap message
    frontierMap.header.seq = msg.header.seq ;
    frontierMap.header.stamp = msg.header.stamp ;
    frontierMap.header.frame_id = msg.header.frame_id ;
    
    frontierMap.info.map_load_time = msg.info.map_load_time ;
    frontierMap.info.resolution = msg.info.resolution ;
    frontierMap.info.width = msg.info.width ;
    frontierMap.info.height = msg.info.height ;
    
    frontierMap.info.origin.position.x = msg.info.origin.position.x ;
    frontierMap.info.origin.position.y = msg.info.origin.position.y ;
    frontierMap.info.origin.position.z = msg.info.origin.position.z ;
    frontierMap.info.origin.orientation.x = msg.info.origin.orientation.x ;
    frontierMap.info.origin.orientation.y = msg.info.origin.orientation.y ;
    frontierMap.info.origin.orientation.z = msg.info.origin.orientation.z ;
    frontierMap.info.origin.orientation.w = msg.info.origin.orientation.w ;
    
    frontierMap.data = msg.data ;
    
    slamMap.data = msg.data ;
    
    width = msg.info.width ;
    height = msg.info.height ;
    
    fMap = true ;
  }
  else { // update time stamps
    frontierMap.header.seq = msg.header.seq ;
    frontierMap.header.stamp = msg.header.stamp ;
    frontierMap.info.map_load_time = msg.info.map_load_time ;
    slamMap.data = msg.data ;
  }
  
  for (UINT i = 0; i < msg.data.size(); i++){
    if (msg.data[i] < 10 && msg.data[i] >= 0) { // free space threshold
      // collect row column indices of all valid neighbours
      vector<UINT> r ;
      vector<UINT> c ;
      neighbours(i,r,c) ;
            
      // determine if two or more neighbouring cells are unknown
      UINT k = 0 ;
      for (UINT ii = 0; ii < r.size(); ii++){
        for (UINT jj = 0; jj < c.size(); jj++) {
          if (msg.data[r[ii]*width+c[jj]] == -1)
            k++ ;
        }
      }
      if (k >= 2)
        frontierMap.data[i] = 1 ;
      else
        frontierMap.data[i] = 0 ;
    }
    else
      frontierMap.data[i] = 0 ;
  }
  pubFrontierMap.publish(frontierMap) ;
}

void frontier::odomCallback(const nav_msgs::Odometry& msg){
  fOdom = true ;
  pose.position.x = msg.pose.pose.position.x ;
  pose.position.y = msg.pose.pose.position.y ;
  pose.position.z = msg.pose.pose.position.z ;
  pose.orientation.x = msg.pose.pose.orientation.x ;
  pose.orientation.y = msg.pose.pose.orientation.y ;
  pose.orientation.z = msg.pose.pose.orientation.z ;
  pose.orientation.w = msg.pose.pose.orientation.w ;
}

void frontier::neighbours(UINT i, vector<UINT> &r, vector<UINT> &c){
  r.push_back(i/width) ;
  c.push_back(i%width) ;
  if (i/width > 0)
    r.push_back(i/width - 1) ;
  if (i/width < height-1)
    r.push_back(i/width + 1) ;
  if (i%width > 0)
    c.push_back(i%width - 1) ;
  if (i%width < width-1)
    c.push_back(i%width + 1) ;
}

vector< vector<UINT> > frontier::frontierCentroids(){
  vector< vector<UINT> > centroids ; // [no. members, x, y] rows in order of set number
  vector<UINT> temp (width,0) ;
  vector< vector<UINT> > membership (height,temp) ;
  UINT k = 0 ;
  
  // Assumes square map
  vector<UINT> border ;
  for (UINT i = 0; i < width; i++){
    if (frontierMap.data[i*width+i] == 1 && membership[i][i] == 0) // Border corner cell
      border.push_back(i*width+i) ;
    for (UINT j = 0; j < i; j++){
      if (frontierMap.data[j*width+i] == 1 && membership[j][i] == 0) // Border column cells
        border.push_back(j*width+i) ;
      if (frontierMap.data[i*width+j] == 1 && membership[i][j] == 0) // Border row cells
        border.push_back(i*width+j) ;
    }
    
    // Search all border cells marked as frontier cells
    for (UINT j = 0; j < border.size(); j++) {
      UINT x = border[j]/width ;
      UINT y = border[j]%width ;
      // Check neighbours for existing membership
      vector<UINT> r ;
      vector<UINT> c ;
      neighbours(border[j],r,c) ;
      bool assigned = false ;
      for (UINT ii = 0; ii < r.size(); ii++){
        for (UINT jj = 0; jj < c.size(); jj++) {
          if (membership[r[ii]][c[jj]] != 0){
            membership[x][y] = membership[r[ii]][c[jj]] ;
            assigned = true ;
            break ;
          }
        }
        if (assigned)
          break ;
      }
      
      // Create new frontier set
      if (!assigned){
        vector<UINT> c_temp (3,0) ;
        centroids.push_back(c_temp) ;
        membership[x][y] = ++k ;
      }
      
      // Update centroids
      centroids[membership[x][y]-1][0]++ ; // increment number of members
      centroids[membership[x][y]-1][1] += x ;
      centroids[membership[x][y]-1][2] += y ;
    }
    border.clear() ;
  }
  
  for (UINT i = 0; i < centroids.size(); i++){
    centroids[i][1] /= centroids[i][0] ;
    centroids[i][2] /= centroids[i][0] ;
  }
  return centroids ;
}

vector<double> frontier::centroidSelection(vector <vector<UINT> > centroids){
  vector<double> cmd_wp ;
  double minDistance = sqrt(pow((double)width,2)+pow((double)height,2))*frontierMap.info.resolution ;
  int minInd = -1 ;
  for (UINT i = 0; i < centroids.size(); i++){
    UINT ii = centroids[i][1]*width+centroids[i][2] ;
    // Check size of frontier and that centroid is known free space
    if (centroids[i][0] >= frontierThreshold && slamMap.data[ii] < 10){
      double x = (double)centroids[i][2]*frontierMap.info.resolution + frontierMap.info.origin.position.y ;
      double y = (double)centroids[i][1]*frontierMap.info.resolution + frontierMap.info.origin.position.x ;
      bool noGoList = false ;
      for (UINT j = 0; j < cancelledWaypoints.size(); j++){
        if (abs(cancelledWaypoints[j][0] - x) < cancelThreshold && abs(cancelledWaypoints[j][1] - y) < cancelThreshold){
          ROS_INFO_STREAM("Ignoring previous failed cases.") ;
          noGoList = true ;
          break ;
        }
      }
      for (UINT j = 0; j < sentWaypoints.size(); j++){
        if (abs(sentWaypoints[j][0] - x) < cancelThreshold && abs(sentWaypoints[j][1] - y) < cancelThreshold){
          ROS_INFO_STREAM("Ignoring previously sent waypoints.") ;
          noGoList = true ;
          break ;
        }
      }
      double distance = sqrt(pow((x-pose.position.x),2) + pow((y-pose.position.y),2)) ;
      if (distance < minDistance && !noGoList){
        minDistance = distance ;
        minInd = i ;
      }
    }
  }
  
  if (minInd >= 0){
    cmd_wp.push_back(centroids[minInd][2]*frontierMap.info.resolution + frontierMap.info.origin.position.y) ;
    cmd_wp.push_back(centroids[minInd][1]*frontierMap.info.resolution + frontierMap.info.origin.position.x) ;
  }
  else
    cmd_wp.push_back(0) ; // flag since no valid frontiers were found
  
  return cmd_wp ;
}
