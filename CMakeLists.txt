cmake_minimum_required(VERSION 2.8.3)
project(frontier_waypoint)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gmapping
  move_base
  roscpp
  rospy
  tf
  costmap_2d
)


catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(frontier_waypoint src/frontier_waypoint.cpp)
target_link_libraries(frontier_waypoint ${catkin_LIBRARIES})
add_dependencies(frontier_waypoint ${catkin_EXPORTED_TARGETS})

add_executable(frontier_map src/frontier_map.cpp)
target_link_libraries(frontier_map ${catkin_LIBRARIES})
add_dependencies(frontier_map ${catkin_EXPORTED_TARGETS})
