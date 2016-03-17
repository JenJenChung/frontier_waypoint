# frontier_waypoint
Basic frontier exploration package to be used in conjunction with move_base and simple_navigation_goals.

When used with move_base, it is recommended that you:
  - set the xy_goal_tolerance higher (~1.0)
  - set the yaw_goal_tolerance to a value > pi (final heading invariant)

These values can be found in the base_local_planner_params.yaml, which is typically in the folder with the move_base.launch file.

The frontier waypoint execution will begin after an initial waypoint is sent and achieved by the base. You can do this by commanding a 2D nav goal via the rviz interface, or by publishing a message to the /map_goal or /base_link_goal topics.

The node publishes three topics:
  - "fontier_cells": an OccupancyGrid message that identifies frontier cells
  - "map_goal": a Twist message that includes the (x,y) waypoint coordinates in the map frame (to be used in conjunction with the simple_navigation_goals and move_base package)
  - "move_base/cancel": a GoalID message to cancel a waypoint
