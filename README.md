# frontier_waypoint
Basic frontier exploration package to be used in conjunction with move_base and simple_navigation_goals.

When used with move_base, it is recommended that you:
  - set the yaw_goal_tolerance to a value > pi (final heading invariant)

These values can be found in the base_local_planner_params.yaml, which is typically in the folder with the move_base.launch file.

The frontier waypoint execution will begin after an initial waypoint is sent and achieved by the base.

The frontier_waypoint node handles the calculation of frontier cells, frontier centroids and manages waypoint allocation. The node keeps track of previously unachieved waypoints and does not attempt to submit them again. It publishes two topics:
  - "fontier_cells": an OccupancyGrid message that identifies frontier cells
  - "map_goal": a Twist message that includes the (x,y) waypoint coordinates in the map frame (to be used in conjunction with the simple_navigation_goals and move_base package)

The frontier_map node maintains its own copy of the move_base global costmap to internally assess the feasibility of assigned waypoints as the costmap is updated. This node will pre-emptively cancel waypoints that are deemed too costly. It publishes one topic:
  - "move_base/cancel": a GoalID message to cancel a waypoint
