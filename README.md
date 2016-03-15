# frontier_waypoint
Basic frontier exploration package

When used with move_base, it is recommended that you:
  - set the xy_goal_tolerance higher (~1.0)
  - set the yaw_goal_tolerance to a value > pi (final heading invariant)

These values can be found in the base_local_planner_params.yaml, which is typically in the folder with the move_base.launch file.
