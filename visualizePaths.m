%% --- Visualize Paths for Robots ---

% display waypoint trajectories
for i=1:height(waypoints_robot1)
   task_waypoint_plot_robot1 = scatter3(waypoints_robot1(i, 1), waypoints_robot1(i, 2), ...
       waypoints_robot1(i, 3), 50, 'magenta', 'filled');
   plotGridCoordinates(waypoints_robot1(i, :), 0.1);
end
for i=1:height(waypoints_robot2)
   task_waypoint_plot_robot2 = scatter3(waypoints_robot2(i, 1), waypoints_robot2(i, 2), ...
       waypoints_robot2(i, 3), 50, 'cyan', 'filled');
   plotGridCoordinates(waypoints_robot2(i, :), 0.1);
end

% display paths
for i=1:height(planned_path_robot1)
   scatter3(planned_path_robot1(i, 1), planned_path_robot1(i, 2), ...
       planned_path_robot1(i, 3), 15, 'k', 'filled');
   plotGridCoordinates(planned_path_robot1(i, :), 0.025);
end
for i=1:height(planned_path_robot2)
   scatter3(planned_path_robot2(i, 1), planned_path_robot2(i, 2), ...
       planned_path_robot2(i, 3), 15, 'k', 'filled');
   plotGridCoordinates(planned_path_robot2(i, :), 0.025);
end