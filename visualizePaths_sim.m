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

% display EE trajectory path
for i=1:height(q_robot1)
   T1 = getTransform(robot, q_robot1(i,:)', robot.BodyNames{end});
   pose1 = [T1(1,4) T1(2,4) T1(3,4) tform2eul(T1, "XYZ")];
   scatter3(pose1(1), pose1(2), pose1(3), 15, 'k', 'filled');
   plotGridCoordinates(pose1, 0.025);
end
for i=1:height(q_robot2)
   T2 = getTransform(obst_robot, q_robot2(i,:)', obst_robot.BodyNames{end});
   pose2 = [T2(1,4) T2(2,4) T2(3,4) tform2eul(T2, "XYZ")];
   scatter3(pose2(1), pose2(2), pose2(3), 15, 'k', 'filled');
   plotGridCoordinates(pose2, 0.025);
end
