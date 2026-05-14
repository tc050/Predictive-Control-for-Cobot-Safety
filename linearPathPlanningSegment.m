function [planned_path_robot, len_path] = linearPathPlanningSegment(planned_path_robot, pose_robot, waypoints_robot)
    traj_x = linspace(pose_robot(1), waypoints_robot(1), 5); % x-Axis linear vector for linear motion
    traj_y = linspace(pose_robot(2), waypoints_robot(2), 5); % y-Axis linear vector for linear motion
    traj_z = linspace(pose_robot(3), waypoints_robot(3), 5); % z-Axis linear vector for linear motion
    
    planned_path_robot = [planned_path_robot; 
        traj_x', traj_y', traj_z', repmat(pose_robot(4), 1, height(traj_x'))', repmat(pose_robot(5), 1, height(traj_x'))', repmat(pose_robot(6), 1, height(traj_x'))'];

    % save count of how long path is
    len_path = height(traj_x');
end

