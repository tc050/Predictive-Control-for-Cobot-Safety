function joint_waypoints = taskToJointWaypoints(task_waypoints, robot)
    % interpolate task-space points to obtain smoother trajectory
    [task_waypoints, ~, ~] = trapveltraj(task_waypoints', size(task_waypoints,1));

    % solve inverse kinematics to obtain joints over path
    joint_waypoints = zeros(height(task_waypoints'), 7);
    
    robot_ik = inverseKinematics("RigidBodyTree", robot);
    robot_ik.SolverParameters.AllowRandomRestart = false;
    weights = [1 1 1 1 1 1];
    initialConfig = homeConfiguration(robot);

    [q, ~] = robot_ik(robot.BodyNames{end}, ...
        trvec2tform(task_waypoints(1:3, 1)')*eul2tform(task_waypoints(4:end, 1)', "XYZ"), ...
        weights, initialConfig);

    joint_waypoints(1,:) = q;
    for i=2:height(task_waypoints')
        [q, ~] = robot_ik(robot.BodyNames{end}, ...
            trvec2tform(task_waypoints(1:3, i)')*eul2tform(task_waypoints(4:end, i)', "XYZ"), ...
            weights, joint_waypoints(i-1,:)');

        joint_waypoints(i,:) = q;
    end
end