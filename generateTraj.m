function trajectory = generateTraj(waypoints, robot)
    if size(waypoints, 2) == 6
        joint_waypoints = taskToJointWaypoints(waypoints, robot); % waypoints input in task-space
    else
        joint_waypoints = waypoints; % waypoints input in joint-space
    end

    % generate trajectory on joints implementing velocity constaints
    %joint_velocity_limits = [1.39; 1.39; 1.39; 1.39; 1.22; 1.22; 1.22]; % KinovaGen3 speed general limits
    joint_velocity_limits = [0.75; 0.75; 0.75; 0.75; 0.5; 0.5; 0.5];

    % generate trajectory using trapezoidal velocity profiles
    [q_traj, qd_traj, qdd_traj, T, ~] = trapveltraj(joint_waypoints', size(joint_waypoints,1), PeakVelocity=joint_velocity_limits);

    % extablish discrete time
    dt = 0.05; % time step
    t = 0:dt:T(end);
    
    % 1D-interpolation on non-uniform trajectory into the uniform discrete time
    q = interp1(T, q_traj', t);
    qd = interp1(T, qd_traj', t);
    qdd = interp1(T, qdd_traj', t);
    
    % compile into trajectory object
    trajectory = [q, qd, qdd];
end