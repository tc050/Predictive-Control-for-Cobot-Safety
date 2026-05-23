function trajectory = generateTraj(joint_waypoints)
    % generate trajectory on joints implementing velocity constaints
    scale = 0.5; % limit to 50% of maximum speed limit
    joint_velocity_limits = [1.39*scale; 1.39*scale; 1.39*scale; 1.39*scale; 1.22*scale; 1.22*scale; 1.22*scale]; % KinovaGen3 speed general limits

    % generate trajectory using trapezoidal velocity profiles
    sample_resolution = 10;
    [q_traj, qd_traj, qdd_traj, T, ~] = trapveltraj(joint_waypoints', size(joint_waypoints,1)*sample_resolution, PeakVelocity=joint_velocity_limits);

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