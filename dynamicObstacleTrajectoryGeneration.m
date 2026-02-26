%% Generate trajectory for dynamic obstacles

% discrete time parameters
st = 0.2;
toolSpeed = 0.2; % m/s

% individual task waypoints
task_waypoints = [0.7 0.4 0.3; 
    0.4 0.0 0.3; 
    0.8 -0.2 0.4; 
    0.8 -0.2 0.7;
    0.5 -0.3 0.8;
    0.8 -0.4 0.7;
    0.8 -0.4 0.4;
    0.4 -0.5 0.3;
    0.4 0.1 0.3;
    0.7 0.4 0.3];

% matrix of distances between waypoints
waypoints = [];
velocities = [];
for i=2:height(task_waypoints)
    dist_i = norm(task_waypoints(i-1,:) - task_waypoints(i,:));
    
    % segment trajectory time
    t = (0:st:(dist_i/toolSpeed))';
    timeInterval = [t(1); t(end)];

    % trajectory
    [waypoints_i, velocities_i] = transformtraj(trvec2tform(task_waypoints(i-1,:)), trvec2tform(task_waypoints(i,:)), timeInterval, t);
    [~, ~, size_i] = size(waypoints_i);
    for j=1:size_i
        waypoints = [waypoints; {waypoints_i(:,:,j)}];
        velocities = [velocities; {velocities_i(:,j)}];
    end
end
