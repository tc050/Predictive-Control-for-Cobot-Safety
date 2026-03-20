%% --- Generate trajectory for dynamic obstacles ---

% check for computational redundancy on main file executions
if exist("robotStateAll", "var") == 0
    % create motion model for the task space of the robot
    motionModel = taskSpaceMotionModel(RigidBodyTree=obst_robot,EndEffectorName=endEff);
    motionModel.Kp(1:3, 1:3) = eye(3)*200;
    motionModel.Kd(1:3, 1:3) = eye(3)*200;
    
    % initial state
    initialState = [homeConfiguration(obst_robot);zeros(7,1)];
    
    % define discrete time
    st = 0.2;
    toolSpeed = 0.2; % m/s
    tspan = 0:0.1:st;
    
    %% calculate intermediate waypoint instances for trajectory smoothing
    waypoints = [];
    waypoint_velocities = [];
    
    tic
    for i=2:height(task_waypoints)
        % distance between waypoint to next
        dist_i = norm(task_waypoints(i-1,1:3) - task_waypoints(i,1:3));
        
        % segment trajectory time
        t = (0:st:(dist_i/toolSpeed))';
        timeInterval = [t(1); t(end)];
    
        % waypoints
        [waypoints_i, waypoint_vel_i] = transformtraj(trvec2tform(task_waypoints(i-1,1:3)) ...
            *axang2tform(task_waypoints(i-1,4:end)), trvec2tform(task_waypoints(i,1:3))* ...
            axang2tform(task_waypoints(i-1,4:end)), timeInterval, t);
    
        % log all waypoints in corresponding matrices
        [~, ~, size_i] = size(waypoints_i);
        parfor j=1:size_i
            waypoints = [waypoints; {waypoints_i(:,:,j)}];
            waypoint_velocities = [waypoint_velocities; {waypoint_vel_i(:,j)}];
        end
        disp("Waypoints >> Loop Num: " + string(i) + " | Compute Time: " + string(toc) + " s")
    end
    disp(newline + "Required State Calculations: " + string(height(waypoints)))
    
    %% compute states in pursuit of waypoints
    robotStateAll = [];
    
    for i = 2:height(waypoints)
        % set the final reference position for the robot
        refPose = waypoints{i};
        refVel = waypoint_velocities{i};
        
        % find joint rotation and velocity state solutions over discrete steps
        [t,robotState] = ode15s(@(t,state) ...
            timeBasedInput_Task(motionModel, tspan, getTransform(obst_robot,initialState(1:7),"EndEffector_Link"), ...
                refPose, t, state), tspan,initialState);
    
        % save states
        parfor j=1:height(robotState)
            robotStateAll = [robotStateAll; robotState(j,:)];
        end
    
        % update initial state
        initialState = robotState(end,:)';
    
        disp("States >> Loop Num: " + string(i) + " | Compute Time: " + string(toc) + " s")
    end
    
    % clear unnecessary variables in workspace
    clearvars dist_i i motionModel refPose refVel robotState size_i st t timeInterval toolSpeed tspan waypoint_vel_i waypoint_velocities waypoints_i
end
