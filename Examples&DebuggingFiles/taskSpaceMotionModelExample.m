%% task waypoints
task_waypoints = [0.6 0.3 0.3; 
    0.4 -0.1 0.3;
    0.8 -0.2 0.4; 
    0.8 -0.2 0.7];

% generate robot rigid body tree
robot = loadrobot("kinovaGen3",DataFormat="column",Gravity=[0 0 -9.81]);

% create motion model for the task space of the robot
motionModel = taskSpaceMotionModel(RigidBodyTree=robot,EndEffectorName="EndEffector_Link");
motionModel.Kp(1:3, 1:3) = eye(3)*200;
motionModel.Kd(1:3, 1:3) = eye(3)*200;

% initial state
initialState = [homeConfiguration(robot);zeros(7,1)];

% define discrete time
st = 0.2;
toolSpeed = 0.2; % m/s
tspan = 0:0.1:st;

% calculate intermediate waypoint instances for trajectory smoothing
waypoints = [];
waypoint_velocities = [];
tic
for i=2:height(task_waypoints)
    % distance between waypoint to next
    dist_i = norm(task_waypoints(i-1,:) - task_waypoints(i,:));
    
    % segment trajectory time
    t = (0:st:(dist_i/toolSpeed))';
    timeInterval = [t(1); t(end)];

    % waypoints
    [waypoints_i, waypoint_vel_i] = transformtraj(trvec2tform(task_waypoints(i-1,:))*axang2tform([0 1 0 pi]), trvec2tform(task_waypoints(i,:))*axang2tform([0 1 0 pi]), timeInterval, t);

    % log all waypoints in corresponding matrices
    [~, ~, size_i] = size(waypoints_i);
    parfor j=1:size_i
        waypoints = [waypoints; {waypoints_i(:,:,j)}];
        waypoint_velocities = [waypoint_velocities; {waypoint_vel_i(:,j)}];
    end
    disp("Waypoints >> Loop Num: " + string(i) + " | Compute Time: " + string(toc))
end

disp(newline + "Required State Calculations: " + string(height(waypoints)))
% compute states in pursuit of waypoints
robotStateAll = [];
for i = 2:height(waypoints)
    % set the final reference position for the robot
    refPose = waypoints{i};
    refVel = waypoint_velocities{i};
    
    % find joint rotation and velocity state solutions over discrete steps
    [t,robotState] = ode15s(@(t,state) ...
        timeBasedInput_Task(motionModel, tspan, getTransform(robot,initialState(1:7),"EndEffector_Link"), ...
            refPose, t, state), ...
            tspan,initialState);

    % save states
    parfor j=1:height(robotState)
        robotStateAll = [robotStateAll; robotState(j,:)];
    end

    % update initial state
    initialState = robotState(end,:)';

    disp("States >> Loop Num: " + string(i) + " | Compute Time: " + string(toc))
end

% show initial robot configuration
figure
show(robot,initialState(1:7), "PreservePlot", false, "Frames", "off");
axis([-1 1 -1 1 0 1.25])
hold on
title("Robot and Target End-Effector Pose")

% simulate motion of the robot across state solutions
r = rateControl(5);
title("Robot Reaching Target Pose")
for i = 1:size(robotStateAll,1)
    show(robot,robotStateAll(i,1:7)',PreservePlot=false);
    waitfor(r);
end
hold off