clear;
% generate dynamic obstacle robot
joint_qty = 7;
endEff = 'EndEffector_Link';
weights = [1 1 1 1 1 1];

% load another KinovaGen3 robot representing obstacle
[obst_robot, ~] = loadrobot("kinovaGen3", DataFormat="column", Gravity=[0 0 -9.81]);

% discrete time parameters
st = 0.2;
toolSpeed = 0.2; % m/s

%% task waypoints
task_waypoints = [0.7 0.4 0.3; 
    0.4 0.0 0.3];

% obstacle robot inverse kinematics
obst_robot_InverseKinematics = inverseKinematics("RigidBodyTree", obst_robot);
obst_robot_InverseKinematics.SolverParameters.AllowRandomRestart = false;

% obstacle robot end-effector task-space coordinates transformation matrix
obst_robot_coord = trvec2tform(task_waypoints(1,:))*axang2tform([0 1 0 pi]);

% obstacle robot initial configuration
obst_robot_config = obst_robot_InverseKinematics(endEff, obst_robot_coord, weights, homeConfiguration(obst_robot));
obst_robot_config = wrapToPi(obst_robot_config);
obst_q = [obst_robot_config;zeros(7,1)];
disp(obst_q)

%% generate motion model of dynamic obstacle element
obst_robot_motionModel = taskSpaceMotionModel(RigidBodyTree=obst_robot, EndEffectorName=endEff);

% % establish PD controller gains within the motion model
% obst_robot_motionModel.Kp(1:3, 1:3) = 0;
% obst_robot_motionModel.Kd(1:3, 1:3) = 0;

%% Generate trajectory for dynamic obstacles

% matrix of distances between waypoints
waypoints = [];
obst_task_t = zeros(1);
obst_task_state = [];

% compute discrete robot waypoints between primary task waypoints
tic
for i=2:height(task_waypoints)
    disp(toc)
    % define next reference
    eeInit = se3(task_waypoints(i-1,:),"trvec");
    eeRef = se3(task_waypoints(i,:), "trvec");
    refVel = zeros(6,1);

    %% discrete time of waypoint segment
    % distance between waypoint to next
    dist_i = norm(task_waypoints(i-1,:) - task_waypoints(i,:));

    % segment time
    t = (0:st:(dist_i/toolSpeed))';
    timeInterval = [t(1); t(end)];
    % 
    % % waypoints
    % [waypoints_i, ~] = transformtraj(trvec2tform(task_waypoints(i-1,:)), trvec2tform(task_waypoints(i,:)), timeInterval, t);
    % 
    % % log all waypoints in corresponding matrices
    % [~, ~, size_i] = size(waypoints_i);
    % parfor j=1:size_i
    %     waypoints = [waypoints; {waypoints_i(:,:,j)}];
    % end
    
    % trajectory robot states and velocities
    [obst_task_t_i, obst_task_state_i] = ode15s(@(T, state) ...
        timeBasedInput_Task(obst_robot_motionModel, timeInterval, trvec2tform(task_waypoints(i-1,:)), ...
        trvec2tform(task_waypoints(i,:)), T, state), timeInterval, obst_q);

    % % keep time steps consistent across trajectory splits
    % obst_task_t_i = obst_task_t_i(2:end) + obst_task_t(end);

    % log all trajectories and velocities in corresponding matrices
    parfor j=1:numel(obst_task_t_i)
        obst_task_state = [obst_task_state; obst_task_state_i(j,:)];
        obst_task_t = [obst_task_t; obst_task_t_i(j)];
    end

    % update the states of the point positions and velocities
    obst_q = obst_task_state(end,:);
end
disp(toc)

% % remove added time stamp at end
% obst_task_t = obst_task_t(1:end-1);

% overall discrete time
t = (0:st:numel(waypoints)/st);
timeInterval = [t(1); t(end)];

%% Simulation
figure('Name', 'Task Simulation');

% visualize obstacle robot configuration
show(obst_robot, obst_robot_config, "PreservePlot", false, "Frames", "off");
axis([-0.5 1.5 -1.0 1.0 -0.04 1.3])
view([-90 10 5]);
hold on

% display obstacle waypoints
for i=1:height(task_waypoints)
    plot3(task_waypoints(1), task_waypoints(2), task_waypoints(3), 'y.', 'MarkerSize', 10); 
end

% track dynamic obstacle poses
obst_poses = zeros(4, 4, length(t));

r = rateControl(2);
title("Robot Reaching Target Pose")
for i = 1:size(obst_task_state,1)
    show(obst_robot,obst_task_state(i,1:7)',PreservePlot=false);
    waitfor(r);
end
hold off

% % iteratively simulate robot state changes
% for i = 1:length(t)
%     disp(i)
%     % interpolate simulated dynamic obstacle joint positions for current configuration
%     obst_configCurr = interp1(obst_task_t, obst_task_state(:, 1:joint_qty), t(i))';
%     obst_poseCurr = getTransform(obst_robot, obst_configCurr, endEff);
%     obst_poses(:,:,i) = obst_poseCurr;
% 
%     % display changes within the plot
%     show(obst_robot, obst_task_state(i, 1:joint_qty)', "PreservePlot", false, "Frames", "off");
% 
%     % % plot path taken
%     % obst_path = plot3(squeeze(obst_poses(1,4,1:i)), squeeze(obst_poses(2,4,1:i)), squeeze(obst_poses(3,4,1:i)), 'b');
%     drawnow;
% end
