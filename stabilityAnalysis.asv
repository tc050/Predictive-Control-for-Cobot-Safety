clear; clc;

function stateDot = timeBasedInputs_Task(MotionModel, timeInterval, eeInit, eeRef, T, state)
    [refPose, refVel] = transformtraj(eeInit, eeRef, timeInterval, T);
    stateDot = derivative(MotionModel, state, refPose, refVel);
end

function stateDot = timeBasedInputs_Joint(MotionModel, timeInterval, configWaypoints, t, state)
    % Use a B-spline curve to ensure the trajectory is smooth and moves
    % through the waypoints with non-zero velocity
    [qd, qdDot] = bsplinepolytraj(configWaypoints,  timeInterval , t);
    
    % Compute state derivative
    stateDot = derivative(MotionModel, state, [qd; qdDot]);
end

%% robot model setup
% load the KINOVA Gen 3 robot manipulator multibody model and rigid body tree
[robotRBT, robotInformation] = loadrobot("kinovaGen3", "DataFormat", "column", "Gravity", [0 0 -9.81]);
joint_qty = numel(homeConfiguration(robotRBT));

% establish end effector
endEff = 'EndEffector_Link';

% initial joint positions
qInit = [0 0 0 0 0 0 0]'*pi/180; % converted from deg to rad
eeInit = getTransform(robotRBT, qInit, endEff); % set initial configuration

% reference/goal joint positions (step input)
qRef = [90 90 90 90 90 90 90]'*pi/180;
eeRef = getTransform(robotRBT, qRef, endEff);

% prepare inverse kinematic equations for the rigid body tree
robot_inverseKinematics = inverseKinematics("RigidBodyTree", robotRBT);
robot_inverseKinematics.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];s

%% generate task-space trajectory
% end-effector travel distance
dist = norm(tform2trvec(eeInit)- tform2trvec(eeRef));

% define discrete time
st = 0.2;
toolSpeed = 0.2; % m/s
t = (0:st:(dist/toolSpeed))';
timeInterval = [t(1); t(end)];

% trajectory
[waypoints, velocities] = transformtraj(eeInit, eeRef, timeInterval, t);

%% control task-space motion
% motion model of rigid body tree
tsMotionModel = taskSpaceMotionModel('RigidBodyTree', robotRBT, 'EndEffectorName', endEff);

% set gains orientation of PID controller
tsMotionModel.Kp(1:3, 1:3) = 0;
tsMotionModel.Kd(1:3, 1:3) = 0;

% define initial states (joint positions & velocities)
q0 = homeConfiguration(robotRBT);
qd0 = zeros(size(q0));

[task_t, task_state] = ode15s(@(T, state) timeBasedInputs_Task(tsMotionModel, timeInterval, eeInit, eeRef, T, state), timeInterval, [q0; qd0]);

%% generate joint-space trajectory
% desired joint configurations by inverse kinematics
joints_Final = robot_inverseKinematics(endEff, eeRef, weights, qInit);
wrappedJointFinal = wrapTo2Pi(joints_Final); % ensured final trajectory covers minimal distance

ctrlpoints = [qInit, wrappedJointFinal];
jointConfigArray = cubicpolytraj(ctrlpoints, timeInterval, t);

%% control joint-space trajectory
jsMotionModel = jointSpaceMotionModel('RigidBodyTree',robotRBT,'MotionType','PDControl');
q0 = homeConfiguration(robotRBT);
qd0 = zeros(size(q0));

[tJoint, stateJoint] = ode15s(@(t, state) timeBasedInputs_Joint(jsMotionModel, timeInterval, jointConfigArray, t, state), timeInterval, [q0; qd0]);

%% simulation
% vizualize initial configuration
show(robotRBT, qInit, 'PreservePlot', false, 'Frames', 'off');
axis auto;
view([60, 10]);
hold on

for i = 1:length(t)
    % current time
    tCurr = t(i);

    % interpolate simulated joint positions to get current configuration
    configCurr = interp1(task_t, task_state(:, 1:joint_qty), tCurr)';
    poseCurr = getTransform(robotRBT, configCurr, endEff);
    show(robotRBT, configCurr, 'PreservePlot', false, 'Frames', 'off');
    taskSpaceMarker = plot3(poseCurr(1,4), poseCurr(2,4), poseCurr(3,4), 'b.', 'MarkerSize', 20);
    drawnow;
end

% Return to initial configuration
show(robotRBT, configCurr, 'PreservePlot', false, 'Frames', 'off');

% simulate joint-space
for i = 1:length(t)
    % current time
    tCurr = t(i);

    % interpolate simulated joint positions to get configuration
    configCurr = interp1(tJoint, stateJoint(:, 1:joint_qty), tCurr)';
    poseCurr = getTransform(robotRBT, configCurr, endEff);
    show(robotRBT, configCurr, 'PreservePlot', false, 'Frames', 'off');
    jointSpaceMarker = plot3(poseCurr(1,4), poseCurr(2,4), poseCurr(3,4), 'r.', 'MarkerSize', 20);
    drawnow; 
end

% legends and title
legend([taskSpaceMarker jointSpaceMarker], {'Defined in Task-Space', 'Defined in Joint-Space'});
title('Manipulator Trajectories')

%%
% % pre-allocated memory for joint waypoint joint configurations
% q_steps = zeros(size(waypoints, 1), numel(homeConfiguration(robotRBT)));
% 
% % initial transformation (end-effector position)
% eeCurr = eeInit;
% 
% % iteration loop for simulation
% for i = 0:size(waypoints, 1)
%     % 
%     eeCurr(1:3, 4) = waypoints(i, :)';
% end
