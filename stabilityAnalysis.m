%% robot model setup
% load the KINOVA Gen 3 robot manipulator multibody model and rigid body tree
[robotRBT, robotInformation] = loadrobot("kinovaGen3", "DataFormat", "column", "Gravity", [0 0 -9.81]);

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
weights = [1 1 1 1 1 1];

%% generate task-space trajectory
% end-effector travel distance
dist = norm(tform2trvec(eeInit)- tform2trvec(eeRef));

% define discrete time
st = 0.2;
toolSpeed = 0.2; % m/s
t = (0:st:(distance/toolSpeed))';
timeInterval = [t(1); t(end)];

% trajectory
[waypoints, velocities] = transformtraj(eeInit, eeRef, timeInterval, t);

%% simulation
% vizualize initial configuration
show(robotRBT, qInit);
axis auto;
view([60, 10]);

% pre-allocated memory for joint waypoint joint configurations
q_steps = zeros(size(waypoints, 1), numel(homeConfiguration(robotRBT)));

% initial transformation (end-effector position)
eeCurr = eeInit;

% iteration loop for simulation
for i = 0:size(waypoints, 1)
    % 
    eeCurr(1:3, 4) = waypoints(i, :)';
end
