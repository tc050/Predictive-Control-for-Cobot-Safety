% load the KINOVA Gen 3 robot manipulator multibody model and rigid body tree
[robotRBT, robotInformation] = loadrobot("kinovaGen3", "DataFormat", "column");
showdetails(robotRBT) % display the joint layout of the robot

joint_qty = numel(homeConfiguration(robotRBT));

% establish end effector
endEff = 'EndEffector_Link';

% initial joint positions
qInit = [0 15 180 -130 0 55 90]'*pi/180; % converted from deg to rad
tInit = getTransform(robotRBT, qInit, endEff); % set initial configuration

% vizualize initial configuration
show(robotRBT, qInit);
axis auto;
view([60, 10]);

% compute initial current robot joint configuration using inverse
% kinematics
robot_inverseKinematics = inverseKinematics("RigidBodyTree", robotRBT);
robot_inverseKinematics.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];

%% Example Path
% define a circular path for the robot to follow
centre = [0.5 0 0.4]; % x y z
radius = 0.1;

% time steps
st = 0.1;
t = (0:st:10)';

% define waypoints by time steps
theta = t * (2 * pi/t(end)) - pi/2;
points = centre + radius * [0*ones(size(theta)) cos(theta) sin(theta)];
num_waypoints = size(points, 1);

hold on;
plot3(points(:, 1), points(:, 2), points(:, 3), '-*g', 'LineWidth', 0.5);
xlabel('X');
ylabel('Y');
zlabel('Z');
axis auto;
view([60, 10]);
grid("minor");

% pre-defined matrix of joint configurations at each waypoint
qs = zeros(num_waypoints, joint_qty);

for i = 1:num_waypoints
    tDes = tInit;
    tDes(1:3,4) = points(i, :)';
    [q_sol, q_info] = robot_inverseKinematics(endEff, tDes, weights, qInit);

    % Display status of inverse kinematics results
    disp(q_info.Status);

    % store configuration
    qs(i, :) = q_sol(1:joint_qty);

    % prepare baseline configuration for next iteration
    qInit = q_sol;
end

% visualize animation
figure; set(gcf, "Visible", 'on');
ax = show(robotRBT, qs(1,:)');
ax.CameraPositionMode = "auto";
hold on;

% plot waypoints
plot3(points(:,1), points(:,2), points(:,3), '-g', 'LineWidth', 1.5);
axis auto;
view([60, 10]);
grid('minor');
hold on;
title('Simulated Trajectory of Kinova Gen 3 Robot')

% animate
fps = 30;
r = robotics.Rate(fps);
for i = 1:num_waypoints
    show(robotRBT, qs(i,:)', 'PreservePlot', false);
    drawnow;
    waitfor(r);
end