% load the KINOVA Gen 3 robot manipulator multibody model and rigid body tree
[robotRBT, robotInformation] = loadrobot("kinovaGen3", "DataFormat", "column", "Gravity", [0 0 -9.81]);

% establish end effector
endEff = 'EndEffector_Link';

% initial joint positions
qInit = [-0.5538; 1.3496; 0.0001; 0.0000; 0.0004; 1.7776; -0.5537]; % converted from deg to rad
eeInit = getTransform(robotRBT, qInit, endEff); % set initial configuration

% vizualize initial configuration
show(robotRBT, qInit);
axis auto;
view([60, 10]);
