%% --- Modify Base Location of Obstacle Robot ---

% new base location
base_T = eye(4);
base_T(1:3,1:3) = eul2rotm([0 0 0]);
base_T(1:3,4) = [0.25 0.25 0]';

obst_robot = rigidBodyTree;
obst_robot.BaseName = 'redefinedBase';

locationBase = rigidBody('locationBase');
locationBase.Joint.setFixedTransform(base_T);
obst_robot.addBody(locationBase, obst_robot.BaseName);

obst_robot.addSubtree('locationBase', obst_robot_model);

obst_robot.DataFormat = 'column';
obst_robot.Gravity = [0 0 0];
