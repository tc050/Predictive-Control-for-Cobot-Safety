%% Create static collision obstacles representing the environment

% obstacle representing the floor of the environment space
obst_floor = collisionBox(2.0, 2.0, 0.04);
obst_floor.Pose = trvec2tform([0.5 0 -0.02]);

% obstacle to represent the surface of a table
obst_tableTop = collisionBox(0.5, 2.0, 0.1);
obst_tableTop.Pose = trvec2tform([0.5 0 0.2]);

% obstacles to represent the table legs
obst_tableLeg1 = collisionBox(0.2, 0.1, 0.65);
obst_tableLeg1.Pose = trvec2tform([0.5 0.8 0.325]);

obst_tableLeg2 = collisionBox(0.2, 0.1, 0.65);
obst_tableLeg2.Pose = trvec2tform([0.5 -0.8 0.325]);

% obstacle to represent shelf
obst_shelf = collisionBox(0.4, 2.0, 0.05);
obst_shelf.Pose = trvec2tform([0.5 0 0.65]);


% single element containing all the static obstacles to compress code
staticObst = {obst_floor, obst_tableTop, obst_tableLeg1, obst_tableLeg2, obst_shelf};
