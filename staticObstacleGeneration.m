%% Create static collision obstacles representing the environment

% obstacle representing the floor of the environment space
obst_floor = collisionBox(1.25, 1.25, 0.02);
obst_floor.Pose = trvec2tform([0.125 0.125 -0.02]);

% obstacle to represent the surface of a table
obst_tableTop = collisionBox(0.5, 0.6, 0.1);
obst_tableTop.Pose = trvec2tform([0.35 -0.2 0.2]);

% obstacle to represent shelf
obst_shelf = collisionBox(0.4, 0.4, 0.1);
obst_shelf.Pose = trvec2tform([-0.05 0.25 0.4]);

% shelf walls
obst_wall1 = collisionBox(0.3, 0.1, 0.4);
obst_wall1.Pose = trvec2tform([0 0.4 0.65]);

obst_wall2 = collisionBox(0.1, 0.4, 0.4);
obst_wall2.Pose = trvec2tform([-0.2 0.25 0.65]);

% obstacles to represent the bases
obst_tableLeg1 = collisionBox(0.4, 0.4, 0.2);
obst_tableLeg1.Pose = trvec2tform([0.34 -0.2 0.1]);

obst_tableLeg2 = collisionBox(0.22, 0.3, 0.4);
obst_tableLeg2.Pose = trvec2tform([0.01 0.25 0.2]);

% End-Effector Part Pick
obst_pick1 = collisionBox(0.05, 0.05, 0.05);
obst_pick1.Pose = trvec2tform([-0.125 0.15 0.8]);

obst_pick2 = collisionBox(0.05, 0.05, 0.05);
obst_pick2.Pose = trvec2tform([0 0.325 0.6]);

% End-Effector Part Place
obst_place1 = collisionBox(0.05, 0.05, 0.05);
obst_place1.Pose = trvec2tform([0.15 -0.2 0.275]);

obst_place2 = collisionBox(0.05, 0.05, 0.05);
obst_place2.Pose = trvec2tform([0.22 -0.4 0.275]);

% single element containing all the static obstacles to compress code
staticObst = {obst_floor, obst_tableTop, obst_tableLeg1, obst_tableLeg2, obst_shelf, obst_wall1, obst_wall2, obst_pick1, obst_pick2, obst_place1, obst_place2};

% clear unnecessary variables in workspace
clearvars obst_floor obst_tableTop obst_tableLeg1 obst_tableLeg2 obst_shelf obst_wall obst_pick1 obst_pick2 obst_place1 obst_place2
