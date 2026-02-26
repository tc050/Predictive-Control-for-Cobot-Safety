%% Create dynamic obstacle representing the human in the collaborative task 

% load another KinovaGen3 robot representing obstacle
[obst_robot, ~] = loadrobot("kinovaGen3", "DataFormat", "column");

% obstacle robot inverse kinematics
obst_robot_InverseKinematics = inverseKinematics("RigidBodyTree", obst_robot);
obst_robot_InverseKinematics.SolverParameters.AllowRandomRestart = false;

% obstacle robot end-effector task-space coordinates transformation matrix
obst_robot_coord = trvec2tform([0.7 0.4 0.3])*axang2tform([0 1 0 pi]);

% obstacle robot initial configuration
obst_robot_config = obst_robot_InverseKinematics(endEff, obst_robot_coord, weights, homeConfiguration(obst_robot));
obst_robot_config = wrapToPi(obst_robot_config);
