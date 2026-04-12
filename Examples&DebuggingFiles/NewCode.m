%% --- System of Interest ---
[robot, ~] = loadrobot("kinovaGen3", DataFormat="column", Gravity=[0 0 -9.81]);

% inverse kinematic solver
robot_ik = inverseKinematics("RigidBodyTree", robot);
robot_ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];

%% --- Motion Planning ---

if (exist('interpolatedStates', 'var') == 0) || (~info.IsPathFound)
    %% construct 3D binary occupancy grid map
    % define occupancy map object
    resolution = 35;
    omap = occupancyMap3D(resolution);
    
    % load static collision obstacles of the environment
    staticObstacleGeneration;
    
    % load robot for dynamic collision obstacle
    [obst_robot_model, ~] = loadrobot("kinovaGen3", DataFormat="column", Gravity=[0 0 -9.81]);
    
    base_T = eye(4);
    base_T(1:3,1:3) = eul2rotm([pi 0 0]);
    base_T(1:3,4) = [1 0 0]';
    
    obst_robot = rigidBodyTree;
    obst_robot.BaseName = 'redefinedBase';
    
    locationBase = rigidBody('locationBase');
    locationBase.Joint.setFixedTransform(base_T);
    obst_robot.addBody(locationBase, obst_robot.BaseName);
    
    obst_robot.addSubtree('locationBase', obst_robot_model);
    
    % workspace bound limits
    xRange = [-0.5 2];
    yRange = [-1 1.5];
    zRange = [0 1.2];
    
    % convert continuous 3D space into discrete sampe points (voxels)
    [x, y, z] = ndgrid( ...
        linspace(xRange(1), xRange(2), resolution*(xRange(2)-xRange(1))), ... % x axis samples
        linspace(yRange(1), yRange(2), resolution*(yRange(2)-yRange(1))), ... % y axis samples
        linspace(zRange(1), zRange(2), resolution*(zRange(2)-zRange(1)))); % z axis samples
    
    mapVoxels = [x(:), y(:), z(:)];
    
    % find occupancy of collision obstacles in the voxel space
    occupied = false(size(mapVoxels, 1), 1);
    
    % identify voxels of static obstacles
    for i = 1:length(staticObst)
        % transform into box frame
        T = inv(staticObst{i}.Pose); % inverse of the transformation, local frame rather than global
        angles = T(1:3, 1:3);
        coords = T(1:3, 4);
    
        localPoints = (angles * mapVoxels' + coords)';
    
        dims = [staticObst{i}.X/2, staticObst{i}.Y/2, staticObst{i}.Z/2];
    
        % check points that are inside box
        detectedCollision = abs(localPoints(:, 1)) <= dims(1) & abs(localPoints(:, 2)) <= dims(2) & abs(localPoints(:, 3)) <= dims(3);
    
        occupied = occupied | detectedCollision;
    end
    
    globalRefPose = zeros(4);
    % identify voxels of robot obstacle
    for i = 1:length(obst_robot.Bodies)-1 % not include the EndEffector_Link which is just a positional body
        % obstain body pose relevant to parent (accumulative)
        if i == 1
            globalRefPose = obst_robot.Bodies{1,i}.Joint.JointToParentTransform;
            continue
        else
            globalRefPose = globalRefPose * obst_robot.Bodies{1,i}.Joint.JointToParentTransform;
        end
    
        % obtain mesh data from bodies
        bodyObj = getVisual(obst_robot.Bodies{1,i});
    
        vertices = bodyObj.Triangulation.Points; % Nx3 array of vertex coordinates
        faces = bodyObj.Triangulation.ConnectivityList; % Mx3 array of faces
    
        % find local points
        T = inv(globalRefPose);
        angles = T(1:3, 1:3);
        coords = T(1:3, 4);
        localPoints = (angles * mapVoxels' + coords)';
    
        % check if points are inside mesh
        vol_th_mesh = delaunayTriangulation(vertices);
        detectedCollision = ~isnan(pointLocation(vol_th_mesh, localPoints));
    
        occupied = occupied | detectedCollision;
    end
    
    % insert obstacles into occupancy map
    setOccupancy(omap, mapVoxels(occupied,:), 1);
    
    % consider unkown spaces as occupied
    omap.FreeThreshold = omap.OccupiedThreshold;
    
    % clear unnecessary variables in workspace
    clearvars angles bodyObj coords detectedCollision dims faces globalRefPose i localPoints mapVoxels occupied resolution T vertices x xRange y yRange z zRange vol_th_mesh obst_robot_model locationBase base_T
    
    %% define start and end poses
    startPose = [0.4 0.4 0.3 pi 0 pi]; % X Y Z phi theta psi
    desiredPose = [0.1 -0.1 0.8 pi 0 0];
    
    % visualize the occupancy map
    figure('Name', 'Start and Goal')
    map = show(omap);
    hold on
    
    % display the start and goal points
    scatter3(map, startPose(1), startPose(2), startPose(3), 20, "green", "filled")
    scatter3(map, desiredPose(1), desiredPose(2), desiredPose(3), 20, "red", "filled")
    hold off
    view([-31 63])
    
    % convert from task-space into joint-space
    [start_q, ~] = robot_ik(robot.BodyNames{end}, ...
        trvec2tform(startPose(1:3))*eul2tform(startPose(4:end), "XYZ"), ...
        weights, homeConfiguration(robot));
    
    [desired_q, ~] = robot_ik(robot.BodyNames{end}, ...
        trvec2tform(desiredPose(1:3))*eul2tform(desiredPose(4:end), "XYZ"), ...
        weights, homeConfiguration(robot));
    
    clearvars map
    
    %% define state-space object of the robot
    ss = manipulatorStateSpace(robot);
    ss.StateBounds = [-174.53 174.53; % define physical joint limitations
        -2.2 2.2; -174.53 174.53; 
        -2.5656 2.5656; 
        -174.53 174.53; 
        -2.05 2.05; 
        -174.53 174.53]; 
    % -inf inf; % define physical joint limitations
    %     -128.9*pi/180 128.9*pi/180;
    %     -inf inf; 
    %     -147.8*pi/180 147.8*pi/180; 
    %     -inf inf; 
    %     -120.3*pi/180 120.3*pi/180; 
    %     -inf inf
    
    %% define state-validator object of the robot
    sv = customRobotMapValidator(ss, robot, omap);
    sv.ValidationDistance = 0.05; % validation distance threshold
    
    %% RRT path-planning algorithm
    planner = plannerRRTStar(ss, sv);
    planner.MaxConnectionDistance = 10;
    planner.BallRadiusConstant = 0.1;
    planner.GoalBias = 0.5;
    planner.MaxIterations = 2000;
    planner.ContinueAfterGoalReached = false;
    
    % function for flagging achieved distance to desired pose
    weights_position = [5 5 3];
    weights_orientation = [0.5 0.5 0.5];
    proximity_th = 0.5;
    planner.GoalReachedFcn = @(~,x,y) plannerGoalFunction(robot, x, y, weights_position, weights_orientation, proximity_th);
    
    % plan trajectory
    [pathObj, info] = plan(planner, start_q', desired_q');
    
    %% visualise planned path
    if info.IsPathFound
        % shorten the path
        shortennedPathObj = shortenpath(pathObj, sv);
    
        figure('Name', 'Manipulator RRT Planned Path')
    
        % visualize the occupancy map
        map = show(omap);
        hold on
        
        % display the start and goal points
        scatter3(map, startPose(1), startPose(2), startPose(3), 20, "green", "filled")
        scatter3(map, desiredPose(1), desiredPose(2), desiredPose(3), 20, "red", "filled")
        
        % interpolate states
        states = shortennedPathObj.States;
        st = 1 / (200 / size(states,1)); % interpolation ratio distance
        interpolatedStates = zeros(length(0:st:1)*(size(states,1)-1), size(states,2));
    
        j = 1; % initial index in interpolatedPath
        for i=1:size(states,1)-1
            % topmost index in interpolatedPath
            j_top = j + length(0:st:1) - 1;
            
            % interpolate intermediary states
            interpolatedStates_data = interpolate(ss, states(i,:), states(i+1,:), 0:st:1);
            
            % log
            interpolatedStates(j:j_top,:) = interpolatedStates_data;
    
            % update interpolatedPath lower index
            j = j_top + 1;
        end
    
        % convert all joint path states from joint-space to task-space (forward kinematics)
        poses = zeros(size(interpolatedStates, 1), robot.NumBodies, 6);
        for i=1:size(interpolatedStates,1) % over length of path
            for j=1:robot.NumBodies % over all joints
                pose_T = getTransform(robot, interpolatedStates(i,:)', robot.BodyNames{j});
        
                poses(i,j,1:3) = pose_T(1:3,4)';
                poses(i,j,4:end) = tform2eul(pose_T, "XYZ");
            end
        end
    
        % plot interpolated path
         pathPlot_1 = plot3(poses(:,1,1), poses(:,1,2), poses(:,1,3), LineWidth=2, Color='b');
         pathPlot_2 = plot3(poses(:,2,1), poses(:,2,2), poses(:,2,3), LineWidth=2, Color='m');
         pathPlot_3 = plot3(poses(:,3,1), poses(:,3,2), poses(:,3,3), LineWidth=2, Color='w');
         pathPlot_4 = plot3(poses(:,4,1), poses(:,4,2), poses(:,4,3), LineWidth=2, Color='c');
         pathPlot_5 = plot3(poses(:,5,1), poses(:,5,2), poses(:,5,3), LineWidth=2, Color='k');
         pathPlot_6 = plot3(poses(:,6,1), poses(:,6,2), poses(:,6,3), LineWidth=2, Color='y');
         pathPlot_7 = plot3(poses(:,7,1), poses(:,7,2), poses(:,7,3), LineWidth=2, Color='r');
         pathPlot_8 = plot3(poses(:,8,1), poses(:,8,2), poses(:,8,3), LineWidth=2, Color='g');
         legend([pathPlot_1, pathPlot_2, pathPlot_3, pathPlot_4, pathPlot_5, pathPlot_6, pathPlot_7, pathPlot_8], ...
             ["RRT Planned Path: Joint 1", "RRT Planned Path: Joint 2", "RRT Planned Path: Joint 3", ...
             "RRT Planned Path: Joint 4", "RRT Planned Path: Joint 5", "RRT Planned Path: Joint 6", ...
             "RRT Planned Path: Joint 7", "RRT Planned Path: Joint 8"])
    
        hold off
        view([-31 63])
    else
        fprintf("Path not found :(" + newline)
    end
    
    clearvars -except robot robot_ik weights obst_robot interpolatedStates poses omap info pathObj startPose desiredPose
end

%% --- Trajectory Planning ---

% joint speed general limits
joint_velocity_limits = [1.39; 1.39; 1.39; 1.39; 1.22; 1.22; 1.22];

% generate trajectory using trapezoidal velocity profiles
[q_traj, qd_traj, qdd_traj, T, ~] = trapveltraj(interpolatedStates', size(interpolatedStates,1), PeakVelocity=joint_velocity_limits);

% extablish discrete time
dt = 0.01; % time step
t = 0:dt:T(end);

% 1D-interpolation on non-uniform trajectory into the uniform discrete time
q = interp1(T, q_traj', t);
qd = interp1(T, qd_traj', t);
qdd = interp1(T, qdd_traj', t);

%% --- Visualization ---

% prepare figure for plotting
figure('Name', 'Robot Motion Simulation')
    
% visualize the occupancy map
map = show(omap);
hold on

% display the start and goal points
scatter3(map, startPose(1), startPose(2), startPose(3), 20, "green", "filled")
scatter3(map, desiredPose(1), desiredPose(2), desiredPose(3), 20, "red", "filled")

% predefined plot for trajectory path
pathEE = zeros(size(q,1), 3); % store positional values of End-Effector
h = plot3(0, 0, 0, 'm', LineWidth=2);

for i=1:size(t,2)
    % show robot configuration at timestep
    show(robot, q(i,:)', PreservePlot=false, Frames="off")
    
    % End-Effector pose
    T = getTransform(robot, q(i,:)', robot.BodyNames{end});
    pathEE(i,:) = T(1:3,4)';
    
    % update trajectory plot
    set(h, 'XData', pathEE(1:i,1), 'YData', pathEE(1:i,2), 'ZData', pathEE(1:i,3));

    drawnow;

    % synchronize timing
    if i < size(q,2)
        pause(t(i+1) - t(i));
    end
end
hold off
