function [planned_path_robot, states_path_robot] = RRTstarBased_pathPlanning(robot, staticObst, startPose_robot, desiredPose_robot, waypoints_robot, base_T, obst)
    arguments
        robot rigidBodyTree
        staticObst (1,:) cell
        startPose_robot (1,6) double
        desiredPose_robot (1,6) double
        waypoints_robot (:,6) double
        base_T (4,4) double = zeros(4,4);
        obst logical =  false;
    end

    % initiate empty path memory variable
    planned_path_robot = [];

    %% linear trajectory for pick operation
    [planned_path_robot, len_path] = linearPathPlanningSegment(planned_path_robot, startPose_robot, waypoints_robot(1,:));
    
    %% RRT* path-planning algorithm for joint-space path
    % convert waypoints from task-space into joint-space
    initialConfig = homeConfiguration(robot);
    
    % inverse kinematic solver
    robot_ik = inverseKinematics("RigidBodyTree", robot);
    robot_ik.SolverParameters.AllowRandomRestart = false;
    weights = [1 1 1 1 1 1];

    [start_q, ~] = robot_ik(robot.BodyNames{end}, ...
        trvec2tform(waypoints_robot(1, 1:3))*eul2tform(waypoints_robot(1, 4:end), "XYZ"), ...
        weights, initialConfig);
    
    [desired_q, ~] = robot_ik(robot.BodyNames{end}, ...
        trvec2tform(waypoints_robot(2, 1:3))*eul2tform(waypoints_robot(2, 4:end), "XYZ"), ...
        weights, initialConfig);
    
    % --- State-space of Robot ---
    ss = manipulatorStateSpace(robot);
    
    ss.StateBounds = [-3.046 3.046; % define physical joint limitations
        -2.2 2.2;
        -3.046 3.046; 
        -2.5656 2.5656; 
        -3.046 3.046; 
        -2.05 2.05; 
        -3.046 3.046]; 
     
    % --- State-validator for Static Collisions ---
    sv = manipulatorCollisionBodyValidator(ss);
    
    sv.Environment = staticObst;
    sv.SkippedSelfCollisions = "parent";
    sv.ValidationDistance = 0.02; % validation distance threshold

    % RRT path-planning algorithm
    planner = plannerRRTStar(ss, sv);
    planner.MaxConnectionDistance = 0.5;
    planner.GoalBias = 0.5;
    planner.MaxIterations = 2000;
    planner.ContinueAfterGoalReached = false;
    
    % plan trajectory
    [pathObj, info] = plan(planner, start_q', desired_q');
    
    % smoothen path
    if info.IsPathFound
        % shorten the path
        shortennedPathObj = shortenpath(pathObj, sv);
        
        % interpolate states
        states = shortennedPathObj.States;
        st = 1 / (50 / size(states,1)); % interpolation ratio distance
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
        planned_path_robot = [planned_path_robot; zeros(size(interpolatedStates, 1), 6)];
        for i=1:size(interpolatedStates,1) % over length of path
            pose_T = getTransform(robot, interpolatedStates(i,:)', robot.BodyNames{end});
            
            planned_path_robot(len_path+i,1:3) = pose_T(1:3,4)';
            planned_path_robot(len_path+i,4:end) = tform2eul(pose_T, "XYZ");
        end
    else
        fprintf("Path not found :(" + newline)
    end
    
    %% linear trajectory for place operation
    [planned_path_robot, ~] = linearPathPlanningSegment(planned_path_robot, waypoints_robot(2,:), desiredPose_robot);

    %% return states corresponding to the path
    states_path_robot = interpolatedStates;
end
