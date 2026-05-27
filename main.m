% test if initial static obstacle collision control has been done
if exist("t", "var") == 0
    %% --- System of Interest ---
    [robot, ~] = loadrobot("kinovaGen3", DataFormat="column", Gravity=[0 0 -9.81]);
    
    %% --- Environment (static & dynamic) ---
    % load static collision obstacles of the environment
    staticObstacleGeneration;
    
    % load robot for dynamic collision obstacle
    [obst_robot_model, ~] = loadrobot("kinovaGen3", DataFormat="column", Gravity=[0 0 -9.81]);
    modifyBaseLoc;
    
    %% --- Visualize as collision obstacles and STL files ---
    % define figure
    figure('Name', 'Environment Visualization');
    
    % robot rigid body tree at the initial joint configuration
    show(robot, homeConfiguration(robot), PreservePlot=false, Frames="on");
    title('Environment Visualization')
    hold on
    
    % environment
    visualizeStaticObstacles;
    
    % dynamic obstacle
    obst_dynamic = show(obst_robot, homeConfiguration(obst_robot), ...
        PreservePlot=false, Frames="off");
    for i=1:(numel(homeConfiguration(obst_robot)))
        obst_dynamic.Children(i).FaceColor = 'c';
    end
    
    hold off
    view([-31 63])
    axis([-0.5 0.75 -0.5 0.75 -0.04 1.3]);
    
    %% Define Start and End Poses for each robot
    startPose_robot1 = [0 0.3 0.6 pi/2 pi pi/2]; % X Y Z phi theta psi
    desiredPose_robot1 = [0.22 -0.4 0.3 -pi 0 0];
    
    startPose_robot2 = [-0.1 0.15 0.8 pi/2 -pi/2 0];
    desiredPose_robot2 = [0.15 -0.2 0.3 0 pi 0];
    
    %% Define waypoints for motion of robot 1
    waypoints_robot1 = [0 0.15 0.6 pi/2 pi pi/2; 
        0.22 -0.4 0.5 -pi 0 0];
    
    %% Task motion for robot 1
    [planned_path_robot1, states_path_robot1] = RRTstarBased_pathPlanning(robot, staticObst, startPose_robot1, desiredPose_robot1, waypoints_robot1);
    
    %% Define waypoints for motion of robot 2
    waypoints_robot2 = [0.05 0.15 0.8 pi/2 -pi/2 0; 
        0.15 -0.2 0.5 0 pi 0];
    
    %% Task motion for robot 2
    [planned_path_robot2, states_path_robot2] = RRTstarBased_pathPlanning(obst_robot, staticObst, startPose_robot2, desiredPose_robot2, waypoints_robot2, base_T, true);
    
    %% Visualize path-planning
    % define figure
    figure('Name', 'Start and Goal')
    
    % robot rigid body tree at the initial joint configuration
    show(robot, homeConfiguration(robot), PreservePlot=false, Frames="on");
    title('Environment Visualization')
    hold on
    
    % environment
    visualizeStaticObstacles;
    
    % dynamic obstacle
    obst_dynamic = show(obst_robot, homeConfiguration(obst_robot), ...
        PreservePlot=false, Frames="off");
    for i=1:(numel(homeConfiguration(obst_robot)))
        obst_dynamic.Children(i).FaceColor = 'c';
    end
    
    % display the start and goal points
    startForRobot1 = scatter3(startPose_robot1(1), startPose_robot1(2), startPose_robot1(3), 100, "green", "filled");
    plotGridCoordinates(startPose_robot1, 0.1);
    
    goalForRobot1 = scatter3(desiredPose_robot1(1), desiredPose_robot1(2), desiredPose_robot1(3), 100, "red", "filled");
    plotGridCoordinates(desiredPose_robot1, 0.1)
    
    startForRobot2 = scatter3(startPose_robot2(1), startPose_robot2(2), startPose_robot2(3), 100, "blue", "filled");
    plotGridCoordinates(startPose_robot2, 0.1)
    
    goalForRobot2 = scatter3(desiredPose_robot2(1), desiredPose_robot2(2), desiredPose_robot2(3), 100, "yellow", "filled");
    plotGridCoordinates(desiredPose_robot2, 0.1)
    
    % display paths
    visualizePaths;
    
    legend([startForRobot1, goalForRobot1, startForRobot2, goalForRobot2, task_waypoint_plot_robot1, task_waypoint_plot_robot2], ...
        'Start: Robot 1', 'End: Robot 1', 'Start: Robot 2', 'End: Robot 2', 'Waypoints Robot 1', 'Waypoints Robot 2')
    
    hold off
    view([-31 63])
    axis([-0.75 1.0 -0.75 0.75 -0.04 1.3]);
    
    %% Generate Trajectories
    % generate trajectories in trapezoidal velocity profiles for paths to obtain dynamics and time sampling
    full_states_robot1 = [taskToJointWaypoints(planned_path_robot1(1:5,:), robot); % waypoints input in task-space
        states_path_robot1;
        taskToJointWaypoints(planned_path_robot1(end-5:end,:), robot); % waypoints input in task-space
        ];

    trajectory_robot1 = generateTraj(full_states_robot1);
    
    full_states_robot2 = [taskToJointWaypoints(planned_path_robot2(1:5,:), obst_robot); % waypoints input in task-space
        states_path_robot2;
        taskToJointWaypoints(planned_path_robot2(end-5:end,:), obst_robot); % waypoints input in task-space
        ];

    trajectory_robot2 = generateTraj(full_states_robot2);
    
    % make sure samples are equal duration of longest sample length
    if max(height(trajectory_robot1), height(trajectory_robot2)) == height(trajectory_robot1)
        trajectory_robot2 = [trajectory_robot2; repmat(trajectory_robot2(end,:), height(trajectory_robot1)-height(trajectory_robot2), 1)];
    else
        trajectory_robot1 = [trajectory_robot1; repmat(trajectory_robot1(end,:), height(trajectory_robot2)-height(trajectory_robot1), 1)];
    end

    % split into components
    q_robot1 = trajectory_robot1(:,1:7);
    qd_robot1 = trajectory_robot1(:,8:14);
    qdd_robot1 = trajectory_robot1(:,15:end);

    q_robot2 = trajectory_robot2(:,1:7);
    qd_robot2 = trajectory_robot2(:,8:14);
    qdd_robot2 = trajectory_robot2(:,15:end);
    
    % discrete time
    dt = 0.05;
    t = 0:dt:(height(trajectory_robot1)-1)*dt;
    t = t';
end

%% simulate three comparative controllers
controllers = ["MPC"]; %["SSM","PID","MPC"];

% quantitative metric data collection
min_separation = zeros(length(controllers),1); % lowest separation distance (least safe proximity)
mean_separation = zeros(length(controllers),1); % average distance
violation_qty = zeros(length(controllers),1); % number of violations of safety distance
jointPosViolation_qty = zeros(length(controllers),1); % quantity of violations to joint position limits
jointVelViolation_qty = zeros(length(controllers),1); % quantity of violations to joint velocity limits
mean_controlEffort = zeros(length(controllers),1); % average speed scale limiting effect
std_controlEffort = zeros(length(controllers),1); % consistency of speed scaling
compTimes = zeros(length(controllers),1); % task time productivity
mean_trackingError = zeros(length(controllers),1); % mean RMSE of End-Effector tracking
max_trackingError = zeros(length(controllers),1); % highest RMSE of End-Effector tracking

% computational and real-time cost data collection
totalComputation = zeros(length(controllers),1); % total computation time
P95_coputationTime = zeros(length(controllers),1); % high average overhead for safety controller
mean_computationTime = zeros(length(controllers),1); % average time to compute safety controller

% save pos, vel, and acc limits for joints
q_qty = 7;
joint_position_limits = zeros(length(q_qty),1);
for i=1:q_qty
    joint_position_limits(i) = robot.Bodies{1, i}.Joint.PositionLimits(2); % max as absolute
end
joint_velocity_limits = [1.39; 1.39; 1.39; 1.39; 1.22; 1.22; 1.22]; % KinovaGen3 speed general limits
joint_acceleration_limits = [5.2; 5.2; 5.2; 5.2; 10.0; 10.0; 10.0]; % KinovaGen3 acceleration hard limits

% optional MPC uncertainty injection
uncertainty = true; % manually modify
load = 0.1; % maximum uncertainty [m]

for c = 1:length(controllers)
    %% Select Corresponding Controller
    contType = controllers(c);
    
    %% Dynamic Simulation
    % controller gains for computed torque control
    wn = [8.75 8.75 8.75 8.75 3.75 3.75 3.75];
    zeta = 1;
    
    Kp = diag(wn.^2);
    Kd = diag(2*zeta*wn);
    
    % create MPC object
    if contType == "MPC"
        generateMPC;
        safety_th = 0.25; % safety threshold distance

        % define handle of current controller state
        xc = mpcstate(mpc_obj);
    
        % set initial previous cost function weight scaling
        Q_scalePrev = 0;
        Rd_scalePrev = 0;

        % log risks
        risk_log = zeros(height(t),1);

        % log ground truth when there is uncertainty
        if uncertainty == true
            groundTruth_log = zeros(height(t),1);
        end
    end
    
    % initialize parameters and variables for PID safety control
    if contType == "PID"
        e_acc = 0; % accumulated error
        e_prev = 0; % previous timestep error
    
        % PID gains
        Kp_ssm = 2.0;
        Ki_ssm = 0.2;
        Kd_ssm = 0.2;
    
        safety_th = 0.25; % safety threshold distance
    end
    
    % initial states
    q1 = q_robot1(1,:)';
    qd1 = zeros(7,1);
    
    q2 = q_robot2(1,:)';
    qd2 = zeros(7,1);
    
    % visual elements
    figure('Name', 'Simulation Visualization');
    
    hold("on")
    view([-45 45])
    axis([-0.75 1.0 -0.75 0.75 -0.04 1.3]);
    
    title("Simulation of " + contType + " Safety Controller")
    
    % robot at initial states
    main_robot = show(robot, q1, PreservePlot=false, Frames="off");
    obst_dynamic = show(obst_robot, q2, PreservePlot=false, Frames="off");
    
    % environment
    visualizeStaticObstacles;
    
    % display paths
    visualizePaths_sim;
    
    camlight('right');
    lighting flat;
    material default;
    
    % pre-allocated log variables
    eeTrail1 = zeros(height(t),3);
    eeTrail2 = zeros(height(t),3);
    
    e1_log = zeros(height(t),7);
    ed1_log = zeros(height(t),7);
    e2_log = zeros(height(t),7);
    ed2_log = zeros(height(t),7);
    
    EEe1_log = zeros(height(t),1);
    EEe2_log = zeros(height(t),1);
    
    EE_to_end_log = zeros(height(t),1);
    
    q_robot1_log = zeros(height(t),7);
    q_robot2_log = zeros(height(t),7);
    
    % violation counters
    jointPosViolation_qty(c) = 0;
    jointVelViolation_qty(c) = 0;
    
    % Safety-related logs
    d_min_log = zeros(height(t),1);
    S_log = zeros(height(t),1);
    speedScale_log = zeros(height(t),1);
    safetyTime = zeros(height(t),1);
    
    % initial state of End-Effector trail line
    eeLine1 = plot3(NaN, NaN, NaN, 'r', 'LineWidth', 2);
    eeLine2 = plot3(NaN, NaN, NaN, 'b', 'LineWidth', 2);
    
    % main simulation loop
    r = rateControl(1/dt);
    tic
    for k=1:height(t)
        %% plot end-effector poses
        p1_T = getTransform(robot, q1, robot.BodyNames{end});
        p2_T = getTransform(obst_robot, q2, obst_robot.BodyNames{end});
    
        % add to plotline objects
        eeTrail1(k,:) = p1_T(1:3,4)';
        set(eeLine1, ...
            'XData', eeTrail1(1:k,1), ...
            'YData', eeTrail1(1:k,2), ...
            'ZData', eeTrail1(1:k,3));
    
        eeTrail2(k,:) = p2_T(1:3,4)';
        set(eeLine2, ...
            'XData', eeTrail2(1:k,1), ...
            'YData', eeTrail2(1:k,2), ...
            'ZData', eeTrail2(1:k,3));
        
        %% desired trajectories of robots at timestep
        [q1_d, qd1_d, qdd1_d] = referenceJoints(k, q_robot1, qd_robot1, qdd_robot1);
        [q2_d, qd2_d, qdd2_d] = referenceJoints(k, q_robot2, qd_robot2, qdd_robot2);
    
        %% Safety Control
        startSafety = tic;
        %% Speed and Separation Monitoring [SSM] (Mid-Level Safety Control)
        if contType == "SSM"
            [d_min, v_rel, closestJoints] = monitorRobotSeparation(robot, q1, qd1, obst_robot, q2, qd2);
            [speedScale, stop_flag, S] = SSMcontrol(d_min, v_rel);
            
            % implement speed scaling control
            qd1_d = speedScale * qd1_d;
            qdd1_d = speedScale * qdd1_d;
    
            safety_th = S; % allocate for logging
        end
    
        %% MPC-enhanced SSM (Mid-Level Safety Control)
        if contType == "MPC"
            % predict separation distance over horizon
            obst_q_pred = zeros(mpc_obj.PredictionHorizon, 7);
            obst_qd_pred = zeros(mpc_obj.PredictionHorizon, 7);
            robot_q_pred = zeros(mpc_obj.PredictionHorizon, 7); 
            robot_qd_pred = zeros(mpc_obj.PredictionHorizon, 7); 
            future_d_min = zeros(mpc_obj.PredictionHorizon, 1);

            % weighted average on collision horizon (closest highest risk)
            riskWeights = zeros(mpc_obj.PredictionHorizon, 1);
            groundTruth_d_min = zeros(mpc_obj.PredictionHorizon, 1);
            
            for i=1:mpc_obj.PredictionHorizon
                % robot state prediction
                robot_predID = min(k+i, size(q_robot1,1));
                robot_q_pred(i,:) = q_robot1(robot_predID,:);
                robot_qd_pred(i,:) = qd_robot1(robot_predID,:);

                % obstacle state prediction
                obst_predID = min(k+i, size(q_robot2,1));
                obst_q_pred(i,:) = q_robot2(obst_predID,:);
                obst_qd_pred(i,:) = qd_robot2(obst_predID,:);

                % find ground true in uncertainty scenarios
                if uncertainty == true
                    [groundTruth_d_min(i), ~, ~] = monitorRobotSeparation(robot, ...
                        robot_q_pred(i,:)', robot_qd_pred(i,:)', obst_robot, ...
                        obst_q_pred(i,:)', obst_qd_pred(i,:)');
                end

                % uncertainty injection
                if uncertainty == true
                    noiseScale = load * sqrt(i); % increased noise as horizon extends
                    obst_q_pred(i,:) = obst_q_pred(i,:) + noiseScale * (randn(1,7) + 0.5);
                    obst_qd_pred(i,:) = obst_qd_pred(i,:) + (noiseScale/dt) * (randn(1,7) + 0.5);
                end

                % monitor separation across horizon
                [future_d_min(i), ~, ~] = monitorRobotSeparation(robot, ...
                    robot_q_pred(i,:)', robot_qd_pred(i,:)', obst_robot, ...
                    obst_q_pred(i,:)', obst_qd_pred(i,:)');

                % risk weighting
                if i < mpc_obj.PredictionHorizon/2
                    riskWeights(i) = 1;
                elseif i < (mpc_obj.PredictionHorizon/2 + mpc_obj.PredictionHorizon/4)
                    riskWeights(i) = 0.5;
                else
                    riskWeights(i) = 0.2;
                end
            end
            
            % identify closest separation distance
            d_min = min(future_d_min);

            % identify risk by horizon weight
            risk = sum(riskWeights .* max(0, safety_th - future_d_min) ./ safety_th) / sum(riskWeights);
        
            if d_min <= safety_th
                speedScale = max(0.1, 1 - risk);
                
                % scale constraints
                for i=1:q_qty
                    mpc_obj.OutputVariables(q_qty+i).Min = -speedScale*joint_velocity_limits(i);
                    mpc_obj.OutputVariables(q_qty+i).Max = speedScale*joint_velocity_limits(i);
                end
            else
                % reset velocity constraints to normal values
                for i=1:q_qty
                    mpc_obj.OutputVariables(q_qty+i).Min = -joint_velocity_limits(i);
                    mpc_obj.OutputVariables(q_qty+i).Max = joint_velocity_limits(i);
                end
            end

            % Cost Function Weight Scaling
            Q_scale = 1 + 5.0*risk;
            Q_scaleFiltered = 0.5 * Q_scalePrev + 0.5 * Q_scale; % prevent harsh scaling
            
            Rd_scale = max(0.2, 1 - 0.7*risk);
            Rd_scaleFiltered = 0.5 * Rd_scalePrev + 0.5 * Rd_scale;

            mpc_obj.Weights.OutputVariables = Q * Q_scaleFiltered; % joint position and velocity tracking weight
            mpc_obj.Weights.ManipulatedVariablesRate = Rd * Rd_scaleFiltered; % smoothness weight
    
            % MPC timestep state
            ym = [q1; qd1]; % current measured outputs
            y_ref = [q1_d; qd1_d]; % output references
    
            % online MPC computation
            u_out = mpcmove(mpc_obj, xc, ym, y_ref);
    
            qdd1_d = u_out;

            % save previous cost function scales for filtering
            Q_scalePrev = Q_scaleFiltered;
            Rd_scalePrev = Rd_scaleFiltered;

            % log risk
            risk_log(k) = risk;
            
            % log ground truth if there is uncertainty
            if uncertainty == true
                groundTruth_log(k) = min(groundTruth_d_min);
            end
        end
    
        %% PID-enhanced SSM (Mid-level Safety Control)
        if contType == "PID"
            [d_min, v_rel, closestJoints] = monitorRobotSeparation(robot, q1, qd1, obst_robot, q2, qd2);
    
            % compute current error
            e = safety_th - d_min;

            % track accumulate error
            e_acc = e_acc + e*dt;
    
            % safety PID
            u_ssm = Kp_ssm*e + Ki_ssm*(e_acc + e*dt) + Kd_ssm*(e - e_prev)/dt;
    
            % clamp scaling results to limits
            u_ssm = max(0, min(1, u_ssm));
    
            % inverse scaling (large PID action should reflect in low speed)
            speedScale = 1 - u_ssm;
    
            % clamp saturation
            speedScale = max(0.1, min(1, speedScale));
    
            % implement speed scaling control
            qd1_d = speedScale * qd1_d;
            qdd1_d = speedScale * qdd1_d;
    
            % track previous error for derivatives
            e_prev = e;
        end
        safetyTime(k) = toc(startSafety);
    
        %% Error Calculations
        [e1, ed1, e1_log, ed1_log] = calcError(k, q1, qd1, q1_d, qd1_d, e1_log, ed1_log);
        [e2, ed2, e2_log, ed2_log] = calcError(k, q2, qd2, q2_d, qd2_d, e2_log, ed2_log);
        
        %% Computed Torque Control (Joint-Level PD Control)
        qdd1 = computedTorqueControl(robot, q1, qd1, qdd1_d, e1, ed1, Kp, Kd);
        qdd2 = computedTorqueControl(obst_robot, q2, qd2, qdd2_d, e2, ed2, Kp, Kd);
        
        %% additional logs
        q_robot1_log(k,:) = q1';
        p1_d_T = getTransform(robot, q1_d, robot.BodyNames{end});
        EEe1_log(k,1) = rms(p1_T(1:3,4) - p1_d_T(1:3,4)); % RMSE of End-Effector
    
        q_robot2_log(k,:) = q2';
        p2_d_T = getTransform(obst_robot, q2_d, obst_robot.BodyNames{end});
        EEe2_log(k,1) = rms(p2_T(1:3,4) - p2_d_T(1:3,4));
        
        % Safety Controller Logs
        d_min_log(k) = d_min;
        S_log(k) = safety_th;
        speedScale_log(k) = speedScale;
    
        % distance to end goal
        EE_to_end_log(k) = rms(p1_T(1:3,4) - desiredPose_robot1(1:3)'); % RMSE

        % joint position voilations
        for j = 1:q_qty
            if q1(j) < -joint_position_limits(j) || q1(j) > joint_position_limits(j)
                jointPosViolation_qty(c) = jointPosViolation_qty(c) + 1;
            end % joint velocity violations
            if qd1(j) < -joint_velocity_limits(j) || qd1(j) > joint_velocity_limits(j)
                jointVelViolation_qty(c) = jointVelViolation_qty(c) + 1;
            end
        end
    
        %% integration joint update
        [q1, qd1] = updateJoints(q1, qd1, qdd1, dt);
        [q2, qd2] = updateJoints(q2, qd2, qdd2, dt);
    
        %% visualization
        main_robot = show(robot, q1, PreservePlot=false, Frames="off");
        obst_dynamic = show(obst_robot, q2, PreservePlot=false, Frames="off");
    
        drawnow;
        %waitfor(r);
    end
    simTime = toc;
    hold off
    
    %% find completion time
    completion_th = 0.01; % distance to target in meters
    
    compID = find(EE_to_end_log < completion_th, 1);
    
    if ~isempty(compID)
        compTimes(c) = t(compID);
    else
        compTimes(c) = t(end);
    end

    %% collect metric data
    % quantitative metric data collection
    min_separation(c) = min(d_min_log);
    mean_separation(c) = mean(d_min_log); 
    violation_qty(c) = sum(d_min_log < S_log); 
    if uncertainty == true && contType == "MPC"
        min_separation(c) = min(groundTruth_log);
        mean_separation(c) = mean(groundTruth_log); 
        violation_qty(c) = sum(groundTruth_log < S_log); 
    end
    mean_controlEffort(c) = mean(speedScale_log);
    std_controlEffort(c) = std(speedScale_log); 
    mean_trackingError(c) = mean(EEe1_log); 
    max_trackingError(c) = max(EEe1_log); 

    % computational and real-time cost data collection
    totalComputation(c) = simTime; 
    P95_coputationTime(c) = prctile(safetyTime,95);
    mean_computationTime(c) = mean(safetyTime); 
    
    %% plot simulation results
    % joint errors
    figure('Name', 'Joint Error')
    tiledlayout(2,2);
    nexttile;
    plot(t, e1_log) % joint error robot 1
    title('Joint Error Main Robot')
    nexttile;
    plot(t, e2_log) % joint error robot 2
    title('Joint Error Obstacle Robot')
    nexttile;
    plot(t, ed1_log) % joint vel error robot 1
    title('Joint Velocity Error Main Robot')
    nexttile;
    plot(t, ed2_log) % joint vel error robot 2
    title('Joint Velocity Error Obstacle Robot')
    
    % RMSE End-Effector pose errors
    figure('Name', 'Pose Error')
    tiledlayout(2,1);
    nexttile;
    plot(t, EEe1_log) % joint error robot 1
    title('Pose Error Main Robot')
    nexttile;
    plot(t, EEe2_log) % joint error robot 2
    title('Pose Error Obstacle Robot')
    
    % safety separations and speed control results
    figure('Name', 'Safety Controller Results')
    tiledlayout(2,1);
    nexttile;
    plot(t, d_min_log) % minimum body distance
    hold on
    plot(t, S_log) % safety distance
    legend('Minimum Distance', 'Safety Distance')
    if uncertainty == true && contType == "MPC"
        plot(t, groundTruth_log) % ground trueth minimum body distance
        legend('Minimum Distance', 'Safety Distance', 'Ground Truth Distance')
    end
    title('Speed and Separation Monitoring')
    nexttile;
    plot(t, speedScale_log) % scale applied by control to robot speed 
    title('Velocity Scaling')

    % MPC risk values
    if contType == "MPC"
        figure('Name', 'MPC Collision Risk Measurement over Horizon')
        plot(t, risk_log)
        title('MPC Collision Risk over Horizon Measurement')
    end
end

% Task Productivity Comparison
figure('Name', 'Task Completion Time (Productivity)')
productivity = bar(compTimes);
productivity .FaceColor = 'flat';
productivity.CData(1,:) = [0 0.4470 0.7410]; % SSM bar color
productivity.CData(2,:) = [0.8500 0.3250 0.0980]; % PID bar color
productivity.CData(3,:) = [0.4660 0.6740 0.1880]; % MPC bar color
title('Controller Productivity Comparison')
set(gca, 'XTickLabel', controllers)
ylabel('Completion Time [s]')
for i = 1:length(compTimes)
    text(i, compTimes(i)-1, ...
        sprintf('%.2f s', compTimes(i)), ...
        'HorizontalAlignment','center', 'Color', 'w', 'FontWeight', 'bold');
end
grid on

% safety to productivity pareto analysis
figure('Name', 'Productivity to Safety Trade-off Analysis')
colors = [
    0 0.4470 0.7410; % SSM
    0.8500 0.3250 0.0980; % PID
    0.4660 0.6740 0.1880; % MPC
];
hold on
for i = 1:length(controllers)
    scatter(compTimes(i), min_separation(i), 150, colors(i,:), 'filled')
    text(compTimes(i)+0.01, min_separation(i), controllers(i), ...
         'FontSize', 12, 'FontWeight', 'bold');
end
title('Safety-Productivity Trafe-off Pareto Analysis')
xlabel('Task Completion Time [s]')
ylabel('Minimum Separation Distance [m]')
grid on

%% log table of metric data
ResultsTable = table( ...
    min_separation, ...
    mean_separation, ...
    violation_qty, ...
    jointPosViolation_qty, ...
    jointVelViolation_qty, ...
    mean_controlEffort, ...
    std_controlEffort, ...
    compTimes, ...
    mean_trackingError, ...
    max_trackingError, ...
    totalComputation, ...
    P95_coputationTime, ...
    mean_computationTime,...
    'VariableNames', { ...
        'Minimum Separation [m]', ...
        'Mean Separation [m]', ...
        'Distance Violation Quantity', ...
        'Joint Position Violation Quantity', ...
        'Joint Velocity Violation Quantity', ...
        'Mean Control Effort', ...
        'Std Control Effort', ...
        'Time to Reach Goal [s]', ...
        'Mean Tracking Error [m]', ...
        'Maximum Tracking Error [m]',...
        'Simulation Computation Time [s]', ...
        'P95 Controller Computation Time [s]', ...
        'Mean Controller Computation Time [s]'} ...
);

writetable(ResultsTable, 'MetricResults.csv');
