%% Specific Joint Selection for Tuning
joint_n = 7;

% tuning parameters
wn = [8.75 8.75 8.75 8.75 3.75 3.75 3.75]; % natural frequency
zeta = 1; % damping ration

%% --- System of Interest ---
[robot, ~] = loadrobot("kinovaGen3", DataFormat="column", Gravity=[0 0 0]);

% step reference
jWaypoints = zeros(8,7);
jWaypoints(height(jWaypoints)/3:end,1:joint_n) = 1; % ### tune for joint ###

% generate trajectory on joints implementing velocity constaints
joint_velocity_limits = [1.39; 1.39; 1.39; 1.39; 1.22; 1.22; 1.22]; % KinovaGen3 speed general limits

% generate trajectory using trapezoidal velocity profiles
[q_traj, qd_traj, qdd_traj, T, ~] = trapveltraj(jWaypoints', size(jWaypoints,1), PeakVelocity=joint_velocity_limits);

% extablish discrete time
dt = 0.05;
t = 0:dt:T(end);
t = t';

% 1D-interpolation on non-uniform trajectory into the uniform discrete time
q_ref = interp1(T, q_traj', t);
qd_ref = interp1(T, qd_traj', t);
qdd_ref = interp1(T, qdd_traj', t);

%% controller gains for computed torque control
Kp = diag(wn.^2);
Kd = diag(2*zeta*wn);

% initial states
q = zeros(7,1);
qd = zeros(7,1);

%% logs
q_robot_log = zeros(height(t),7);
e_log = zeros(height(t),7);
ed_log = zeros(height(t),7);
EEe_log = zeros(height(t),1);

% main simulation loop
r = rateControl(1/dt);
for k=1:height(t)
    %% plot end-effector poses
    p_T = getTransform(robot, q, robot.BodyNames{end});
    
    %% desired trajectories of robots at timestep
    q_d = q_ref(k,:)';
    qd_d = qd_ref(k,:)';
    qdd_d = qdd_ref(k,:)';

    %% Error Calculations
    e = q_d - q;
    ed = qd_d - qd;

    %% Robot dynamics
    M = massMatrix(robot, q);
    C = velocityProduct(robot, q, qd);
    G = gravityTorque(robot, q);

    %% PD / Computed Torque Control
    V = qdd_d + Kd*ed + Kp*e;
    tau = M*V + C + G;

    %% Forward Dynamics
    qdd = forwardDynamics(robot, q, qd, tau);
    
    %% Logs for plots
    e_log(k,:) = e';
    ed_log(k,:) = ed';

    p_d_T = getTransform(robot, q_d, robot.BodyNames{end});
    EEe_log(k,1) = norm(p_T(1:3,4) - p_d_T(1:3,4));

    q_robot_log(k,:) = q';

    %% Integrate
    qd = qd + qdd*dt;
    q  = q  + qd*dt;

    qd = qd + qdd*dt;
    q  = q  + qd*dt;
    
    waitfor(r);
end

%% Computed Torque Control Stability Analysis (Only Robot 1)
fprintf('\n--- Joint Stability Analysis Metrics ---\n\n')
ss_error = abs(e_log(end,:));
fprintf('Steady-State Error: %.4f\n', string(ss_error(joint_n))); % ### tune for joint ###

overshoot = zeros(1,7);
for j = 1:7
    
    qmax = max(q_robot_log(:,j));
    qref = max(q_ref(:,j));

    overshoot(j) = ...
        ((qmax - qref)/abs(qref))*100;
end
fprintf('Overshoot: %.4f\n', string(overshoot(joint_n))); % ### tune for joint ###

tol = 0.02;
settlingTime = zeros(1,7);
for j = 1:7
    idx = find(abs(e_log(:,j)) > tol, 1, 'last');

    if isempty(idx)
        settlingTime(j) = 0;
    else
        settlingTime(j) = t(idx);
    end
end
fprintf('Settling Time: %.4f\n', string(settlingTime(joint_n))); % ### tune for joint ###

%% Plot Step Response
figure('Name', 'Step Response')
plotJoints = plot(t, q_robot_log(:,joint_n), LineWidth=2, Color='r'); % joints robot 1 ### tune for joint ###
hold on
plotJointsRef = plot(t, q_ref(:,joint_n), LineWidth=2, LineStyle="--", Color='b'); % joint reference robot 1 ### tune for joint ###
title('Step Response: Joint', string(joint_n)) % ### tune for joint ###
legend([plotJoints,plotJointsRef], 'Joint Response', 'Reference')
