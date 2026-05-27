%% --- Define Internal Plant and MPC Object ---

% state-space formulation
q_qty = 7; % joint quantity

%% State-space model
% system matrix (states)
A = zeros(q_qty*2, q_qty*2); % 14x14

% [I Idt; 0 I]
A(1:q_qty,1:q_qty) = eye(q_qty);
A(1:q_qty,q_qty+1:end) = eye(q_qty)*dt;
A(q_qty+1:end,q_qty+1:end) = eye(q_qty);

% control matrix (inputs)
B = zeros(q_qty*2, q_qty); % 14x7

% [(1/2)Idt^2; Idt]
B(1:q_qty,1:q_qty) = 1/2*eye(q_qty)*dt; 
B(q_qty+1:end,1:q_qty) = eye(q_qty)*dt; 

% observational matrix (outputs)
C = zeros(q_qty*2, q_qty*2); % 14x14

% [I 0; 0 I]
C(1:q_qty,1:q_qty) = eye(q_qty);
C(q_qty+1:end,q_qty+1:end) = eye(q_qty);

% feed-forward matrix (direct paths)
D = zeros(q_qty*2, q_qty); % 14x7 [0; 0]

Ts = 0.2; % sample time

model = ss(A, B, C, D, dt); % internal model of the robot used for predictions

%% Define MPC Object
mpc_obj = mpc(model, dt);

% horizon definition
mpc_obj.PredictionHorizon = 10; % length of predictions
mpc_obj.ControlHorizon = 3; % length of control solutions

%% Constraints
% joint constraints
for i=1:q_qty
    % OV
    % positional constraints [rad]
    mpc_obj.OutputVariables(i).Min = robot.Bodies{1, i}.Joint.PositionLimits(1);
    mpc_obj.OutputVariables(i).Max = robot.Bodies{1, i}.Joint.PositionLimits(1);

    % velocity constraints [rad/s]
    mpc_obj.OutputVariables(i+q_qty).Min = -joint_velocity_limits(i);
    mpc_obj.OutputVariables(i+q_qty).Max = joint_velocity_limits(i);

    % MV
    % acceleration constraints [rad/s^2]
    mpc_obj.ManipulatedVariables(i).Min = -joint_acceleration_limits(i);
    mpc_obj.ManipulatedVariables(i).Max = joint_acceleration_limits(i);
end

%% Cost Function Weight Matrices
Q = [8 8 8 8 5 5 5 2 2 2 2 2 2 2];
%Q = ones(1,14);
R = ones(1,7)*0.05;
%R = ones(1,7)*0.1;
Rd = ones(1,7)*0.1;
%Rd = zeros(1,7);

mpc_obj.Weights.OutputVariables = Q; % joint position and velocity tracking weight
mpc_obj.Weights.ManipulatedVariables = R; % control effort weight
mpc_obj.Weights.ManipulatedVariablesRate = Rd; % smoothness weight
