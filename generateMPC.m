%% --- Create Model Predictive Controller Object ---

%% State-space model
% system matrix (states)
A = zeros(joint_qty*2, joint_qty*2); % 14x14
A(1:joint_qty,joint_qty+1:end) = eye(joint_qty); % [0 I; 0 0]

% control matrix (inputs)
B = zeros(joint_qty*2, joint_qty); % 14x7
B(joint_qty+1:end,1:joint_qty) = eye(joint_qty); % [0; I]

% observational matrix (outputs)
C = zeros(joint_qty, joint_qty*2); % 7x14
C(1:joint_qty,1:joint_qty) = eye(joint_qty); % [I 0]

% feed-forward matrix (direct paths)
D = zeros(joint_qty, joint_qty); % 7x7 [0]

model = ss(A, B, C, D); % internal model of the robot used for predictions
Ts = 0.2; % sample time

%% Define MPC object
mpc_obj = mpc(model, Ts);

% MPC parameters
mpc_obj.PredictionHorizon = 10; % length of predictions
mpc_obj.ControlHorizon = 2; % length of control solutions

%% Constraints
% output variables (states)
minStates = {-174.53; -2.2; -174.53; -2.5656; -174.53; -2.05; -174.53};
maxStates = {174.53; 2.2; 174.53; 2.5656; 174.53; 2.05; 174.53};
mpc_obj.OutputVariables = struct('Min', minStates, 'Max', maxStates);

% manipulated variables
minInputs = {-1; -1; -1; -1; -10; -10; -10};
maxInputs = {1; 1; 1; 1; 10; 10; 10};
mpc_obj.ManipulatedVariables = struct('Min', minInputs, 'Max', maxInputs);
