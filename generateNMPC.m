%% --- Generate Non-Linear MPC Controller ---

% create N-MPC object
nx = joint_qty * 2; % quantity of model states [q,qDot] (14)
ny = joint_qty; % quantity of model outputs [q] (7)
nu = joint_qty; % quantity of model inputs [qDdot] (7)
n_mpc = nlmpc(nx, ny, nu);

% prediction parameters
Ts = 1; % sample time
p = 5; % prediction horizon
c = 3; % control horizon
n_mpc.Ts = Ts;
n_mpc.PredictionHorizon = p;
n_mpc.ControlHorizon = c;

% constraints for states and measured variables (inputs)
minimumStateValues = {-174.53; -2.2; -174.53; -2.5656; -174.53; -2.05; -174.53; -0.8727; -0.8727; -0.8727; -0.8727; -0.8727; -0.8727; -0.8727};
maximumStateValues = {174.53; 2.2; 174.53; 2.5656; 174.53; 2.05; 174.53; 0.8727; 0.8727; 0.8727; 0.8727; 0.8727; 0.8727; 0.8727};
n_mpc.States = struct('Min', minimumStateValues, 'Max', maximumStateValues);
n_mpc.MV = struct('Min', {-1; -1; -1; -1; -10; -10; -10}, 'Max', {1; 1; 1; 1; 10; 10; 10});

%% State Function
% state function
n_mpc.Model.StateFcn = @(x,u) robotInternalModel(x,u);

% jacobian function (state matrices A and B)
n_mpc.Jacobian.StateFcn = @(x,u) robotJacobianModel(u);

%% Output Funcion
% state function
n_mpc.Model.OutputFcn = @(x,u) x(1:joint_qty);

% jacobian function
n_mpc.Jacobian.OutputFcn = @(x,u) robotJacobianOutput(u);

%% Cost Function
% weight on end-effector pose [X Y Z phi theta psi]
Qr = diag([3 3 3 0 0 0]); % running cost
Qt = diag([5 5 5 1 1 0]); % terminal cost

% weight on joints
Qu = diag([1 1 1 1 1 1 1])/10; % input cost on accelerations
Qv = diag([1 1 1 1 1 1 1]); % terminal cost on velocities

% custom cost function

