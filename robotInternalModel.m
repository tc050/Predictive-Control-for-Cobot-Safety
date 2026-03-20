function stateDot = robotInternalModel(x, u)
    % STATE PREDICTION EQUATION MODEL
    joint_qty = length(u);
    stateDot = zeros(size(x));

    stateDot(1:joint_qty) = x(joint_qty+1:end); % current qDot
    stateDot(joint_qty+1:end) =  u; % input joint accelerations
end