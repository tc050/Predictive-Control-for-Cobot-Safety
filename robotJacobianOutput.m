function [C, D] = robotJacobianOutput(u)
    % OUTPUT EQUATION RELATIONAL MATRICES
    % JACOBIAN MATRICES (C & D)
    joint_qty = length(u);

    C = zeros(joint_qty, joint_qty*2); % empty 7x14 matrix
    C(1:joint_qty,1:joint_qty) = eye(joint_qty); % identity matrix to relate q

    D = zeros(joint_qty, joint_qty); % empty 7x7 matrix
end

