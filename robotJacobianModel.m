function [A, B] = robotJacobianModel(u)
    % STATE PREDICTION EQUATION RELATIONAL MATRICES
    % JACOBAIN MATRICES (A & B)
    joint_qty = length(u);

    A = zeros(joint_qty*2, joint_qty*2); % empty 14x14 matrix
    A(1:joint_qty,joint_qty+1:end) = eye(joint_qty); % identity matrix to relate q_dot

    B = zeros(joint_qty*2, joint_qty); % empty 14x7 matrix
    B(joint_qty+1:end,:) = eye(joint_qty); % identity matrix to relate q_ddot
end