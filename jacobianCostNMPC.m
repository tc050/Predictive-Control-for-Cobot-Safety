function [G, Gmv, Ge] = jacobianCostNMPC(X,U,data,poseFinal,robot,Qr,Qt,Qu,Qv)
    % parameters
    p = data.PredictionHorizon;
    joint_qty = data.NumOfOutputs;

    % initial matrices
    G = zeros(p,joint_qty*2); % stored derivatives of state costs
    Gmv = zeros(p, joint_qty); % gradient cost of control inputs
    Ge = 0; % since cost does not depend on slack, derivative is zero

    % update G
    for i=1:p
        q = X(i,1:joint_qty); % select joint angles
        transformation = getTransform(robot, q', 'EndEffector_Link'); % forward kinematics for EE pose
        angles = rotm2eul(transformation(1:3,1:3), "XYZ"); % rotational matrix to euler
        poseCurr = [transformation(1:3,4);angles']; % pose vector [X; Y; Z; roll; pitch; yaw]
        e_r = [poseFinal(1:3)-poseCurr(1:3); angdiff(poseCurr(4:6), poseFinal(4:6))]; % error between desired pose and final pose
        
        % geometric to analytical
        rx = angles(1); % roll
        py = angles(2); % pitch
        B = [1 0 sin(py); 0 cos(rx) -cos(py)*sin(rx); 0 sin(rx) cos(py)*cos(rx)]; % converts angular velocities (w) to euler angles (droll, dpitch, dyaw)

        % robot jacobian
        robotJacobian_i = geometricJacobian(robot, q', 'EndEffector_Link'); % map of joint velocities to EE twist
        robotJacobian = robotJacobian_i;
        robotJacobian(1:3,:) = robotJacobian_i(4:6,:); % swap order (angles, position) to (position, angles)
        robotJacobian(4:6,:) = B\robotJacobian_i(1:3,:); % joint velocities to pose rate

        % cost of running jacobian
        G(i,1:joint_qty) = (-2 * e_r' * Qr * robotJacobian); % cost of running / derivative of pose error penalty
        Gmv(i,:) = 2 * U(i+1,:) * Qu; % cost of control effort / derivative of control penalty
    end

    % cost terminal jacobian
    G(p,1:joint_qty) = G(p,1:joint_qty) + (-2 * e_r' * Qt * robotJacobian); % added derivative of terminal pose error
    G(p,joint_qty+1:end) = 2* X(p+1,joint_qty+1:end) * Qv; % derivative of terminal velocities
end

