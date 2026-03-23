function cost = costFunctionNMPC(X,U,data,poseFinal,robot,Qr,Qt,Qu,Qv)
    % parameters
    p = data.PredictionHorizon;
    joint_qty = data.NumOfOutputs;

    % running cost
    cost_r = 0;
    for i=2:p+1 % iterate over prediction horizon
        q = X(i,1:joint_qty); % select joint angles
        transformation = getTransform(robot, q', 'EndEffector_Link'); % forward kinematics for EE pose
        angles = rotm2eul(transformation(1:3,1:3), "XYZ"); % rotational matrix to euler
        poseCurr = [transformation(1:3,4);angles']; % pose vector [X; Y; Z; roll; pitch; yaw]
        e_r = [poseFinal(1:3)-poseCurr(1:3); angdiff(poseCurr(4:6), poseFinal(4:6))]; % error between desired pose and final pose
        cost_r_i = e_r' * Qr * e_r; % quadratic cost to positional and angular errors
        cost_r = cost_r + cost_r_i + U(i,:) * Qu * U(i,:)'; % accumulative cost over horizon
    end

    % terminal cost
    cost_t = e_r' * Qt * e_r + X(p+1, joint_qty+1:end) * Qv * X(p+1,joint_qty+1:end)'; % cost of horizon end pose error and joint velocity

    % total cost
    cost = cost_r + cost_t;
end

