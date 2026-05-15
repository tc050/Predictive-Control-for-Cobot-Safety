%% --- Robot Dynamics and Control over Joint Stability ---

function qdd = computedTorqueControl(robot, q, qd, qdd_d, e, ed, Kp, Kd)
    % Robot dynamics (viscous friction assumed as ideal)
    M = massMatrix(robot, q); % joint centres of masses
    C = velocityProduct(robot, q, qd); % coriolis term
    G = gravityTorque(robot, q); % joint centres of gravity

    % PD / Computed Torque Control
    V = qdd_d + Kd*ed + Kp*e;
    tau = M*V + C + G;

    % Forward Dynamics
    qdd = forwardDynamics(robot, q, qd, tau);
end

