%% --- Update Step for Joints based on Discrete Integration ---

function [q, qd] = updateJoints(q, qd, qdd, dt)
    % integrate
    q  = q  + qd*dt; % positions
    qd = qd + qdd*dt; % velocities
end
