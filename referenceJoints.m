%% --- Obtain Reference Joints at TimeStep from Trajectories ---

function [q_d,qd_d, qdd_d] = referenceJoints(k, q_robot,qd_robot, qdd_robot)
    q_d = q_robot(k,:)'; % position
    qd_d = qd_robot(k,:)'; % velocities
    qdd_d = qdd_robot(k,:)'; % accelerations
end

