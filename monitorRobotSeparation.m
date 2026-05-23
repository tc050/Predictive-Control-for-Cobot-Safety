%% --- Measure Closest Separation Joints and Relative Velocities ---

function [d_min, v_rel, closestJoints] = monitorRobotSeparation(robot1, q1, qd1, robot2, q2, qd2)
    % initial minimum distance definition to identify shortest
    d_min = inf;
    closestJoints = [1 1];
    pos1_closest = zeros(3,1);
    pos2_closest = zeros(3,1);
    
    % filter through main robot bodies
    for i=1:length(robot1.BodyNames)
        % joint position in space
        T1 = getTransform(robot1, q1, robot1.BodyNames{i});
        pos1 = T1(1:3,4);

        % filter through obstacle robot bodies
        for j=1:length(robot2.BodyNames)
            % joint position in space
            T2 = getTransform(robot2, q2, robot2.BodyNames{j});
            pos2 = T2(1:3,4);

            d = norm(pos1 - pos2); % distance between bodies

            % test if minimum seen
            if d < d_min
                d_min = d;
                closestJoints = [i j];

                pos1_closest = pos1;
                pos2_closest = pos2;
            end
        end
    end

    % calculate relative velocities
    if norm(pos1_closest - pos2_closest) < 1e-6
        direction = [0;0;0];
    else
        direction = (pos1_closest - pos2_closest)/norm(pos1_closest - pos2_closest);
    end

    J1 = geometricJacobian(robot1, q1, robot1.BodyNames{closestJoints(1)});
    J2 = geometricJacobian(robot2, q2, robot2.BodyNames{closestJoints(2)});
        
    v1 = J1(1:3,:) * qd1;
    v2 = J2(1:3,:) * qd2;

    v_rel = dot((v1 - v2), direction); % relative closing velocity

    % clamp noise
    v_rel = max(v_rel, 0);
end
