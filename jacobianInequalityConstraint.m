function [G,Gmv,Ge] = jacobianInequalityConstraint(X,data,obstacles,robot)
    % parameters
    p = data.PredictionHorizon;
    joint_qty = data.NumOfOutputs;
    body_qty = robot.NumBodies;
    obstacle_qty = numel(obstacles);

    % initialize jacobian matrices
    G = zeros(p, joint_qty*2, p*body_qty*obstacle_qty);
    Gmv = zeros(p, joint_qty, p*body_qty*obstacle_qty);
    Ge = zeros(p*body_qty*obstacle_qty,1);

    iteration = 1;
    for i=1:p
        collisionConfiguration = X(i+1,1:joint_qty);
        [~, ~, waypointsAll] = checkCollision(robot, collisionConfiguration', ...
            obstacles, IgnoreSelfCollision='On', Exhaustive='on', SkippedSelfCollisions='parent');
        for j=1:obstacle_qty
            for k=1:obstacle_qty
                waypoints = waypointsAll(1+(j-1)*3:3+(j-1)*3, 1+(k-1)*2:2+(k-1)*2);
                if isempty(waypoints(isinf(waypoints)|isnan(waypoints)))
                    if any((waypoints(:,1)-waypoints(:,2))~=0)
                        normal = (waypoints(:,1)-waypoints(:,2))/norm(waypoints(:,1)-waypoints(:,2));
                    else
                        normal = [0;0;0];
                    end
                    jacobianBody = geometricJacobian(robot, ...
                        collisionConfiguration', robot.BodyNames{j});
                    G(i, 1:joint_qty, iteration) = -normal' * jacobianBody(4:6,:);
                else
                    G(i, 1:joint_qty, iteration) = zeros(1, joint_qty);
                end
                iteration = iteration + 1;
            end
        end
    end
end
