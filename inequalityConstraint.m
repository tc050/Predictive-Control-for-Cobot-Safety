function inequality = inequalityConstraint(X,data,safety_dist,obstacles,robot)
    % parameters
    p = data.PredictionHorizon;
    joint_qty = data.NumOfOutputs;
    body_qty = robot.NumBodies;
    obstacle_qty = numel(obstacles);
    distances = zeros(p*body_qty*obstacle_qty,1);

    for i=1:p
        collisionConfiguration = X(i+1,1:joint_qty);
        [~, separation_dist, ~] = checkCollision(robot, collisionConfiguration', ...
            obstacles, IgnoreSelfCollision='On', Exhaustive='on', SkippedSelfCollisions='parent');
        distances_i = separation_dist(1:body_qty,1:obstacle_qty);
        distances_i(isinf(distances_i)|isnan(distances_i)) = 1e5; % large random distance
        distances((1+(i-1)*body_qty*obstacle_qty):body_qty*obstacle_qty*i,1) = reshape(distances_i', ...
            [body_qty*obstacle_qty,1]);
    end
    inequality = -distances + safety_dist;
end

