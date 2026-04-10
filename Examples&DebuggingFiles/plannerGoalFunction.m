function goalReached = plannerGoalFunction(robot, x, y, weights_position, weights_orientation, th)
    % current states
    T = getTransform(robot, x', robot.BodyNames{end});
    EE_taskSpacePosition = T(1:3,4)';
    EE_taskSpaceOrientation = rotm2eul(T(1:3,1:3), "XYZ");
    
    % goal states
    Tg = getTransform(robot, y', robot.BodyNames{end});
    goalEE_taskSpacePosition = Tg(1:3,4)';
    goalEE_taskSpaceOrientation = rotm2eul(Tg(1:3,1:3), "XYZ");

    goalReached = norm(weights_position .* (EE_taskSpacePosition(1:3) - goalEE_taskSpacePosition(1:3))) + norm(weights_orientation .* (EE_taskSpaceOrientation(1:3) - goalEE_taskSpaceOrientation(1:3))) < th;
end