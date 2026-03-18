% --- Visualize simulation changes across time ---

% simulation control rate
fps = 50;
r = robotics.Rate(fps);

% simulate motion of the robot across state solutions
for i = 1:size(robotStateAll,1)
    obst_dynamic = show(obst_robot, robotStateAll(i,1:7)', PreservePlot=false, Frames="off", Position=[1 0 0 pi], Visuals="on");
    % for j=1:(numel(homeConfiguration(obst_robot))+1)
    %     obst_dynamic.Children(i).FaceColor = 'y';
    % end

    waitfor(r);
end

hold off
