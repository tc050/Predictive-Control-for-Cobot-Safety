function plotGridCoordinates(pose, len)
    R = eul2rotm([pose(4), pose(5), pose(6)]);

    x_axis = R(:,1) * len;
    y_axis = R(:,2) * len;
    z_axis = R(:,3) * len;

    quiver3(pose(1), pose(2), pose(3), x_axis(1), x_axis(2), x_axis(3), 'r', 'LineWidth', 2)
    quiver3(pose(1), pose(2), pose(3), y_axis(1), y_axis(2), y_axis(3), 'g', 'LineWidth', 2)
    quiver3(pose(1), pose(2), pose(3), z_axis(1), z_axis(2), z_axis(3), 'b', 'LineWidth', 2)
end