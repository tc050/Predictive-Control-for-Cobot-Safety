%% --- Speed and Separation Monitoring Algorithm ---

function [speedScale, stop_flag, S] = SSMcontrol(d_min, v_rel)
    % parameters
    Tr = 0.15; % reaction time
    a_max = 1.5; % maximum decceleration time in m/s^2
    v_rel = max(v_rel,0);
    B = (v_rel^2)/(2*a_max); % braking term
    C = 0.05; % 5cm uncertainty margin

    % safety distance
    S = Tr * v_rel + B + C;

    % safety tier barriers
    stop_flag = false;

    if d_min <= S
        % emergency stop zone
        speedScale = 0;
        stop_flag = true;

    elseif d_min <= 1.5*S
        % slow zone
        speedScale = (d_min - S) / (0.5 * S);
        speedScale = max(0, min(1, speedScale));

    else
        % safe zone
        speedScale = 1;

    end

    % clamp speed limit result to limits
    speedScale = max(0, min(1, speedScale));
end
