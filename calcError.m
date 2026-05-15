%% --- Calculate Error at TimeStep between Reference and Actual Joints ---

function [e, ed, e_log, ed_log] = calcError(k, q, qd, q_d, qd_d, e_log, ed_log)
    e = q_d - q; % positional error
    ed = qd_d - qd; % velocity error

    % log
    e_log(k,:) = e';
    ed_log(k,:) = ed';
end
