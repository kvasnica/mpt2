function sub_computeTrajectory_test6

% tests sub_computeTrajectory with Options.sysStruct and Options.sysHandle

load ctrl_data
ctrl = ctrl_tr0_du0;

lti2Dstable;               % load another 2d system
simSysStruct = sysStruct;  % use this system for simulations

simHandle = @di_sim_fun;   % use this function for another simulations

x0 = [1; 1];
N = 4;
Options.minnorm = 0.2;

%---------------------------------------------------
[X, U, Y] = sub_computeTrajectory(ctrl, x0);
mbg_assertequal(size(X, 1), 13);
mbg_asserttolequal(X(end, :), [0 0], 1e-2);
mbg_assertequal(size(Y, 2), 1);    % ctrl.sysStruct has 1 outputs
mbg_assertequal(size(U, 1), 12);   % U must have correct dimension
mbg_assertequal(size(Y, 1), 12);    % Y must have correct dimension


%---------------------------------------------------
[X, U, Y] = sub_computeTrajectory(ctrl, x0, N);
mbg_assertequal(size(X, 1), N+1);
mbg_assertequal(size(U, 1), N);    % U must have correct dimension
mbg_assertequal(size(Y, 1), N);    % Y must have correct dimension


%---------------------------------------------------
[X, U, Y] = sub_computeTrajectory(ctrl, x0, [], Options);
mbg_assertequal(size(X, 1), 7);
mbg_assertequal(size(U, 1), 6);    % U must have correct dimension
mbg_assertequal(size(Y, 1), 6);    % Y must have correct dimension



%---------------------------------------------------
X = sub_computeTrajectory(ctrl, x0, N, Options);
mbg_assertequal(size(X, 1), N+1);


%---------------------------------------------------
[X, U, Y] = sub_computeTrajectory(ctrl, x0, [], struct('sysStruct', simSysStruct));
mbg_assertequal(size(X, 1), 11);
mbg_asserttolequal(X(end, :), [0 0], 1e-2);
mbg_assertequal(size(U, 1), 10);    % U must have correct dimension
mbg_assertequal(size(Y, 1), 10);    % Y must have correct dimension


%---------------------------------------------------
[X, U, Y] = sub_computeTrajectory(ctrl, x0, N, struct('sysStruct', simSysStruct));
mbg_assertequal(size(X, 1), N+1);
mbg_assertequal(size(U, 1), N);    % U must have correct dimension
mbg_assertequal(size(Y, 1), N);    % Y must have correct dimension


%---------------------------------------------------
X = sub_computeTrajectory(ctrl, x0, [], struct('sysStruct', simSysStruct, 'minnom', Options.minnorm));
if size(X, 1) ~= 6,
    warning('Options.minnorm is not respected.');
end


%---------------------------------------------------
[X, U, Y] = sub_computeTrajectory(ctrl, x0, [], struct('sysHandle', simHandle));
mbg_assertequal(size(X, 1), 13);
mbg_assertequal(size(Y, 2), 1);    % di_sim_fun returns only one output
mbg_asserttolequal(X(end, :), [0 0], 1e-2);
mbg_assertequal(size(U, 1), 12);   % U must have correct dimension


%---------------------------------------------------
[X, U, Y] = sub_computeTrajectory(ctrl, x0, N, struct('sysHandle', simHandle));
mbg_assertequal(size(X, 1), N+1);
mbg_assertequal(size(Y, 2), 1);    % di_sim_fun returns only one output
mbg_assertequal(size(U, 1), N);    % U must have correct dimension


%---------------------------------------------------
[X, U, Y] = sub_computeTrajectory(ctrl, x0, [], struct('sysHandle', simHandle, 'minnorm', Options.minnorm));
mbg_assertequal(size(X, 1), 7);
mbg_assertequal(size(Y, 2), 1);    % di_sim_fun returns only one output
mbg_assertequal(size(U, 1), 6);    % U must have correct dimension
