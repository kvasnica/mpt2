function sim_test1

% available combinations of input arguments:
% sim(ctrl, x0)
% sim(ctrl, x0, N)
% sim(ctrl, x0, Options)
% sim(ctrl, x0, N, Options)
% sim(ctrl, system, x0)
% sim(ctrl, system, x0, N)
% sim(ctrl, system, x0, N, Options)


load ctrlN2

lti2Dstable;               % load another 2d system
simSysStruct = sysStruct;  % use this system for simulations
simHandle = @di_sim_fun;   % use this function for another simulations
x0 = [1; 1];
N = 4;
Options.minnorm = 0.2;


%---------------------------------------------------
[X, U, Y] = sim(ctrl, x0);
mbg_assertequal(size(X, 1), 13);
mbg_asserttolequal(X(end, :), [0 0], 1e-2);
mbg_assertequal(size(Y, 2), 2);    % ctrl.sysStruct has 2 outputs
mbg_assertequal(size(U, 1), 12);   % U must have correct dimension
mbg_assertequal(size(Y, 1), 12);    % Y must have correct dimension


%---------------------------------------------------
[X, U, Y] = sim(ctrl, x0, N);
mbg_assertequal(size(X, 1), N+1);
mbg_assertequal(size(U, 1), N);    % U must have correct dimension
mbg_assertequal(size(Y, 1), N);    % Y must have correct dimension


%---------------------------------------------------
[X, U, Y] = sim(ctrl, x0, Options);
mbg_assertequal(size(X, 1), 7);
mbg_assertequal(size(U, 1), 6);    % U must have correct dimension
mbg_assertequal(size(Y, 1), 6);    % Y must have correct dimension



%---------------------------------------------------
X = sim(ctrl, x0, N, Options);
mbg_assertequal(size(X, 1), N+1);


%---------------------------------------------------
[X, U, Y] = sim(ctrl, simSysStruct, x0);
mbg_assertequal(size(X, 1), 11);
mbg_asserttolequal(X(end, :), [0 0], 1e-2);
mbg_assertequal(size(U, 1), 10);    % U must have correct dimension
mbg_assertequal(size(Y, 1), 10);    % Y must have correct dimension


%---------------------------------------------------
[X, U, Y] = sim(ctrl, simSysStruct, x0, N);
mbg_assertequal(size(X, 1), N+1);
mbg_assertequal(size(U, 1), N);    % U must have correct dimension
mbg_assertequal(size(Y, 1), N);    % Y must have correct dimension


%---------------------------------------------------
X = sim(ctrl, simSysStruct, x0, Options);
mbg_assertequal(size(X, 1), 6);


%---------------------------------------------------
[X, U, Y] = sim(ctrl, simHandle, x0);
mbg_assertequal(size(X, 1), 13);
mbg_assertequal(size(Y, 2), 1);    % di_sim_fun returns only one output
mbg_asserttolequal(X(end, :), [0 0], 1e-2);
mbg_assertequal(size(U, 1), 12);   % U must have correct dimension


%---------------------------------------------------
[X, U, Y] = sim(ctrl, simHandle, x0, N);
mbg_assertequal(size(X, 1), N+1);
mbg_assertequal(size(Y, 2), 1);    % di_sim_fun returns only one output
mbg_assertequal(size(U, 1), N);    % U must have correct dimension


%---------------------------------------------------
[X, U, Y] = sim(ctrl, simHandle, x0, Options);
mbg_assertequal(size(X, 1), 7);
mbg_assertequal(size(Y, 2), 1);    % di_sim_fun returns only one output
mbg_assertequal(size(U, 1), 6);    % U must have correct dimension
