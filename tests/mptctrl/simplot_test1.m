function simplot_test1

% available combinations of input arguments:
% simplot(ctrl)
% simplot(ctrl, Options)
% simplot(ctrl, x0)
% simplot(ctrl, x0, N)
% simplot(ctrl, x0, Options)
% simplot(ctrl, x0, N, Options)
% simplot(ctrl, system, x0)
% simplot(ctrl, system, x0, N)
% simplot(ctrl, system, x0, N, Options)


load ctrlN2

lti2Dstable;               % load another 2d system
simSysStruct = sysStruct;  % use this system for simulations
simHandle = @di_sim_fun;   % use this function for another simulations
x0 = [1; 1];
N = 4;
Options.minnorm = 0.2;


%---------------------------------------------------
[X, U, Y] = simplot(ctrl, struct('x0', [1;1]));
close all


%---------------------------------------------------
[X, U, Y] = simplot(ctrl, x0); 
close all
mbg_assertequal(size(X, 1), 13);
mbg_asserttolequal(X(end, :), [0 0], 1e-2);
mbg_assertequal(size(Y, 2), 2);    % ctrl.sysStruct has 2 outputs
mbg_assertequal(size(U, 1), 13);   % U must have correct dimension
mbg_assertequal(size(Y, 1), 13);    % Y must have correct dimension


%---------------------------------------------------
[X, U, Y] = simplot(ctrl, x0, N);
close all
mbg_assertequal(size(X, 1), N+1);
mbg_assertequal(size(U, 1), N+1);    % U must have correct dimension
mbg_assertequal(size(Y, 1), N+1);    % Y must have correct dimension


%---------------------------------------------------
[X, U, Y] = simplot(ctrl, x0, Options);
close all
mbg_assertequal(size(X, 1), 7);
mbg_assertequal(size(U, 1), 7);    % U must have correct dimension
mbg_assertequal(size(Y, 1), 7);    % Y must have correct dimension



%---------------------------------------------------
X = simplot(ctrl, x0, N, Options);
close all
mbg_assertequal(size(X, 1), N+1);


%---------------------------------------------------
[X, U, Y] = simplot(ctrl, simSysStruct, x0);
close all
mbg_assertequal(size(X, 1), 11);
mbg_asserttolequal(X(end, :), [0 0], 1e-2);
mbg_assertequal(size(U, 1), 11);    % U must have correct dimension
mbg_assertequal(size(Y, 1), 11);    % Y must have correct dimension


%---------------------------------------------------
[X, U, Y] = simplot(ctrl, simSysStruct, x0, N);
close all
mbg_assertequal(size(X, 1), N+1);
mbg_assertequal(size(U, 1), N+1);    % U must have correct dimension
mbg_assertequal(size(Y, 1), N+1);    % Y must have correct dimension


%---------------------------------------------------
X = simplot(ctrl, simSysStruct, x0, Options);
close all
mbg_assertequal(size(X, 1), 6);


%---------------------------------------------------
[X, U, Y] = simplot(ctrl, simHandle, x0);
close all
mbg_assertequal(size(X, 1), 13);
mbg_assertequal(size(Y, 2), 1);    % di_sim_fun returns only one output
mbg_asserttolequal(X(end, :), [0 0], 1e-2);
mbg_assertequal(size(U, 1), 13);   % U must have correct dimension


%---------------------------------------------------
[X, U, Y] = simplot(ctrl, simHandle, x0, N);
close all
mbg_assertequal(size(X, 1), N+1);
mbg_assertequal(size(Y, 2), 1);    % di_sim_fun returns only one output
mbg_assertequal(size(U, 1), N+1);    % U must have correct dimension


%---------------------------------------------------
[X, U, Y] = simplot(ctrl, simHandle, x0, Options);
close all
mbg_assertequal(size(X, 1), 7);
mbg_assertequal(size(Y, 2), 1);    % di_sim_fun returns only one output
mbg_assertequal(size(U, 1), 7);    % U must have correct dimension
