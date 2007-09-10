function simplot_test2

% tests that we properly catch cases where the user-provided sysStruct has
% incompatible dimensions

load ctrlN2

sysStruct.A = 1;
sysStruct.B = 1;
sysStruct.C = 1;
sysStruct.D = 0;
sysStruct.umax = 1;
sysStruct.umin = -1;
sysStruct.ymax = 1;
sysStruct.ymin = -1;

%fprintf('mpt_plotTimeTrajectory should complain about wrong dimension of x0\n');
lasterr('');
try
    % mpt_getInput should complain
    simplot(ctrl, sysStruct, [1]);
    worked = 1;
catch
    worked = 0;
end
close all
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'mpt_plotTimeTrajectory: x0 must be a 2 x 1 vector!');


%fprintf('\nsub_computetrajectory should complain about incompatible dimensions\n');
lasterr('');
try
    % sub_computetrajectory should complain
    simplot(ctrl, sysStruct, [1; 1]);
    worked = 1;
catch
    worked = 0;
end
close all
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'The provided system structure has incompatible number of states and/or inputs.');
