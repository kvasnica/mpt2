function sim_test2

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

%fprintf('mpt_getInput should complain about wrong dimension of x0\n');
lasterr('');
try
    % mpt_getInput should complain
    sim(ctrl, sysStruct, [1]);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Wrong dimension of x0.');


%fprintf('\nsub_computetrajectory should complain about incompatible dimensions\n');
lasterr('');
try
    % sub_computetrajectory should complain
    sim(ctrl, sysStruct, [1; 1]);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'The provided system structure has incompatible number of states and/or inputs.');
