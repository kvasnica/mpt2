function mpt_yalmipTracking_test2

% tests that we properly reject systems with boolean inputs for which either
% tracking and/or deltaU constraints/penalties are given
%

Double_Integrator

% no boolean inputs, tracking=1 should work
fprintf('no boolean inputs, tracking=1 should work\n');
S = sysStruct;
P = probStruct;
P.tracking = 1;
[sst, pst] = mpt_yalmipTracking(S, P);
mbg_asserttrue(isfield(pst, 'tracking_augmented'));
mbg_assertequal(size(sst.A, 1), 5);

% no boolean inputs, tracking=2 should work
fprintf('\nno boolean inputs, tracking=2 should work\n');
S = sysStruct;
P = probStruct;
P.tracking = 2;
[sst, pst] = mpt_yalmipTracking(S, P);
mbg_asserttrue(isfield(pst, 'tracking_augmented'));
mbg_assertequal(size(sst.A, 1), 4);

% no boolean inputs, tracking=1 with deltaU constraints should work
fprintf('\nno boolean inputs, tracking=1 w/ deltaU constraints should work\n');
S = sysStruct;
S.dumax = 3; S.dumin = -3;
P = probStruct;
P.tracking = 1;
[sst, pst] = mpt_yalmipTracking(S, P);
mbg_asserttrue(isfield(pst, 'tracking_augmented'));
mbg_assertequal(sst.umax, S.dumax);
mbg_assertequal(sst.umin, S.dumin);
mbg_assertequal(size(sst.A, 1), 5);

% no boolean inputs, tracking=2 with deltaU constraints should work, but should
% give a warning
fprintf('\nno boolean inputs, tracking=2 w/ deltaU constraint should give a warning\n');
S = sysStruct;
S.dumax = 3; S.dumin = -3;
P = probStruct;
P.tracking = 2;
[sst, pst] = mpt_yalmipTracking(S, P);
mbg_asserttrue(isfield(pst, 'tracking_augmented'));
mbg_assertequal(sst.umax, S.umax);
mbg_assertequal(sst.umin, S.umin);
mbg_assertequal(size(sst.A, 1), 4);

% boolean inputs, tracking=1 should fail
lasterr('');
fprintf('boolean inputs, tracking=1 should fail\n');
S = sysStruct;
S.Uset = [0 1];
P = probStruct;
P.tracking = 1;
try
    [sst, pst] = mpt_yalmipTracking(S, P);
    worked = 1;
catch
    worked = 0;
end
disp(lasterr);
mbg_assertfalse(worked);

% boolean inputs, tracking=2 should work
lasterr('');
fprintf('boolean inputs, tracking=2 should work but should give a warning\n');
S = sysStruct;
S.Uset = [0 1];
P = probStruct;
P.tracking = 2;
[sst, pst] = mpt_yalmipTracking(S, P);
mbg_assertequal(size(sst.A, 1), 4);

% boolean inputs, tracking=2 with deltaU constraints should fail
lasterr('');
fprintf('boolean inputs, tracking=2 with deltaU constraints should fail\n');
S = sysStruct;
S.Uset = [0 1];
S.dumax = 3; S.dumin = -3;
P = probStruct;
P.tracking = 2;
try
    [sst, pst] = mpt_yalmipTracking(S, P);
    worked = 1;
catch
    worked = 0;
end
disp(lasterr);
mbg_assertfalse(worked);
