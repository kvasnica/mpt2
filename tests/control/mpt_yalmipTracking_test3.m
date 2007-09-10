function mpt_yalmipTracking_test2

% tests that we correctly augment probStruct.Q:
%

Double_Integrator

probStruct.norm = 2;
sub_testtracking(sysStruct, probStruct);

probStruct.norm = 1;
sub_testtracking(sysStruct, probStruct);

probStruct.norm = Inf;
sub_testtracking(sysStruct, probStruct);


%---------------------------------------------------------
function sub_testtracking(sysStruct, probStruct)

% tracking=1, state tracking
S = sysStruct;
P = probStruct;
P.tracking = 1;
[sst, pst] = mpt_yalmipTracking(S, P);
if probStruct.norm==2, nr = 5; else nr = 2; end
mbg_assertequal(size(pst.Q), [nr 5]);

% tracking=2, state tracking
S = sysStruct;
P = probStruct;
P.tracking = 2;
[sst, pst] = mpt_yalmipTracking(S, P);
if probStruct.norm==2, nr = 4; else nr = 2; end
mbg_assertequal(size(pst.Q), [nr 4]);

% tracking=1, output tracking (ny==nx)
S = sysStruct;
P = probStruct;
P.tracking=1;
P.Qy = eye(2);
[sst, pst] = mpt_yalmipTracking(S, P);
if probStruct.norm==2, nr = 5; else nr = 5; end
mbg_assertequal(size(pst.Q), [nr 5]);
if probStruct.norm==2, nr = 5; else nr = 2; end
mbg_assertequal(size(pst.Qy), [nr 5]);

% tracking=2, output tracking (ny==nx)
S = sysStruct;
P = probStruct;
P.tracking=2;
P.Qy = eye(2);
[sst, pst] = mpt_yalmipTracking(S, P);
if probStruct.norm==2, nr = 4; else nr = 4; end
mbg_assertequal(size(pst.Q), [nr 4]);
if probStruct.norm==2, nr = 4; else nr = 2; end
mbg_assertequal(size(pst.Qy), [nr 4]);

% tracking=1, output tracking (ny!=nx)
S = sysStruct;
S.C = [1 0]; S.D = 0; S.ymax = 1; S.ymin = -1;
P = probStruct;
P.tracking=1;
P.Qy = 10;
[sst, pst] = mpt_yalmipTracking(S, P);
if probStruct.norm==2, nr = 4; else nr = 4; end
mbg_assertequal(size(pst.Q), [nr 4]);
if probStruct.norm==2, nr = 3; else nr = 1; end
mbg_assertequal(size(pst.Qy), [nr 3]);

% tracking=2, output tracking (ny!=nx)
S = sysStruct;
P = probStruct;
P.tracking=2;
P.Qy = 10;
S.C = [1 0]; S.D = 0; S.ymax = 1; S.ymin = -1;
[sst, pst] = mpt_yalmipTracking(S, P);
if probStruct.norm==2, nr = 3; else nr = 3; end
mbg_assertequal(size(pst.Q), [nr 3]);
if probStruct.norm==2, nr = 2; else nr = 1; end
mbg_assertequal(size(pst.Qy), [nr 2]);
