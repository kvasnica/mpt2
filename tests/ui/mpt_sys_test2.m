function mpt_sys_test2

% test HYSDEL conversion via mpt_sys
sst = mpt_sys('turbo_car');

% simulator should have been generated:
mbg_asserttrue(~isempty(sst.data.SIM.code));

% check whether we generate a valid system structure:
sst.ymax = 10;
sst.ymin = -10;
mpt_verifySysStruct(sst);

% check that the PWA system has 2 PWA dynamics
mbg_asserttrue(iscell(sst.A));
mbg_assertequal(length(sst.A), 2);

% check whether we have state and input constraints
mbg_asserttrue(isfield(sst, 'xmax'));
mbg_asserttrue(isfield(sst, 'xmin'));
mbg_asserttrue(isfield(sst, 'umax'));
mbg_asserttrue(isfield(sst, 'umin'));

% check whether state constraints are correct
mbg_assertequal(sst.xmax, [50; 10; 5]);
mbg_assertequal(sst.xmin, [-50; -10; 0]);
mbg_assertequal(sst.umax, [1; 1]);
mbg_assertequal(sst.umin, [-1; 0]);

% check whether sysStruct.Uset has been set correctly
mbg_asserttrue(isfield(sst, 'Uset'));
mbg_asserttrue(iscell(sst.Uset));
mbg_assertequal(sst.Uset{1}, [-Inf Inf]);
mbg_assertequal(sst.Uset{2}, [0 1]);
