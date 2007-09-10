function mpt_maxCtrlSet_test2

% tests that we cope with deltaU constraints correctly:
% we should augment sysStruct to incorporate deltaU constraints automatically in
% mpt_maxCtrlSet, but then i decided we won't do that and leave it up to the
% user. the reason being that mpt_maxCtrlSet is called automatically from
% mpt_control and the augmentation is done there


lti1d
sysStruct.dumax = 2;
sysStruct.dumin = -2;

% augment the system to incorporate deltaU constraints
[sysStruct, probStruct] = mpt_prepareDU(sysStruct, probStruct);

% expected output is a polytope in 2D because we augment the system matrices to
% include u(k-1)
expected_output = polytope([0 -1; -1 0; 0 1; 1 0], [1;4;1;4]);

% Cinf is a polytope in 2D and should be computed in 2 iterations
[P, iter] = mpt_maxCtrlSet(sysStruct);
mbg_assertequal(dimension(P), 2);      % output must be a 2D polytope
mbg_asserttrue(P==expected_output);
mbg_assertequal(iter, 2);

% Kinf is a polytope in 2D and should be computed in 4 iterations
[P, iter] = mpt_maxCtrlSet(sysStruct, struct('Kinf', 1));
mbg_assertequal(dimension(P), 2);      % output must be a 2D polytope
mbg_asserttrue(P==expected_output);
mbg_assertequal(iter, 4);
