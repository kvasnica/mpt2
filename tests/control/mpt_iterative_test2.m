function mpt_iterative_test2

% test user-defined target sets for minimum-time control

lti1d
% set up minimum-time problem w/ 2-norm
probStruct.subopt_lev = 1;
probStruct.N = Inf;
probStruct.norm = 2;

% use custom target set
probStruct.Tset = unitbox(1, 0.5);

ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 10);

% test also linear norms
probStruct.norm = 1;
ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 14);
