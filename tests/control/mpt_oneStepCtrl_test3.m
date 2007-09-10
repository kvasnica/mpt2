function mpt_oneStepCtrl_test3

% tests whether we can use Options.Kinf together with mpt_oneStepCtrl:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=aba12e95ef07

lti1d

% set up 1-step problem
probStruct.N = 1;
probStruct.subopt_lev = 2;

% solve the 1-step problem
ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(ctrl.details.loopCtr, 2); % 2 iterationes needed to get the Cinf set

% solve the 1-step problem with Kinf sets
ctrl = mpt_control(sysStruct, probStruct, struct('Kinf', 1));
mbg_assertequal(ctrl.details.loopCtr, 4); % 4 iterationes needed to get the Cinf set
