function mpt_oneStepCtrl_test5

% tests Options.Pfinal (loopCtr was undefined):
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=1762ff59889e

lti1d

% set up the one-step problem
probStruct.N = 1;
probStruct.subopt_lev = 2;

% use custom Pfinal set
Options.Pfinal = unitbox(1,1);

% this must work
ctrl = mpt_control(sysStruct, probStruct, Options);
mbg_assertequal(length(ctrl), 3);
