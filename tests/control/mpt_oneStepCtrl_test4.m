function mpt_oneStepCtrl_test4

% tests whether mpt_maxCtrlSet() works with Options.Vconverge>0 and
% Options.Kinf=0:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=c93bd71dd917

lti1d
% set up one-step problem
probStruct.subopt_lev = 2;
probStruct.N = 1;
Options.Vconverge = 30;

ctrl = mpt_control(sysStruct, probStruct, Options); % Options.Kinf=1 should be set automatically
mbg_assertequal(length(ctrl.Pn), 3);
