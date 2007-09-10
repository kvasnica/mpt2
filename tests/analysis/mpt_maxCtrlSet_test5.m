function mpt_maxCtrlSet_test5

% tests whether mpt_maxCtrlSet() works with Options.Vconverge>0 and
% Options.Kinf=0:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=c93bd71dd917

lti1d
Options.Vconverge = 30;
[P, iter] = mpt_maxCtrlSet(sysStruct, Options); % Options.Kinf=1 should be set automatically
mbg_assertequal(iter, 3);  % 3 iterations should have been needed with Options.Vconverge=30
