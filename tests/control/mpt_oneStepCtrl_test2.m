function mpt_oneStepCtrl_test2

% tests whether we properly return Piter from mpt_maxCtrlSet and
% mpt_oneStepCtrl:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=8e3aaba81160

lti1d
probStruct.N = 1;
sysStruct.ymax = 2;
sysStruct.ymin = -2;

[ctrlStruct,feasibleN,loopCtr,Piter] = mpt_oneStepCtrl(sysStruct,probStruct);
mbg_asserttrue(isa(Piter, 'polytope'));
mbg_assertequal(length(Piter), 2);
mbg_assertequal(loopCtr, 2);

[invSet,iterations, Piter] = mpt_maxCtrlSet(sysStruct, struct('Kinf', 1));
mbg_asserttrue(isa(Piter, 'polytope'));
mbg_assertequal(length(Piter), 3);  % we have 3 polytopes because initial LQR set is also included
mbg_assertequal(iterations, 2);
