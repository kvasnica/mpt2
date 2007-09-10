function mpt_getInputXU_test2

% test that we don't break if x0 is outside of feasible set:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=5980185d72a1

load ctrlXU_1d
x0 = 5;  % state outside of feasible set

%this one should not break:
U = mpt_getInput(ctrlXU, x0, struct('useXU', 1));
