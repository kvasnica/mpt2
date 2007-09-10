function mpt_maxCtrlSet_test4

% tests the "volume convergence" option:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=80e99ef5e36f

lti1d

% standard computation has Options.Vconverge disabled and requires 4 iterations:
[p,q,r]=mpt_maxCtrlSet(sysStruct, struct('Kinf', 1, 'verbose', 0));
mbg_assertequal(q, 4);
mbg_asserttrue(p==unitbox(1,4));  % also test that the Kinf set is correct

% the 4 iterations have volume increase as follows:
% 1. 61.8 %
% 2. 38.2 %
% 3. 10.6 %
% 4. 0 %
%
% therefore we set Options.Vconverge=20 which should cause the algorithm to
% break at iteration 3:
[p,q,r]=mpt_maxCtrlSet(sysStruct, struct('Kinf', 1, 'verbose', 0, 'Vconverge', 20));
mbg_assertequal(q, 3);
