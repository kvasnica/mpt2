function mpt_solveMIQP_test2

% tests correctness of the branch&bound method. in particular, tests that we
% properly divide H by two inside of mpt_solveMIQP, otherwise we get wrong
% results. related to:
%   http://control.ee.ethz.ch/~mpt/hg/mpt?cs=9a7778fe232c
% and
%   http://control.ee.ethz.ch/~mpt/hg/mpt?cs=42fe8446872e

p=unitbox(2)+[2;2];
[A,B]=double(p);
H=eye(2); f=-3*ones(1,2); vartype='CB';
[xopt, fval, how, ef] = mpt_solveMIQP(H, f, A, B, [], [], [], [], vartype, [], [], 1);

% correct values as computed by cplex:
% fval = -7
% xopt = [3; 1]

mbg_asserttolequal(fval, -7, 1e-6);
mbg_asserttolequal(xopt, [3; 1], 1e-3);
