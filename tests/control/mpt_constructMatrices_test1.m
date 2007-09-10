function mpt_constructMatrices_test1

% tests whether we properly set Matrices.constraints_reduced:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=d2d3e63d4d5d

lti1d
probStruct.norm = 2; probStruct.N = 1;

% this should automatically kick out redundant constraints and tell us so
M = mpt_constructMatrices(sysStruct, probStruct);
mbg_assertequal(M.constraints_reduced, 1);

% this should NOT remove redundant constraints:
M = mpt_constructMatrices(sysStruct, probStruct, struct('noConstraintReduction', 1));
mbg_assertequal(M.constraints_reduced, 0);
