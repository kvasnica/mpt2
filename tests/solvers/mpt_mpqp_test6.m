function mpt_mpqp_test6

% tests automatic symmetrization of the H terms

Double_Integrator
probStruct.Tconstraint = 0;
probStruct.P_N = probStruct.Q;
probStruct.N = 2;

M = mpt_constructMatrices(sysStruct, probStruct);
% make the H term non-symmetric
M.H(2, 1) = M.H(2, 1)+1;

lastwarn('');
Options.qpsolver = 1;
Pn = mpt_mpqp(M, Options);
% a fixed mpt_mpqp should not give any warnings about a non-symmetrical
% Hessian (with quadprog)
mbg_asserttrue(isempty(lastwarn));
mbg_assertequal(length(Pn), 7);
