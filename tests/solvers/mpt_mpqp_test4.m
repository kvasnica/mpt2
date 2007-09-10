function mpt_mpqp_test4

% tests that we properly deal with cases where Matrices.G has no columns (this
% happens when solving an mpMIQP for LTI systems with discrete inputs - we don't
% have any free optimization move since all inputs are fixed):
%   Double_Integrator
%   sysStruct.Uset = [-1 0 1];
%   ctrl = mpt_otpQuadCtrl(sysStruct, probStruct);
%
% patch:
%

load mpqp_Gempty Matrices

[Pn, Fi, Gi, AC, Ph, D] = mpt_mpqp(Matrices);

% Pn should be the polytope (-Matrices.E*x <= Matrices.W)
Pf = polytope(-Matrices.E, Matrices.W);
mbg_asserttrue(Pn==Pf);

% in case like this we return Fi and Gi as empty matrices
mbg_asserttrue(isempty(Fi{1}));
mbg_asserttrue(isempty(Gi{1}));

% in case like this we return cost corrsponding to Y, Cx and Cc:
mbg_assertequal(D.Ai{1}, Matrices.Y);
mbg_assertequal(D.Bi{1}, Matrices.Cx);
mbg_assertequal(D.Ci{1}, Matrices.Cc);

% Phard should be equal to Pn
mbg_asserttrue(Pn==Ph);
% and it should be a single polytope
mbg_assertequal(length(Ph), 1);
mbg_assertequal(length(Pn), 1);
