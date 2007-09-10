function mpt_mpqp_test1

% test example send by Johan Loefberg on 21-Nov-2005
% his Matrices contain redundand constraints, which lead to crash due to wrong
% active constraints:
% Warning: Matrix is singular to working precision.
% (Type "warning off MATLAB:singularMatrix" to suppress this warning.)
% > In c:\matlabfiles\mpt\mpt\solvers\mpt_mpqp.m (sub1_computelaw) at line 742
%   In c:\matlabfiles\mpt\mpt\solvers\mpt_mpqp.m at line 450
% ??? Error using ==> polytope/normalize
% NORMALIZE: No Inf terms are allowed in the matrix P.H
%
% fixed by:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=d2d3e63d4d5d

load mpqpbug1

% Matrices.G, Matrices.W, Matrices.E contain redundant entries, but we should
% automatically remove them
disp('this should give 13 regions');
Pn = mpt_mpqp(Matrices);
mbg_assertequal(length(Pn), 13);

% do not remove redundant constraints but pretend we have done so. this used to
% lead to an error, but was fixed by:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=47dc10d52961
disp('this should give a bunch of "degeneracy" warnings:');
M = Matrices;
M.constraints_reduced = 1;
Pn = mpt_mpqp(M);
mbg_assertequal(length(Pn), 13);


% remove redundant constraints by hand, but mpt_mpqp() will do it again because
% Matrices.constaints_reduced is not set
disp('this should give 13 regions');
P = polytope([Matrices.G Matrices.E], Matrices.W);
[H, K] = double(P);
Matrices.G = H(:, 1:size(Matrices.G, 2));
Matrices.E = H(:, size(Matrices.G, 2)+1:end);
Matrices.W = K;
Pn = mpt_mpqp(Matrices);
mbg_assertequal(length(Pn), 13);

% remove redundant constraints by hand, tell mpt_mpqp() we have done so
disp('this should give 13 regions');
Matrices.constraints_reduced = 1;
Pn = mpt_mpqp(Matrices);
mbg_assertequal(length(Pn), 13);
