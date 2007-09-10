function mpt_mpqp_test3

% tests that we properly handle cases where the [G W] matrix contains multiple
% identical rows (if so, we have a problem when doing matrix inverse). fixed by:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=47dc10d52961

load mpqp_redundant_prob
% compute feasible set by projection
Pfeas = polytope([-Matrices.E Matrices.G], Matrices.W);
Pproj = projection(Pfeas, 1:size(Matrices.E, 2));

fprintf('Degeneracy should be indicated:\n\n');
Pn=mpt_mpqp(Matrices);
mbg_assertequal(length(Pn), 1);

mbg_asserttrue(Pn==Pproj); % test that mpqp returned correct solution
