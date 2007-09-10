function mplp_infeasible1

% patch in changeset http://control.ee.ethz.ch/~mpt/hg/mpt-26x?cs=38a9458905cb
% fails if the mplp is infeasible (due to a missing details.Bi field)

% an infeasible mplp:
M.G = [1; -1; 0; 0];
M.E = [0; 0; -1; 1];
M.W = [0; 0; -1; -1];
M.H = 1;
M.F = 1;

% check that the problem is really infeasible
P = polytope([-M.E M.G], M.W);
mbg_assertfalse(isfulldim(P));

% solve the mplp
[Pn, Fi, Gi, AC, PH, details] = mpt_mplp(M);

% verify that results indicate an infeasible solution
mbg_assertfalse(isfulldim(Pn));
mbg_assertfalse(isfulldim(PH));
mbg_asserttrue(isempty(Fi));
mbg_asserttrue(isempty(details.Bi));
mbg_assertfalse(details.feasible);
