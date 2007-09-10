function mpt_mplp_test7

% tests that we prevent cycling in mpt_mplp_ver7:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=7d31cfddd4cb

% this example used to cycle (i.e. never finish) even with the following patch
% applied:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=3583353cd0a2

load mplp_cycling4
Pn=mpt_mplp(Matrices, struct('max_regions', 50));
mbg_assertequal(length(Pn), 2);

% compute feasible set by projection
P = polytope([-Matrices.E Matrices.G], Matrices.W);
Pfeas = projection(P, 1:3, struct('projection', 5));

% solution must consist of 2 regions
mbg_assertequal(length(Pn), 2);

% compare Pn to the feasible set
mbg_asserttrue(Pn==Pfeas);
