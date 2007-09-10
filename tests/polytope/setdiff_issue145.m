function issue145_setdiff

% A is an empty polytope. B is an array of 2 polytopes. Default mptOptions 
% are used with NAG as lp and qp solver.
% 
% A\B returns an absurd polytope array of 36 polytopes instead of an empty 
% polytope.

% http://control.ee.ethz.ch/~mpt/hg/mpt?cmd=changeset;node=b244e4703f3334dbeb141eafe141c62201d7e57a

load issue145

p = A\B;

mbg_assertequal(isfulldim(p), 0);
