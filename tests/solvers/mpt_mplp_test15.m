function mpt_mplp_test15
% SKIP

return

load Phard_unbounded
Pfeas = projection(polytope([-Matrices.E Matrices.G], Matrices.W), 1:2);
mbg_asserttrue(isbounded(Pfeas));

% with NAG as of March 23rd 2006 unbounded Phard and wrong Pn is returned. all
% works fine with CDD
[Pn, Fi, Gi, AC, Phard] = mpt_mplp(Matrices, Options);
mbg_asserttrue(isbounded(Phard));
mbg_asserttrue(Pn==Pfeas);
mbg_asserttrue(Phard==Pfeas);
