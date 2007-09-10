function mpt_mplp_test11

% on this example we get an error on linux, but everything works fine under
% windows.
%
% update 22-Nov-2005: it now also works on linux if ALPHA=1e-6 (does not work
% with ALPHA=1e-5!)
%
% cycling also prevented by
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=7d31cfddd4cb

load mplp_cycling5
[Pn, Fi, Gi, AC, Ph] = mpt_mplp(M{2}, struct('max_regions', 20));
Pfeas = projection(polytope([-M{2}.E M{2}.G], M{2}.W), 1:size(M{2}.E, 2));
mbg_asserttrue(isfulldim(Pn));
mbg_asserttrue(Ph==Pfeas);
mbg_asserttrue(Pn==Ph);
