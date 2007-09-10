function mpt_mplp_test9

% here are couple of problems which used to cycle with ZERO_TOL=1e-12.
% especially M{1} and M{5} do cycle, the others do not.
%
% fixed by:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=7d31cfddd4cb

load mplp_cycling5

for ii = 1:length(M),
    if ii==2,
        % this case is handled in mpt_mplp_test11.m
        continue
    end
    Pn = mpt_mplp(M{ii}, struct('max_regions', 20));
    Pfeas = projection(polytope([-M{ii}.E M{ii}.G], M{ii}.W), 1:size(M{ii}.E, 2));
    mbg_asserttrue(Pn==Pfeas);
    mbg_asserttrue(isbounded(Pn));
end
