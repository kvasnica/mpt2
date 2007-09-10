function mpt_mplp_test10

% more examples which cycle. fixed by:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=7d31cfddd4cb

load mplp_cycling6
Pn = mpt_mplp(Matrices, struct('max_regions', 50));

load mplp_cycling7
Pn = mpt_mplp(Matrices, struct('max_regions', 50));

load mplp_cycling9
[Pn, Fi, Gi, AC, Ph] = mpt_mplp(Matrices, struct('max_regions', 50));
%mbg_asserttrue(Pn==Ph);

% % 12D example
% load mplp_cycling8
% Pn = mpt_mplp(Matrices, struct('max_regions', 20));
