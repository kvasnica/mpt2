function mpt_mplp_test8

% this one gives following warning with NAG:
%
% FACETCIRCLE: All available LP solvers failed:
% FACETCIRCLE:   ERROR: Numerical problems with LP solver (radius is negative)
% Warning: Region 27, facet 5: cannot find point on the facet.
%
% it is probably due to the fact that the problem is unbounded

load mplp_facetcircle_prob

[Pn, Fi, Gi, AC, Phard] = mpt_mplp(Matrices);

lp = mpt_options('lpsolver');
if lp==3,
    % 78 regions with CDD (as of 19-Nov-2005)
    mbg_assertequal(length(Pn), 78);
else
    % 77 regions with NAG (as of 19-Nov-2005)    
    mbg_assertequal(length(Pn), 77);
end
