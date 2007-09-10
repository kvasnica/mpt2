function facetcircle_test3

% test if facetcircle gives correct answer on 1D polytopes. remember that facets
% of 1D polytopes are just points, therefore their radius is zero

P = unitbox(1,1);
[x,r] = facetcircle(P, [1 2]);

mbg_asserttolequal(r, [0 0]);
mbg_asserttolequal(x, [1 -1]);
