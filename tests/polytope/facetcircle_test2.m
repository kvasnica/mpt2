function facetcircle_test1

% check if facetcircle works for vector of facets
p = unitbox(2,1);

[x,r] = facetcircle(p, [1 2]);
mbg_assertequal(x, eye(2));
mbg_assertequal(r, [1 1]);
