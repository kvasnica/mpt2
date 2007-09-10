function projection_test1

return

% if projection in the projected space is R^n, we should return it as R^n
% instead of breaking with error:

P = polytope([0 0 0 1; 0 0 0 -1], [1;1]);

% projection of P in x1 and x2 is just R^n
Q = projection(P, 1:2);

[x, r] = chebyball(Q),

% chebycenter of R^n is Inf
mbg_assertequal(r, Inf);


P = polytope([-1 -1; 1 -1], [0;0]);
