function getInnerEllipsoid_test1

% tests that we correctly return ELLIPSOID objects
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=29578a3378d9

p = polytope([eye(3); -eye(3)], [1;2;3;3;2;1]);
E = getInnerEllipsoid(p);
mbg_asserttrue(isa(E, 'ellipsoid'));

% test that the ellipsoid is correct
[x, e] = parameters(E);
mbg_asserttolequal(x, [-1; 0; 1], 1e-6);
mbg_asserttolequal(e, [4 0 0; 0 4 0; 0 0 4], 1e-6);
