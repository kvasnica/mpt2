function getOutterEllipsoid_test1

% tests that we correctly return ELLIPSOID objects
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=29578a3378d9

p = polytope([eye(3); -eye(3)], [1;2;3;3;2;1]);
E = getOutterEllipsoid(p);
mbg_asserttrue(isa(E, 'ellipsoid'));

% test that the ellipsoid is correct
[x, e] = parameters(E);
egood = [12.0001880260842 1.81605701233524e-015 1.0118765183606e-015;1.81605701233524e-015 12.0001880260842 -5.74384512851103e-016;1.0118765183606e-015 -5.74384512851103e-016 11.9996239751792];
mbg_asserttolequal(x, [-1; 0; 1], 1e-6);
mbg_asserttolequal(e, egood, 1e-6);
