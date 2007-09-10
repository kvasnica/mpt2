function getInnerEllipsoid_test2

% tests that we support arbitrary dimensions:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=0143405a6e81

% test polytope/getInnerEllipsoid()
p = polytope([eye(3); -eye(3)], [1;2;3;3;2;1]);
E = getInnerEllipsoid(p);
mbg_asserttrue(isa(E, 'ellipsoid'));

% test mpt_getInnerEllipsoid()
[E, x] = mpt_getInnerEllipsoid(p);
mbg_assertequal(size(E, 1), 3);
mbg_assertequal(size(E, 2), 3);
mbg_assertequal(length(x), 3);
