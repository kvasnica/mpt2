function or_test1

% union of a fully dimensional polytope and an empty polytope must be equal to
% the fully-dimensional one. this works fine when union() is called directly,
% but does not work correctly when the overloaded "|" operator is used (returns
% empty polytope)
%
% fixed by:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=1625dd1cf0d1

p1 = unitbox(2, 1);
p2 = polytope;

u = p1 | p2;
mbg_asserttrue(u==p1);

u = p2 | p1;
mbg_asserttrue(u==p1);

u = union([p1 p2]);
mbg_asserttrue(u==p1);
