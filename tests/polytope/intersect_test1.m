function intersect_test1

% test the new intersect(Pn) syntax:
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x/rev/a0af039865b8

a = intersect(polytope);
mbg_assertfalse(isfulldim(a));
mbg_assertequal(length(a), 0);

a = intersect(unitbox(2));
mbg_asserttrue(a == unitbox(2));
mbg_assertequal(length(a), 1);

a = intersect([unitbox(2) unitbox(2)]);
mbg_asserttrue(a == unitbox(2));
mbg_assertequal(length(a), 1);

p = [unitbox(2), unitbox(2,0.5), unitbox(2,0.5)+[0.2; 0]];
a = intersect(p);
mbg_asserttrue(a == polytope([-0.3 0.5; 0.5 0.5; 0.5 -0.5; -.3 -.5]));
mbg_assertequal(length(a), 1);

p = [unitbox(2), unitbox(2,0.5), unitbox(2,0.5)+[1; 0]];
a = intersect(p);
mbg_assertfalse(isfulldim(a));
mbg_assertequal(length(a), 0);

