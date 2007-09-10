function end_test1

p = unitbox(2, 1);
q = unitbox(2, 2);
P = [p q];

mbg_asserttrue(P(end)==q);

% p(end) should be the same polytope again, if "p" is a single polytope
a = p(end);
mbg_asserttrue(a==unitbox(2,1));
