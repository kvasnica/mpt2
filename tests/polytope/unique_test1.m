function unique_test1

% unique() on a single polytope is the polytope itself
Q = unitbox(2,1);
[P, ind] = unique(Q);
mbg_asserttrue(P == Q);
mbg_assertequal(ind, 1);

% unique() of the following polyarray consists of Q(2), Q(3) and Q(5)
Q = [unitbox(2,1) unitbox(2,3) unitbox(2,1) unitbox(2,2) unitbox(2,2)];
[P, ind] = unique(Q);
mbg_asserttrue(P == Q([2 3 5]));
mbg_assertequal(sort(ind), [2; 3; 5]);
