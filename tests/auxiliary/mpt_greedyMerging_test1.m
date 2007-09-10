function mpt_greedyMerging_test1

% tests merge for 1-D polytopes
result = polytope([-1; 1]);
P1 = polytope([-1; 0]);
P2 = polytope([0; 1]);
P = [P1 P2];
Q = mpt_greedyMerging(P);
R = merge(P);
mbg_asserttrue(Q==result);
mbg_asserttrue(R==result);
mbg_assertequal(length(Q), 1);
mbg_assertequal(length(R), 1);
