function nconstr_test1

% tests whether we handle polytope arrays correctly:

P1 = unitbox(2, 1);

nc = nconstr(P1);
mbg_assertequal(nc, 4);

nc = nconstr([P1 P1]);
mbg_assertequal(nc, 8);
