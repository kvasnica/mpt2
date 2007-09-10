function isinside_test1

% tests cases where x0 is a logical

p1=unitbox(1); p2=unitbox(1)+[-2];

a = isinside(p1, logical(1));
mbg_asserttrue(a);

a = isinside(p1, logical(0));
mbg_asserttrue(a);

a = isinside(p2, logical(1));
mbg_assertfalse(a);

a = isinside(p2, logical(0));
mbg_assertfalse(a);
