function eq_test1

p1 = polytope([0 0; 1 0; 1 1; 0 1]);
p2 = polytope([0 0; -1 0; -1 1; 0 1]);

% p1 and p2 are not equal
e = (p1==p2);
mbg_assertfalse(e);

% p1 and p1 are equal
e = (p1==p1);
mbg_asserttrue(e);

% [p1 p2] and p1 are not equal
e = ([p1 p2]==p1);
mbg_assertfalse(e);

% [p1 p2] and [p2 p1] are equal
e = ([p1 p2]==[p2 p1]);
mbg_asserttrue(e);
