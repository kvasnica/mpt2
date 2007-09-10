function mpt_isnoise_test1

P = unitbox(2,1);
V = eye(2);

% noise is non-empty
a = mpt_isnoise(P);
mbg_asserttrue(a);
a = mpt_isnoise(V);
mbg_asserttrue(a);

% noise is empty
a = mpt_isnoise(polytope);
mbg_assertfalse(a);
a = mpt_isnoise([]);
mbg_assertfalse(a);
