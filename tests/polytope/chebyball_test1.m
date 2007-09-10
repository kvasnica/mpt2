function chebyball_test1


P=unitbox(2,1);
[x,r] = chebyball(P);

mbg_assertequal(x, [0; 0]);
mbg_assertequal(r, 1);
