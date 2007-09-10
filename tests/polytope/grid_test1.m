function grid_test1

P = unitbox(2);
X = grid(P, 3);
mbg_assertequal(X, [-1 -1;-1 0;-1 1;0 -1;0 0;0 1;1 -1;1 0;1 1]);

P = [unitbox(2) unitbox(2)+[1; 1]];
X = grid(P, 3);
mbg_assertequal(X, [-1 -1;-1 0.5;0.5 -1;0.5 0.5;0.5 2;2 0.5;2 2]);
