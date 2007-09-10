function reduce_test2

% test various trivially empty polytopes

% x1 <= -1, x1 >= 1 is an empty polytope
P = polytope([1; -1],[-1;-1]);
mbg_asserttrue(~isfulldim(P));

% x1 <= 0, x1 >= 0, x2 <= 0, x2 >=0 is an empty polytope
P = polytope([eye(2);-eye(2)],[-1;-1;-1;-1]);
mbg_asserttrue(~isfulldim(P));
