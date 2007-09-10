function subsref_test1

% test subindexing of polytopes / polytope arrays
p1 = unitbox(1, 1);
p2 = unitbox(1, 2);
p3 = unitbox(1, 3);
p4 = unitbox(1, 4);
P = [p1 p2 p3 p4];

% indexing of a single polytope should return the same polytope
q = p1(1);
mbg_asserttrue(q==p1);

% indexing by a scalar
q = P(1);
mbg_asserttrue(q==p1);
q = P(3);
mbg_asserttrue(q==p3);

% indexing by a range
q = P(1:2);
mbg_asserttrue(q(1)==p1);
mbg_asserttrue(q(2)==p2);

q = P([4 1]);
mbg_asserttrue(q(1)==p4);
mbg_asserttrue(q(2)==p1);

% indexing by an empty matrix should return an empty polytope
q = P([]);
mbg_assertequal(length(q), 0);
mbg_assertfalse(isfulldim(q));
