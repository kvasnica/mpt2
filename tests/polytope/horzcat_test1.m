function horzcat_test1

% tests concatenation of polytopes
p = polytope;
p1 = unitbox(1, 1);
p2 = unitbox(1, 2);
p3 = unitbox(1, 3);
p4 = unitbox(1, 4);

% concatenation of a single polytope
P = [p1];
mbg_assertequal(length(P), 1);
ps = struct(P);
mbg_assertequal(ps.H, [1; -1]);
mbg_assertequal(ps.K, [1; 1]);

% concatenation of one single empty polytope
P = [p];
mbg_assertequal(length(P), 0);
mbg_assertfalse(isfulldim(P));

% concatenation of one non-empty polytope with an empty polytope, the empty one
% should not be added!
P = [p p1];
mbg_assertequal(length(P), 1);
ps = struct(P);
mbg_assertequal(ps.H, [1; -1]);
mbg_assertequal(ps.K, [1; 1]);

% concatenation of one non-empty polytope with an empty polytope, the empty one
% should not be added!
P = [p1 p];
mbg_assertequal(length(P), 1);
ps = struct(P);
mbg_assertequal(ps.H, [1; -1]);
mbg_assertequal(ps.K, [1; 1]);

% concatenation of two polytopes
P = [p1 p2];
mbg_assertequal(length(P), 2);
[x,r] = chebyball(P);
mbg_assertequal(r, [1; 2]);

% concatenation of three polytopes
Q = [p4 p3 p1];
mbg_assertequal(length(Q), 3);
[x,r] = chebyball(Q);
mbg_assertequal(r, [4; 3; 1]);

% concatenation of two polyarrays
W = [P Q];
mbg_assertequal(length(W), 5);
[x,r] = chebyball(W);
mbg_assertequal(r, [1; 2; 4; 3; 1]);

% concatenation of polytopes and polyarrays
Z = [p2 W p1];
mbg_assertequal(length(Z), 7);
[x,r] = chebyball(Z);
mbg_assertequal(r, [2; 1; 2; 4; 3; 1; 1]);
