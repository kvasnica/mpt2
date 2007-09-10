function uminus_test1

% tests that we properly update the bounding box (Bug 64)

p1 = polytope([-2; 1]);
q = -p1;
ps = struct(p1); qs = struct(q);
mbg_asserttolequal(ps.bbox, [-2 1]);
mbg_asserttolequal(qs.bbox, [-1 2]);

p2 = polytope([-3; 4]);
p = [p1 p2];
q = -p;
ps1 = struct(p(1)); qs1 = struct(q(1));
mbg_asserttolequal(ps1.bbox, [-2 1]);
mbg_asserttolequal(qs1.bbox, [-1 2]);
ps2 = struct(p(2)); qs2 = struct(q(2));
mbg_asserttolequal(ps2.bbox, [-3 4]);
mbg_asserttolequal(qs2.bbox, [-4 3]);

p = polytope([-1 -1; -1 2; 1 3; 1 -2]);
q = -p;
ps = struct(p); qs = struct(q);
mbg_asserttolequal(ps.bbox, [-1 1; -2 3]);
mbg_asserttolequal(qs.bbox, [-1 1; -3 2]);
