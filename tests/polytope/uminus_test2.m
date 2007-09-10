function uminus_test2

% tests that we properly update the extreme points

V1 = [-1 -1; -1 3; 2 2; 3 -1];
V2 = [0 2; 0 -1; 2 2; 2 -4];
p1 = polytope(V1);
p2 = polytope(V2);

q = -p1;
ps = struct(p1); qs = struct(q);
mbg_asserttolequal(ps.vertices, V1);
mbg_asserttolequal(sortrows(qs.vertices), sortrows([-V1(:, 2) -V1(:, 1)]));

q = -[p1 p2];
qs = struct(q(1));
mbg_asserttolequal(sortrows(qs.vertices), sortrows([-V1(:, 2) -V1(:, 1)]));
qs = struct(q(2));
mbg_asserttolequal(sortrows(qs.vertices), sortrows([-2 -2; 0 -2; 0 1; -2 4]));
