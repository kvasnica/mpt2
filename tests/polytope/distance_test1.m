function distance_test1

% tests distance of two polytopes

p = unitbox(2,1);
% two intersecting polytopes, distance is zero
q = unitbox(2,1) + [1;0];
a = distance(p, q);
mbg_assertequal(a, 0);

% two touching polytopes, distance is zero
q = unitbox(2,1) + [2;0];
a = distance(p, q);
mbg_asserttolequal(a, 0, 1e-8);

% two non-intersecting polytopes
q = unitbox(2,1) + [3;0];
a = distance(p, q);
mbg_assertequal(a, 1);

% now test different norms
for n = [1 Inf 2],
    Options.norm = n;
    % two intersecting polytopes, distance is zero
    q = unitbox(2,1) + [1;0];
    a = distance(p, q, Options);
    mbg_assertequal(a, 0);
    
    % two touching polytopes, distance is zero
    q = unitbox(2,1) + [2;0];
    a = distance(p, q, Options);
    mbg_asserttolequal(a, 0, 1e-8);
    
    % two non-intersecting polytopes
    q = unitbox(2,1) + [3;0];
    a = distance(p, q, Options);
    mbg_assertequal(a, 1);
end

% distance of two polyarrays
p1 = polytope([2 0; 3 0; 1 -2; 3 -2]);
p2 = polytope([2 0; 3 0; 1 2; 3 2]);
P = [p1 p2];
Q = -P;
a = distance(P, Q);
mbg_asserttolequal(a, 2);

% polytope+polyarray
a = distance(p, P);
mbg_asserttolequal(a, 0.44721359549996, 1e-8);
