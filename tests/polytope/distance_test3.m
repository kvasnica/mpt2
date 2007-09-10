function distance_test3

% tests distance of point+polytope

p = unitbox(2,1);
% point+polytope, intersecting, distance is zero
q = [0.5; 0.5];
a = distance(q, p);
mbg_assertequal(a, 0);

% point+polytope, point on a vertex, distance is zero
q = [1; 1];
a = distance(q, p);
mbg_assertequal(a, 0);

% point+polytope, point on a facet, distance is zero
q = [1; 0];
a = distance(q, p);
mbg_assertequal(a, 0);

% point+polytope, distinct
q = [2; 0];
a = distance(q, p);
mbg_assertequal(a, 1);

% now test different norms
for n = [1 Inf 2],
    Options.norm = n;
    % point+polytope, intersecting, distance is zero
    q = [0.5; 0.5];
    a = distance(q, p, Options);
    mbg_assertequal(a, 0);
    
    % point+polytope, point on a vertex, distance is zero
    q = [1; 1];
    a = distance(q, p, Options);
    mbg_assertequal(a, 0);
    
    % point+polytope, point on a facet, distance is zero
    q = [1; 0];
    a = distance(q, p, Options);
    mbg_assertequal(a, 0);
    
    % point+polytope, distinct
    q = [2; 0];
    a = distance(q, p, Options);
    mbg_assertequal(a, 1);
end

% distance point / polyarray
p1 = polytope([2 0; 3 0; 1 -2; 3 -2]);
p2 = polytope([2 0; 3 0; 1 2; 3 2]);
P = [p1 p2];
a = distance([1;1], P);
mbg_asserttolequal(a, 0.44721359549996, 1e-8);
