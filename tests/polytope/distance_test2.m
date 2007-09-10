function distance_test2

% tests distance of polytope+point

p = unitbox(2,1);
% polytope+point, intersecting, distance is zero
q = [0.5; 0.5];
a = distance(p, q);
mbg_assertequal(a, 0);

% polytope+point, point on a vertex, distance is zero
q = [1; 1];
a = distance(p, q);
mbg_assertequal(a, 0);

% polytope+point, point on a facet, distance is zero
q = [1; 0];
a = distance(p, q);
mbg_assertequal(a, 0);

% polytope+point, distinct
q = [2; 0];
a = distance(p, q);
mbg_assertequal(a, 1);

% now test different norms
for n = [1 Inf 2],
    Options.norm = n;
    % polytope+point, intersecting, distance is zero
    q = [0.5; 0.5];
    a = distance(p, q, Options);
    mbg_assertequal(a, 0);
    
    % polytope+point, point on a vertex, distance is zero
    q = [1; 1];
    a = distance(p, q, Options);
    mbg_assertequal(a, 0);
    
    % polytope+point, point on a facet, distance is zero
    q = [1; 0];
    a = distance(p, q, Options);
    mbg_assertequal(a, 0);
    
    % polytope+point, distinct
    q = [2; 0];
    a = distance(p, q, Options);
    mbg_assertequal(a, 1);
end

% distance polyarray / point
p1 = polytope([2 0; 3 0; 1 -2; 3 -2]);
p2 = polytope([2 0; 3 0; 1 2; 3 2]);
P = [p1 p2];
a = distance(P, [1;1]);
mbg_asserttolequal(a, 0.44721359549996, 1e-8);
