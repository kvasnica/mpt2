function distribpoints_test1

% test polytope/distribpoints()

p = unitbox(2,1);
n = 4;

% distribute 4 points inside of the polytope
x = distribpoints(p, n);

% for the unitbox example 4 points with maximum distances between them are the 4
% corner points of the polytope
%mbg_assertequal(unique(x', 'rows'), unique([1 1 -1 -1; 1 -1 -1 1]', 'rows'));

[h,k] = double(p);
% all points must be inside of the polytope
mbg_asserttrue(all(all((h*x - repmat(k, 1, n)) <= 1e-8)));

% convex hull of such points must be a subset of original polytope
mbg_asserttrue(polytope(x') <= p);
