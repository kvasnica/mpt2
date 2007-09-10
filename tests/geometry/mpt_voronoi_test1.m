function mpt_voronoi_test1

% tests if we re-order partition correctly:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=8c2c78063c1b

V = [4 4; 3 3; 2 2; 1 1];

% this should automatically sort regions
P = mpt_voronoi(V);
A = [];
for ii = 1:size(V, 1),
    % test that point ii belongs to polytope ii
    A = [A; isinside(P(ii), V(ii, :)')];
end
mbg_asserttrue(all(A==1));


% this should switch off the ordering
P = mpt_voronoi(V, struct('sortcells', 0));
A = [];
for ii = 1:size(V, 1),
    A = [A; isinside(P(ii), V(ii, :)')];
end
% this is the expected results for this example
mbg_assertequal(A, [0;0;0;0]);


% test situation when we have multiple same sites
V = [4 4; 3 3; 4 4; 4 4];

% without sorting, P just consist of 2 polytopes
P = mpt_voronoi(V, struct('sortcells', 0));
mbg_assertequal(length(P), 2);

% with sorting, P has 4 regions
P = mpt_voronoi(V);
mbg_assertequal(length(P), 4);
