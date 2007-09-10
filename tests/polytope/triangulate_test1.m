function triangulate_test1

% tests whether we properly update extreme points:
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x/rev/76db6ca93353

P = unitbox(2,1);
[hhPoly,adj,vvPoly,vol,Q]=triangulate(P);

mbg_asserttrue(isa(hhPoly, 'polytope'));   % 2 triangles
mbg_assertequal(length(hhPoly), 2);        % triangles are polytopes
mbg_asserttrue(hhPoly==P);                 % triangles cover P

% R13:
%   mbg_assertequal(adj, [3 1 4; 3 2 1]);
% R14:
%   mbg_assertequal(sort(adj, 2), [2 3 4; 1 2 4]);

mbg_asserttrue(isa(vvPoly, 'cell'));       % V-representation of extreme points
mbg_assertequal(length(vvPoly), 2);
% R13 and R14 give different results:
%mbg_assertequal(sort(vvPoly{1}, 1), [-1 -1; -1 1; 1 1]);
%mbg_assertequal(sort(vvPoly{2}, 1), [-1 -1; 1 -1; 1 1]);

mbg_assertequal(vol, 4);    % volume of P is 4

mbg_asserttrue(isa(Q, 'polytope'));   % Q is P with updated extreme points
mbg_asserttrue(P==Q);
V = get(Q, 'vertices');
mbg_assertequal(V, [1 -1; 1 1; -1 1; -1 -1]);   % vertices must be present
