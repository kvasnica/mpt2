function pelemfun_test1

% tests the polytope/pelemfun() function:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=7f12da44c432

p1 = unitbox(2,1);
p2 = unitbox(2,1)+[1;1];
P = [p1 p2];

% vertices of p1 and p2
v1 = [1 -1; 1 1; -1 1; -1 -1];
v2 = [2 0; 0 0; 0 2; 2 2];

% chebyball centers and radii for p1 and p2
x1 = [0; 0]; r1 = 1;
x2 = [1; 1]; r2 = 1;

E = pelemfun(@extreme, P);
mbg_asserttrue(iscell(E));    % E must be returned as a cell array
mbg_assertequal(length(E), 2); % E must have 2 elements (=length(P))
mbg_assertequal(E{1}, v1);    % extreme points must be correct
mbg_assertequal(E{2}, v2);

[X, R] = pelemfun(@chebyball, P);
mbg_asserttrue(iscell(X));    % X must be returned as a cell array
mbg_assertequal(length(X), 2); % X must have 2 elements (=length(P))
mbg_asserttrue(iscell(R));    % R must be returned as a cell array
mbg_assertequal(length(R), 2); % R must have 2 elements (=length(P))
mbg_assertequal(X{1}, x1);    % chebyball center must be correct
mbg_assertequal(X{2}, x2);    % chebyball center must be correct
mbg_assertequal(R{1}, r1);    % chebyball radius must be correct
mbg_assertequal(R{2}, r2);    % chebyball radius must be correct

Q = unitbox(2, 1);
I = pelemfun(@le, P, Q);
mbg_asserttrue(iscell(I));    % I must be returned as a cell array
mbg_assertequal(length(I), 2); % I must have 2 elements (=length(P))
mbg_assertequal(I{1}, 1);     % P(1) is a subset of Q
mbg_assertequal(I{2}, 0);     % P(1) is NOT a subset of Q

