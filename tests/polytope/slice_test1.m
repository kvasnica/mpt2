function slice_test1

% tests whether we properly return indicies of regions whose cut is fully
% dimensional:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=c14b55e34403

P1 = unitbox(2, 1);
P2 = unitbox(2, 1) + [0.5; 0];
P3 = unitbox(2, 1) + [2; 1];
P = [P1 P2 P3];

% this cut should slice P1 and P2
[p, s] = slice(P, 1, 0.8);
mbg_assertequal(length(p), 2);
mbg_asserttrue(isequal(s, [1 2]));

% this cut should slice P2 and P3
[p, s] = slice(P, 1, 1.5);
mbg_assertequal(length(p), 2);
mbg_asserttrue(isequal(s, [2 3]));

% this cut should slice only P1
[p, s] = slice(P, 1, -1);
mbg_assertequal(length(p), 1);
mbg_asserttrue(isequal(s, 1));

% this cut should slice no polytope
[p, s] = slice(P, 1, -2);
mbg_assertfalse(isfulldim(p));
mbg_asserttrue(isempty(s));

% verify cutting of single polytopes:

% no slice
[p, s] = slice(P1, 1, -2);
mbg_assertfalse(isfulldim(p));
mbg_asserttrue(isempty(s));

% fully-dimensional slice
[p, s] = slice(P1, 1, 0);
mbg_asserttrue(isfulldim(p));
mbg_assertequal(length(p), 1);
mbg_asserttrue(isequal(s, 1));


% verify slice() with just one output:
[p, s] = slice(P, 1, 1.5);
mbg_assertequal(length(p), 2);

[p, s] = slice(P1, 1, -2);
mbg_assertfalse(isfulldim(p));
