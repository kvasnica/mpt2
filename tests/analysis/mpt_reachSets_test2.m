function mpt_reachSets_test2

% tests mpt_reachSets() when input is a system structure

Double_Integrator
X0 = unitbox(2, 1)+[4;0];
U0 = unitbox(1, 1);
N = 5;

% with user-defined U0 polytope:
[R, V, Rn, Vn] = mpt_reachSets(sysStruct, X0, U0, N);
mbg_asserttrue(isa(R, 'polytope'));
mbg_asserttrue(iscell(Rn));
mbg_asserttrue(isempty(V));
mbg_asserttrue(isempty([Vn{:}]));
mbg_assertequal(length(R), N);
mbg_assertequal(length(Rn), N);

% without user-defined U0 polytope:
[R, V, Rn, Vn] = mpt_reachSets(sysStruct, X0, N);
mbg_asserttrue(isa(R, 'polytope'));
mbg_asserttrue(iscell(Rn));
mbg_asserttrue(isempty(V));
mbg_asserttrue(isempty([Vn{:}]));
mbg_assertequal(length(R), N);
mbg_assertequal(length(Rn), N);
