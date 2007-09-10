function uplus_test1

% uplus should either do nothing if the polytope is in minimal representation,
% or reduce it if it is not in minimal representation

reduced = polytope([eye(2); -eye(2)], [1;1;1;1]);
unreduced = polytope([eye(2); -eye(2)], [1;1;1;1], 0, 2);

% this one should not do anything
a = +reduced;
mbg_asserttrue(reduced==a);

% this one should reduce the polytope "unreduced"
a = +unreduced;
mbg_asserttrue(a==unitbox(2,1));
mbg_asserttrue(isminrep(a));

% test polyarrays
P = [reduced; unreduced];
a = +P;
mbg_asserttrue(all(isminrep(a)));
