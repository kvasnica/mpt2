function reduce_test5

% tests that we deal with Inf terms in K matrix correctly
% related to http://control.ee.ethz.ch/~mpt/hg/mpt?cs=3583353cd0a2

H = [1; 1; -1; 1; 1; -1; 1; 1];
K = [Inf; Inf; 2; 4; 2; 4; Inf; Inf];

P = polytope(H, K);
Punred = polytope(H, K, 0, 2);
[Pred, keptrows] = reduce(Punred);
Pnew = polytope(H(keptrows, :), K(keptrows));

% all polytopes must be bounded
mbg_asserttrue(isbounded(P));
mbg_asserttrue(isbounded(Punred));
mbg_asserttrue(isbounded(Pred));
mbg_asserttrue(isbounded(Pnew));

% P, Pred and Pnew must have 2 constraints
mbg_assertequal(nconstr(P), 2);
mbg_assertequal(nconstr(Pred), 2);
mbg_assertequal(nconstr(Pnew), 2);

% check that polytopes are equal
mbg_asserttrue(P==Punred);
mbg_asserttrue(P==Pred);
mbg_asserttrue(P==Pnew);

% check whether keptrows are correct
hku = double(Punred);  % we must do this instead of "hku=[H K]" because [H K] are not normalized!!
hkr = double(Pred);
D = abs(hku(keptrows, :) - hkr);
mbg_asserttrue(max(max(D)) < 1e-10); % matrices are identical

% check whether keptrows are sorted correctly
goodkeptrows = [3 5];
mbg_assertequal(keptrows(:), goodkeptrows(:));
