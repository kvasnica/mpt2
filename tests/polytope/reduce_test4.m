function reduce_test4

% checks that we return correct keptrows:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=3583353cd0a2

load reduce_wrong_keptrows

P = polytope(H, K);
Punred = polytope(H, K, 0, 2);
[Pred, keptrows] = reduce(Punred);
Pnew = polytope(H(keptrows, :), K(keptrows));

% all polytopes must be bounded
mbg_asserttrue(isbounded(P));
mbg_asserttrue(isbounded(Punred));
mbg_asserttrue(isbounded(Pred));
mbg_asserttrue(isbounded(Pnew));

% P, Pred and Pnew must have 19 constraints
mbg_assertequal(nconstr(P), 19);
mbg_assertequal(nconstr(Pred), 19);
mbg_assertequal(nconstr(Pnew), 19);

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
goodkeptrows = [4 5 11 12 13 14 16 17 18 21 22 23 25 26 27 28 30 31 38];
mbg_assertequal(keptrows(:), goodkeptrows(:));
