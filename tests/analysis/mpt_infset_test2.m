function mpt_infset_test2

% mpt_infset() gave wrong results when additive noise was specified. fixed
% by calling mpt_infsetPWA() from mpt_infset()

A = [0.480245992547641 0.0600424639779321;-0.259877003726179 0.530021231988966];
Ex = [5 -3.82865172025952;5 -1.70089602560967;-5 1.70089602560967;-5 3.82865172025952];
X = polytope(Ex);
noise = unitbox(2, 0.1);
Ei = [2.5200362879052 -1.63858904767551;
    3.78650054633585 -1.02988570874746;
    -1.01185240447142 1.62338646555465;
    1.01185240447142 -1.62338646555465;
    -2.5200362879052 1.63858904767551;
    -3.78650054633585 1.02988570874746];
correct_invset = polytope(Ei);

fprintf('Verbosity enabled:\n');
I = mpt_infset(A, X, 100, noise, struct('verbose', 1));
mbg_asserttrue(I == correct_invset);


fprintf('\n\nVerbosity should be disabled by default:\n');
I = mpt_infset(A, X, 100, noise);

fprintf('\n\n---------------------------\n');
