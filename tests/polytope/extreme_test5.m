function extreme_test5

% CDD fails on these two polytopes as of Nov-18-2005.
% it was fixed by:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=747847fec082

load extreme_test5

% apparently it can also be fixed by Options.roundat=12

E3 = pelemfun(@extreme, P, struct('extreme_solver', 3));
E0 = pelemfun(@extreme, P, struct('extreme_solver', 0));
E1 = pelemfun(@extreme, P, struct('extreme_solver', 1));

mbg_asserttolequal(size(E3{1}, 1), size(E0{1}, 1), 1);
mbg_asserttolequal(size(E1{1}, 1), size(E0{1}, 1), 1);
mbg_asserttolequal(size(E3{2}, 1), size(E0{2}, 1), 1);
mbg_asserttolequal(size(E1{2}, 1), size(E0{2}, 1), 1);
