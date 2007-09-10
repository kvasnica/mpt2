function hull_test1

% Johan Loefberg, 24 Sep 2005, 17:01
% 
% load extreme_test3.mat
% plot(polytope(x_opt'))
% Plotting...
% PLOT: the polytope is unbounded

% fixed by
% http://control.ee.ethz.ch/~mpt/hg/mpt-20x?cmd=changeset;node=0be43f8fb8af150cf3912220965a6b88aa2d6bd8

% CDD fails to reduce the V-representation, leading to Vconv=[]. a bug in
% hull/checkhull() didn't catch this case.

load hull_test1.mat
p = hull(x_opt', struct('extreme_solver',3));

% the result must be bounded
mbg_assertequal(isbounded(p), 1);

% and have 354 facets (that's what is returned by extreme_solver=0)
mbg_assertequal(nconstr(p), 354);
