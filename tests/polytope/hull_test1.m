function hull_test1

% test the "emergency" mode in hull() in which we try to solve the problem with
% different method if result is not good:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cmd=changeset;node=872276717b0cf3d93c3a7c8e9c1eacd7c4aeda83

% note that this fix is superseeded by commit 6c3ea374351e

load hull_test1

% CDD fails on this example, gives unbounded polytope. the emergency mode
% recognizes something is wrong (because input polytopes are bounded) and tries
% to use different method
Options.extreme_solver = 3;  % this problems happens only with CDD
Options.verbose = 2;         % verbose=2 should print out relevant information
Options.roundat = Inf;       % this happens if no rounding is used
h = hull(p, Options);

% hull_test1.out contains the correct output which should be printed to screen
% if verbose > 1
mbg_asserttrue(isbounded(h));

% also test if the resulting hull is what we expect
[x, r] = chebyball(h);
mbg_asserttolequal(r, 0.1539625, 1e-6);

% now test that we have no output on screen when verbose = 1
Options.verbose = 1;
T = evalc('hull(p, Options);');
mbg_asserttrue(isempty(T));
