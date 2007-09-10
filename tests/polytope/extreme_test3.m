function extreme_test3

% tests Options.roundat:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cmd=changeset;node=f858644173da7e01744f387570654a58ce5c64b7

load extreme_test3
Options.verbose = 2;

% we are just testing CDD since that's the solver which causes problems here.
% first use no rounding (i.e. Options.roundat = Inf):
fprintf('computing extreme points\n');
Options.roundat = Inf;
Options.extreme_solver = 3;
e1 = extreme(p(1), Options); e2 = extreme(p(2), Options);
E = [e1; e2];

% now compute convex hull - with CDD and Options.roundat=Inf it will be
% unbounded. however, commit 6c3ea374351e fixes this problem by switching to
% different solver, as can be seen with verbose=2
fprintf('computing hull\n');
H = hull(E, Options);
mbg_asserttrue(isbounded(H));

% now use rounding at 15 decimal points (should be default in extreme()):
fprintf('computing extreme points\n');
clear Options
Options.extreme_solver = 3;
e1 = extreme(p(1), Options); e2 = extreme(p(2), Options);
E = [e1; e2];

% now compute convex hull with CDD and no rounding - it should be bounded,
% because the rounding in extreme() already helps:
fprintf('computing hull\n');
Options.roundat = Inf;
H = hull(E, Options);
mbg_asserttrue(isbounded(H));

% radius of chebyball should be roughly 0.1540 for this example
[x, r] = chebyball(H);
mbg_asserttolequal(r, 0.1540, 1e-4);
