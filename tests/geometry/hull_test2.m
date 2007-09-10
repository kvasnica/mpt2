function hull_test2

% test Options.roundat:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cmd=changeset;node=f858644173da7e01744f387570654a58ce5c64b7

% for CDD it helps to round the V-representation to certain number of decimal
% places (usually 14)

load hull_test2

% this one does no rounding, leading to an unbounded convex hull with CDDMEX
% 0.94b. however, commit 6c3ea374351e seems to solve the problem (another solver
% will be used, which can be seen with verbose=2), therefore we 
% expect a bounded solution
%
% in some random cases hull() fails with a "HULL: Point 8 is not a
% vertex!" error message. however resolving the same problem just after is
% fine
try
    H = hull(E, struct('roundat', Inf, 'extreme_solver', 3, 'verbose', 2));
catch
    pause(3*rand(1));
    try
        H = hull(E, struct('roundat', Inf, 'extreme_solver', 3, 'verbose', 2));
    catch
        pause(3*rand(1));
        H = hull(E, struct('roundat', Inf, 'extreme_solver', 3, 'verbose', 2));
    end
end
mbg_asserttrue(isbounded(H));

% this one uses Options.roundat=14 (default) and leads to a correct
% (bounded) convex hull:
H = hull(E, struct('extreme_solver', 3));
mbg_asserttrue(isbounded(H));

% radius of chebyball should be roughly 0.1540 for this example
[x, r] = chebyball(H);
mbg_asserttolequal(r, 0.1540, 1e-4);
