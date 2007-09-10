function mpt_computeTrajectory_test2

% tests the "recovery" mode used in mpt_getInput:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cmd=changeset;node=538b97ef6abeb742f55dd6da4789eca766ecc3dd
%
% [NEW] allow to use a "recovery" mode for mpt_getInput()
% 
% If Options.recovery is set to 1 and:
% A) current state belongs to the feasible set, but
% B) there is no region associate to x0
% then we use the control law of nearest neighbour.
% 
% Options.recovery is 0 by default.

load ctrl_with_hole
x0 = [4; -1.7];  % this state is in a "hole"

% this should give an infeasible trajectory because x0 lies in a hole
disp('No recovery mode, no feasible control law should be found.');
[x, u, y] = mpt_computeTrajectory(ctrl_with_hole, x0);
mbg_assertequal(x, x0');
mbg_asserttrue(isempty(u));
mbg_asserttrue(isempty(y));

% this should give a feasible trajectory because we use the recovery mode
disp('Recovery mode enable, feasible control law should be found.');
[x, u, y] = mpt_computeTrajectory(ctrl_with_hole, x0, [], struct('recover', 1));
% test that we reached the origin
mbg_asserttolequal(x(end, :), [0 0], 1e-2);