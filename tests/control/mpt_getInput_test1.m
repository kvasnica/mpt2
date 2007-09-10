function mpt_getInput_test1

% tests the "recovery" mode:
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

% without the recovery mode we should get an empty output
u = mpt_getInput(ctrl_with_hole, x0);
mbg_asserttrue(isempty(u));

% with the recovery mode we should get control law of nearest neighbour
u = mpt_getInput(ctrl_with_hole, x0, struct('recover', 1));
mbg_asserttolequal(u, -0.4810882185, 1e-8);
