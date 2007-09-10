function sub_computeTrajectory_test1

% test if sub_computeTrajectory() can be called with just two input arguments:
% http://control.ee.ethz.ch/~mpt/hg/mpt-noise?cmd=changeset;node=8b6376804c057d9291ec695620ac3e486749074e

load lti1d_addu_ctrl

[x,u,y] = sub_computeTrajectory(ctrl, 0.5);

[x,u,y] = sub_computeTrajectory(ctrl, 0.5, 10);

[x,u,y] = sub_computeTrajectory(ctrl, 0.5, []);

[x,u,y] = sub_computeTrajectory(ctrl, 0.5, 10, struct('verbose',0));
