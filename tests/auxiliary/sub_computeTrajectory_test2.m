function sub_computeTrajectory_test2

% test if sub_computeTrajectory() uses noise also when N and Options.minnorm=-1
% is specified:
% http://control.ee.ethz.ch/~mpt/hg/mpt-noise?cmd=changeset;node=99ac14e4d5c9a7125d49bbeb977551ecb61c9e8c

load lti1d_addu_ctrl

[x,u,y,d] = sub_computeTrajectory(ctrl, 0.5, 10);
% noise must be present
mbg_asserttrue(any(d~=0));


[x,u,y,d] = sub_computeTrajectory(ctrl, 0.5, 10, struct('minnorm', -1));
% noise must be present
mbg_asserttrue(any(d~=0));

[x,u,y,d] = sub_computeTrajectory(ctrl, 0.5, 10, struct('randdist', 0));
% noise must NOT be present
mbg_asserttrue(all(d==0));
