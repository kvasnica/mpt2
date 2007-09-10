function mpt_plotPWQ_test2

% test whether Options can be passed as 5th input argument:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cmd=changeset;node=27f1c9a4569fab25b6dcc6d55935ef64253e5f1e
%
% if anybody wants to give additional optinas to mpt_plotPWQ, he had
% to provide "meshgridpoints" first, i.e.:
% 
% mpt_plotPWA(Pn, A, B, C, meshgridpoints, Options)
% 
% now it is possible to pass Options as 5th argument, in which case
% "meshgridpoints" will be defaulted to 30, i.e.
% 
% mpt_plotPWQ(Pn, A, B, C, Options)

load ctrl_pwq

% before 29.9.2005 this would fail with an error:
mpt_plotPWQ(ctrl.Pn, ctrl.Ai, ctrl.Bi, ctrl.Ci, struct('shade', 0.5));
close all
