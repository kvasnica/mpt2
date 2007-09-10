function mpt_plotPWQ_test1

load ctrl_pwq
mpt_plotPWQ(ctrl.Pn, ctrl.Ai, ctrl.Bi, ctrl.Ci);

% test transparency
mpt_plotPWQ(ctrl.Pn, ctrl.Ai, ctrl.Bi, ctrl.Ci, 5, struct('shade', 0.5));
close all

% test edge colors
mpt_plotPWQ(ctrl.Pn, ctrl.Ai, ctrl.Bi, ctrl.Ci, 5, struct('edgecolor', 'w'));
close all

% test "no partition below"
mpt_plotPWQ(ctrl.Pn, ctrl.Ai, ctrl.Bi, ctrl.Ci, 5, struct('showPn', 0));
close all

% test Options.edgewidth:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cmd=changeset;node=6e28982ce208f5d8795b9a80606588bebbdebc18
mpt_plotPWQ(ctrl.Pn, ctrl.Ai, ctrl.Bi, ctrl.Ci, 5, struct('edgewidth', 3));
close all
