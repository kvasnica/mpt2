function mpt_plotPWA_test1

load ctrl_pwa
mpt_plotPWA(ctrl.Pn, ctrl.Bi, ctrl.Ci);

% test transparency
mpt_plotPWA(ctrl.Pn, ctrl.Bi, ctrl.Ci, struct('shade', 0.5));
close all

% test edge colors
mpt_plotPWA(ctrl.Pn, ctrl.Bi, ctrl.Ci, struct('edgecolor', 'w'));
close all

% test "no partition below"
mpt_plotPWA(ctrl.Pn, ctrl.Bi, ctrl.Ci, struct('showPn', 0));
close all

% test "show partition below"
mpt_plotPWA(ctrl.Pn, ctrl.Bi, ctrl.Ci, struct('showPn', 1));
close all

% test Options.edgewidth:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cmd=changeset;node=6e28982ce208f5d8795b9a80606588bebbdebc18
mpt_plotPWA(ctrl.Pn, ctrl.Bi, ctrl.Ci, struct('edgewidth', 3));
close all
