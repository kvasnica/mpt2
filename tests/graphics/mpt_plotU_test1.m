function mpt_plotU_test1

% test plotting of multiple inputs for 1D systems:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cmd=changeset;node=98002ced73241d2c4e6f392ca42956704f294796

sysStruct.A = 1;
sysStruct.B = [1 1];
sysStruct.C = 1;
sysStruct.D = [0 0];
sysStruct.umax = [1; 0.1];
sysStruct.umin = [-0.1; -1];
sysStruct.ymax = 2;
sysStruct.ymin = -2;

% we use higher prediction horizon because it makes Fi and Gi more complicated,
% tests if we are handling such cases correctly
probStruct.N = 3;
probStruct.norm = 2;
probStruct.Q = 1;
probStruct.R = eye(2);

ctrl = mpt_control(sysStruct, probStruct);

% this should plot 2 subplots
mpt_plotU(ctrl);
close all

% this should plot just the first input
mpt_plotU(ctrl, 1);
close all

% this should plot just the second input
mpt_plotU(ctrl, 2);
close all
