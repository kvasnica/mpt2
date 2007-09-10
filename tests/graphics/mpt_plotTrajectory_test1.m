function mpt_plotTrajectory_test1

% tests Options.showPn in mpt_plotTrajectory:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=5c9c6407a6d2

load ctrl_pwq

% showPn should be on by default
mpt_plotTrajectory(ctrl, struct('x0', [0.5; 0], 'maxCtr', 10));
nch = length(get(get(gcf, 'Children'), 'Children'));
close all
mbg_assertequal(nch, 23);

% now don't plot the partition
mpt_plotTrajectory(ctrl, struct('x0', [0.5; 0], 'maxCtr', 10, 'showPn', 0));
nch = length(get(get(gcf, 'Children'), 'Children'));
close all
mbg_assertequal(nch, 20);  % partition has 3 regions which should not be displayed now
