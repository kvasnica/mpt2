function plotj_test1

load ctrl1

ctrl = modify(ctrl, 'pick', [1 2]);

plotj(ctrl);
close all

plotj(ctrl, 3);
close all

plotj(ctrl, struct('showPn', 0));
close all

plotj(ctrl, 3, struct('showPn', 0));
close all
