function plotu_test1

load ctrl1

plotu(ctrl);
close all

plotu(ctrl, 1);
close all

plotu(ctrl, struct('showPn', 1));
close all

plotu(ctrl, 1, struct('showPn', 1));
close all
