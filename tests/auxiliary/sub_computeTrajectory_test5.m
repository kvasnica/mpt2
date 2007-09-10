function sub_computeTrajectory_test5

% tests how we cope with lower-dimensional noise in V-representation
load ctrl_lowdimnoise

x1 = sub_computeTrajectory(ctrl, [1; 0], 5);
x2 = sub_computeTrajectory(ctrl, [1; 0], 5);

% simulation must be affected by the noise, therefore we check that the two
% evolutions are not the same
mbg_asserttrue(max(max(abs(x1-x2)))>1e-2);
