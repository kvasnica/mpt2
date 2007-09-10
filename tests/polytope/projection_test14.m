function projection_test14

% iterative hull used to cycle on this example (with NAG LP solver under linux).
% fixed by:
% http://control.ee.ethz.ch/~mpt/hg/mpt-25x?cs=af2e36fa586f

load projection_iterhull_cycle
P = projection(PA, dim, struct('projection', 2));
