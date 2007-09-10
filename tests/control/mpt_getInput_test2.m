function mpt_getInput_test2

% tests whether we can call mpt_getInput() with empty Options:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=00771efe6087

load ctrl_with_hole

% this one should not throw an error:
u = mpt_getInput(ctrl_with_hole, [0.1;0.1], []);
mbg_asserttolequal(u, -0.1460, 1e-4);
