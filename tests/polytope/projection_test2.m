function projection_test2

% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=4fd50f8eee77

load projection_prob2
% this used to segfault the mex-fourier method, because "dim" was [1 2 12]
dim = [1 2 12];
Q = projection(PA, dim, struct('projection', 5));

% test the same problem once more. this time mex-fourier is not segfaulting, but
% we get the "Error using ==> -, Matrix dimensions must agree." error.
P = unitbox(4,1);
dim = 4;
Q = projection(P, dim, struct('projection', 5));

% this was giving incorrect results due to wrong use of randomization:
Q = projection(unitbox(3, 1), [3 1], struct('projection', 5));
mbg_asserttrue(Q==unitbox(2,1));
