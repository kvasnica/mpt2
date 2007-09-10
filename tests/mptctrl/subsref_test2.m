function subsref_test2

% tests whether we can index controller object by brackets, i.e. ctrl(x0), which
% should just call mpt_getInput for the state x0:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=2b4598c8d2a2

load ctrlN2

% evaluate the controller for a given state
x0 = [1;-1];
ugood = 0.42020352856971;
u = ctrl(x0);
mbg_asserttolequal(u, ugood, 1e-6);

% evaluate the controller for state x0, extract open-loop solution
Options.openloop = 1;
ugood = [0.42020352856971; 0.52406833143126];
u = ctrl(x0, Options);
mbg_assertequal(length(u), 2); % open loop solution has 2 elements
mbg_asserttolequal(u, ugood, 1e-6);
