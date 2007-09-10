function subsref_test3

% tests whether we return a vector of NaNs if problem is infeasible
%

Double_Integrator
probStruct.Tconstraint = 0;
probStruct.N = 3;
ctrl = mpt_control(sysStruct, probStruct, 'online');

% feasible problem, RHC control action
u = ctrl([1;1]);
mbg_asserttolequal(u, -1);

% feasible problem, open-loop control action
u = ctrl([1;1], struct('openloop',1));
mbg_asserttolequal(u, [-1; -0.7778; 0], 1e-4);

% infeasible problem, RHC control action
u=ctrl([100; 100]);
mbg_asserttrue(isnan(u));
mbg_assertequal(length(u), 1);

% infeasible problem, open-loop control action
u=ctrl([100; 100], struct('openloop',1));
mbg_asserttrue(isnan(u));
mbg_assertequal(length(u), probStruct.N);
