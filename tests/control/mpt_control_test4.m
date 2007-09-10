function mpt_control_test4

% tests on-line MPC with linear cost and unbounded states:
%

% pure LP
Double_Integrator
probStruct.norm = 1;
probStruct.Tconstraint = 0;
sysStruct.ymax = [Inf; Inf];
sysStruct.ymin = [-Inf; -Inf];
ctrl = mpt_control(sysStruct, probStruct, 'online');
[x,u,y,c] = sim(ctrl, [1;1], 10);
mbg_asserttolequal(c, 7.35937500000000)

% pure QP
Double_Integrator
probStruct.norm = 2;
probStruct.Tconstraint = 0;
sysStruct.ymax = [Inf; Inf];
sysStruct.ymin = [-Inf; -Inf];
ctrl = mpt_control(sysStruct, probStruct, 'online');
[x,u,y,c] = sim(ctrl, [1;1], 10);
mbg_asserttolequal(c, 5.70185717485871)

% MILP
Double_Integrator
sysStruct.Uset = [-1 0 1];
probStruct.norm = 1;
probStruct.Tconstraint = 0;
sysStruct.ymax = [Inf; Inf];
sysStruct.ymin = [-Inf; -Inf];
ctrl = mpt_control(sysStruct, probStruct, 'online');
[x,u,y,c] = sim(ctrl, [1;1], 5);
mbg_asserttolequal(c, 7)

% MIQP
Double_Integrator
sysStruct.Uset = [-1 0 1];
probStruct.norm = 2;
probStruct.Tconstraint = 0;
sysStruct.ymax = [Inf; Inf];
sysStruct.ymin = [-Inf; -Inf];
ctrl = mpt_control(sysStruct, probStruct, 'online');
[x,u,y,c] = sim(ctrl, [1;1], 5);
mbg_asserttolequal(c, 6)
