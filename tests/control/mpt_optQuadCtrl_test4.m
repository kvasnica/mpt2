function mpt_optQuadCtrl_test4

% test CFTOC for PWA systems with quadratic cost
% focus on yref/uref/xref

% yref
opt_sincos
probStruct.N = 3; probStruct.norm = 2; probStruct.Tconstraint=0;
sysStruct.C{1} = [1 0]; sysStruct.D{1} = 0;
sysStruct.C{2} = [1 0]; sysStruct.D{2} = 0;
sysStruct.ymax = 10; sysStruct.ymin = -10;
probStruct.yref = 0.2;
probStruct.Qy = 10;
probStruct.R = 0.1;
ctrl = mpt_control(sysStruct, probStruct);
%mbg_assertequal(length(ctrl), 16);
mbg_asserttrue(length(ctrl) >= 16);

% verify that we reach a given reference
[x, u, y] = sim(ctrl, [5; 5], 10);
mbg_asserttolequal(y(end), probStruct.yref, 1e-2);


% xref
clear probStruct sysStruct
opt_sincos
probStruct.N = 3; probStruct.norm = 2;  probStruct.Tconstraint=0;
sysStruct.C{1} = [1 0]; sysStruct.D{1} = 0;
sysStruct.C{2} = [1 0]; sysStruct.D{2} = 0;
sysStruct.ymax = 10; sysStruct.ymin = -10;
probStruct.xref = [0.2; -0.17];
probStruct.R = 0.1;
probStruct.Q = 10*eye(2);
ctrl = mpt_control(sysStruct, probStruct);
%mbg_assertequal(length(ctrl), 34);
mbg_asserttrue(length(ctrl) > 30);

% verify that we reach a given reference
[x, u, y] = sim(ctrl, [5; 5], 10);
mbg_asserttolequal(x(end, :)', probStruct.xref, 1e-2);
