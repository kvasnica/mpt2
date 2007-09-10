function mpt_optQuadCtrl_test1

% test CFTOC for PWA systems with quadratic cost

% 2 states, 2 outputs, 1 input (real)
opt_sincos
probStruct.N = 2; probStruct.norm = 2; probStruct.Tconstraint=0;
ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 10);

% 2 states, 1 output, 1 input (real)
opt_sincos
probStruct.N = 2; probStruct.norm = 2; probStruct.Tconstraint=0;
sysStruct.C{1} = [1 0]; sysStruct.D{1} = 0;
sysStruct.C{2} = [1 0]; sysStruct.D{2} = 0;
sysStruct.ymax = 5; sysStruct.ymin = -5;
ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 10);

% 2 states, 1 output, 1 input(real), no state constraints
opt_sincos
probStruct.N = 2; probStruct.norm = 2; probStruct.Tconstraint=0;
sysStruct.C{1} = [1 0]; sysStruct.D{1} = 0;
sysStruct.C{2} = [1 0]; sysStruct.D{2} = 0;
sysStruct.ymax = 5; sysStruct.ymin = -5;
sysStruct = rmfield(sysStruct, 'xmax');
sysStruct = rmfield(sysStruct, 'xmin');
ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 10);
