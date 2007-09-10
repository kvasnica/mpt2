function mpt_optQuadCtrl_test5

% test CFTOC for PWA systems with quadratic cost
% focus on tracking

% tracking=2, some inputs boolean
two_tanks
probStruct.N=2; probStruct.norm=2; probStruct.Tconstraint=0;
probStruct = rmfield(probStruct, 'yref');
probStruct.tracking=2;
ctrl = mpt_control(sysStruct, probStruct);
% test that we reach a given reference
Options.reference = 0.16;
[x, u, y] = sim(ctrl, [0.5; 0.1], 20, Options);
mbg_asserttolequal(y(end)', Options.reference, 0.01);


% tracking=1, all inputs real
opt_sincos
probStruct.N = 2; probStruct.norm = 2; probStruct.Tconstraint=0;
sysStruct.C{1} = [0 1]; sysStruct.D{1} = 0; sysStruct.g{1} = 0;
sysStruct.C{2} = [0 1]; sysStruct.D{2} = 0; sysStruct.g{2} = 0;
sysStruct.ymax = 5; sysStruct.ymin = -5;
probStruct.tracking = 1;
probStruct.Qy = 100;
probStruct.P_N = 0;
ctrl = mpt_control(sysStruct, probStruct);
% test that we reach a given reference
Options.reference = 0.5;
[x, u, y] = sim(ctrl, [5; 5], 20, Options);
mbg_asserttolequal(y(end)', Options.reference, 0.01);
