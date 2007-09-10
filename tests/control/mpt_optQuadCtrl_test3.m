function mpt_optQuadCtrl_test3

% test CFTOC for PWA systems with quadratic cost
% focus on boolean/integer inputs

% 2 states, one output, yref given, 1st input real, 2nd input boolean
two_tanks
probStruct.N = 2; probStruct.norm = 2; probStruct.Tconstraint=0;
ctrl = mpt_control(sysStruct, probStruct);


%mbg_assertequal(length(ctrl), 19);
mbg_asserttrue(length(ctrl) >= 19);

% verify that we reach a given reference
[x, u, y] = sim(ctrl, [0.5; 0.17], 20);
mbg_asserttolequal(y(end), probStruct.yref, 1e-2);



% 2 states, one output, yref given, 1st input integer, 2nd input boolean
two_tanks
sysStruct.Uset{1} = [0 0.4 0.6];
probStruct.N = 1; probStruct.norm = 2; probStruct.Tconstraint=0;
ctrl = mpt_control(sysStruct, probStruct);

%mbg_assertequal(length(ctrl), 4);
mbg_asserttrue(length(ctrl) > 2);
