function mpt_optQuadCtrl_test2

% test CFTOC for PWA systems with quadratic cost
% focus on probStruct.P_N and probStruct.Tset


% P_N=eye(2)
opt_sincos
probStruct.N = 2; probStruct.norm = 2; probStruct.Tconstraint=0;
probStruct.P_N = 10*eye(2);
ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 20);


% Tset given as a single polytope
opt_sincos
probStruct.N = 2; probStruct.norm = 2; probStruct.Tconstraint=0;
probStruct.Tset = unitbox(2, 0.5);
ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 30);
% verify by simulation that we enter the target set
% (although in theory it is only guanrateed in open-loop)
x = sim(ctrl, [3; 0], 10);
mbg_asserttrue(isinside(probStruct.Tset, x(4, :)'));


% Tset given as a polytope array
opt_sincos
probStruct.N = 2; probStruct.norm = 2; probStruct.Tconstraint=0;
p1 = unitbox(2, 0.5) + [0.5; 0];
p2 = unitbox(2, 0.5) + [-0.5; 0];
probStruct.Tset = [p1 p2];
ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 50);
% verify by simulation that we enter the target set
% (although in theory it is only guanrateed in open-loop)
x = sim(ctrl, [3; 0], 10);
mbg_asserttrue(isinside(probStruct.Tset, x(4, :)'));
