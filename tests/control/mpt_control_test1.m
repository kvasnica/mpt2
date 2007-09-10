function mpt_control_test1

% tests multi-model dynamics

% % LTI system
Double_Integrator
probStruct.N=3; probStruct.P_N = zeros(2); probStruct.Tset = unitbox(2,1);
S = {sysStruct, sysStruct, sysStruct};
sub_test(S, probStruct, 35, [-1;-0.777777777777778;0]);

% PWA+LTI system
pwa_DI
S = {};
S{1} = sysStruct;
Double_Integrator
S{2} = sysStruct;

probStruct.N = 2;
probStruct.P_N = eye(2);
probStruct.norm = 2;
% there was a bug in bnb() which caused wrong cost and wrong optimizer, so with
% this test we also verify that bnb() behaves correctly
sub_test(S, probStruct, 14, [-1; -0.77777777777]);

%----------------------------------------------
function sub_test(S, probStruct, nr, uopenloop),

exp1 = mpt_control(S, probStruct);
[C, O, V] = mpt_ownmpc(S, probStruct);
exp2 = mpt_ownmpc(S, probStruct, C, O, V);
mbg_assertequal(length(exp1), nr);
mbg_assertequal(length(exp2), nr);
mbg_asserttrue(exp1.Pfinal == exp2.Pfinal);

onl1 = mpt_control(S, probStruct, 'online');
[C, O, V] = mpt_ownmpc(S, probStruct, 'online');
onl2 = mpt_ownmpc(S, probStruct, C, O, V, 'online');

x0 = [1;1];
[ue1, ff, rr, ce1] = mpt_getInput(exp1, x0, struct('openloop',1));
[ue2, ff, rr, ce2] = mpt_getInput(exp2, x0, struct('openloop',1));
[uo1, ff, rr, co1] = mpt_getInput(onl1, x0, struct('openloop',1));
[uo2, ff, rr, co2] = mpt_getInput(onl2, x0, struct('openloop',1));
mbg_asserttolequal(ue1, uopenloop);
mbg_asserttolequal(ue1, ue2);
mbg_asserttolequal(ue1, uo1);
mbg_asserttolequal(ue1, uo2);
