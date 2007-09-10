function sim_exp_test1

% tests simulation of an explicit controller, regulation problems
x0 = [1; 1];
assignin('base', 'sim_x0', x0);
ref = 0;
assignin('base', 'sim_ref', ref);
Double_Integrator
assignin('base', 'sim_sysStruct', sysStruct);


%----------------------------------------------------------
% regulation problem, no deltaU constraints
clear sysStruct probStruct
Double_Integrator
probStruct.N = 3;
ctrl = mpt_control(sysStruct, probStruct);
assignin('base', 'sim_ctrl', ctrl);
sim('sim_exp');

% solution always feasible?
mbg_asserttrue(all(sim_reg ~= 0));
% solution converged to origin?
mbg_asserttrue(norm(sim_x(end, :)) < 1e-1);



%----------------------------------------------------------
% regulation problem, with deltaU constraints
clear sysStruct probStruct
Double_Integrator
probStruct.N = 2;
sysStruct.dumax = 0.5;
sysStruct.dumin = -0.5;
ctrl = mpt_control(sysStruct, probStruct);
assignin('base', 'sim_ctrl', ctrl);
sim('sim_exp');

% solution always feasible?
mbg_asserttrue(all(sim_reg ~= 0));
% solution converged to origin?
mbg_asserttrue(norm(sim_x(end, :)) < 1e-1);
% deltaU constraints satisfied?
mbg_asserttrue(max(diff(sim_u)) <= sysStruct.dumax);
mbg_asserttrue(sysStruct.dumin <= min(diff(sim_u)));


%----------------------------------------------------------
% regulation problem, no deltaU constraints, mpt_ownmpc()
clear sysStruct probStruct
Double_Integrator
probStruct.N = 3;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
ctrl = mpt_ownmpc(sysStruct, probStruct, C, O, V);
assignin('base', 'sim_ctrl', ctrl);
sim('sim_exp');

% solution always feasible?
mbg_asserttrue(all(sim_reg ~= 0));
% solution converged to origin?
mbg_asserttrue(norm(sim_x(end, :)) < 1e-1);



%----------------------------------------------------------
% regulation problem, with deltaU constraints, mpt_ownmpc()
clear sysStruct probStruct
Double_Integrator
probStruct.N = 2;
sysStruct.dumax = 0.5;
sysStruct.dumin = -0.5;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
ctrl = mpt_ownmpc(sysStruct, probStruct, C, O, V);
assignin('base', 'sim_ctrl', ctrl);
sim('sim_exp');

% solution always feasible?
mbg_asserttrue(all(sim_reg ~= 0));
% solution converged to origin?
mbg_asserttrue(norm(sim_x(end, :)) < 1e-1);
% deltaU constraints satisfied?
mbg_asserttrue(max(diff(sim_u)) <= sysStruct.dumax);
mbg_asserttrue(sysStruct.dumin <= min(diff(sim_u)));


