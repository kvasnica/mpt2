function sim_regul_test1

% tests simulation of on-line controllers, tracking problems
x0 = [1; 1];
assignin('base', 'sim_x0', x0);

%----------------------------------------------------------
% nx == ny
%----------------------------------------------------------
ref = [-1; 0];
assignin('base', 'sim_ref', ref);
Double_Integrator
assignin('base', 'sim_sysStruct', sysStruct);


%----------------------------------------------------------
% tracking=1
clear sysStruct probStruct
Double_Integrator
probStruct.N = 2;
probStruct.tracking = 1;
ctrl = mpt_control(sysStruct, probStruct, 'online');
assignin('base', 'sim_ctrl', ctrl);
sim('sim_online');

% solution always feasible?
mbg_asserttrue(all(sim_reg ~= 0));
% solution converged to reference?
mbg_asserttrue(norm(sim_x(end, :)' - ref) < 0.11);



%----------------------------------------------------------
% tracking=2
clear sysStruct probStruct
Double_Integrator
probStruct.N = 2;
probStruct.tracking = 2;
ctrl = mpt_control(sysStruct, probStruct, 'online');
assignin('base', 'sim_ctrl', ctrl);
sim('sim_online');

% solution always feasible?
mbg_asserttrue(all(sim_reg ~= 0));
% solution converged to reference?
mbg_asserttrue(norm(sim_x(end, :)' - ref) < 0.06);



%----------------------------------------------------------
% tracking=1, mpt_ownmpc()
clear sysStruct probStruct
Double_Integrator
probStruct.N = 2;
probStruct.tracking = 1;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl = mpt_ownmpc(sysStruct, probStruct, C, O, V, 'online');
assignin('base', 'sim_ctrl', ctrl);
sim('sim_online');

% solution always feasible?
mbg_asserttrue(all(sim_reg ~= 0));
% solution converged to reference?
mbg_asserttrue(norm(sim_x(end, :)' - ref) < 0.11);



%----------------------------------------------------------
% tracking=2, mpt_ownmpc()
clear sysStruct probStruct
Double_Integrator
probStruct.N = 2;
probStruct.tracking = 2;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl = mpt_ownmpc(sysStruct, probStruct, C, O, V, 'online');
assignin('base', 'sim_ctrl', ctrl);
sim('sim_online');

% solution always feasible?
mbg_asserttrue(all(sim_reg ~= 0));
% solution converged to reference?
mbg_asserttrue(norm(sim_x(end, :)' - ref) < 0.06);



%----------------------------------------------------------
% ny ~= nx
%----------------------------------------------------------
Double_Integrator
sysStruct.C = [1 0]; sysStruct.D = 0; sysStruct.ymax=5; sysStruct.ymin=-5;
assignin('base', 'sim_sysStruct', sysStruct);
ref = -1;
assignin('base', 'sim_ref', ref);


%----------------------------------------------------------
% tracking=1, ny~=nx
clear sysStruct probStruct
Double_Integrator
sysStruct.C = [1 0]; sysStruct.D = 0; sysStruct.ymax=5; sysStruct.ymin=-5;
probStruct.Qy = 10;
probStruct.N = 2;
probStruct.tracking = 1;
ctrl = mpt_control(sysStruct, probStruct, 'online');
assignin('base', 'sim_ctrl', ctrl);
sim('sim_online');

% solution always feasible?
mbg_asserttrue(all(sim_reg ~= 0));
% solution converged to reference?
mbg_asserttrue(norm(sim_y(end, :)' - ref) < 5e-4);



%----------------------------------------------------------
% tracking=2, ny~=nx
clear sysStruct probStruct
Double_Integrator
sysStruct.C = [1 0]; sysStruct.D = 0; sysStruct.ymax=5; sysStruct.ymin=-5;
probStruct.Qy = 10;
probStruct.N = 2;
probStruct.tracking = 2;
ctrl = mpt_control(sysStruct, probStruct, 'online');
assignin('base', 'sim_ctrl', ctrl);
sim('sim_online');

% solution always feasible?
mbg_asserttrue(all(sim_reg ~= 0));
% solution converged to reference?
mbg_asserttrue(norm(sim_y(end, :)' - ref) < 1.3e-3);



%----------------------------------------------------------
% tracking=1, ny~=nx, mpt_ownmpc()
clear sysStruct probStruct
Double_Integrator
sysStruct.C = [1 0]; sysStruct.D = 0; sysStruct.ymax=5; sysStruct.ymin=-5;
probStruct.Qy = 10;
probStruct.N = 2;
probStruct.tracking = 1;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl = mpt_ownmpc(sysStruct, probStruct, C, O, V, 'online');
assignin('base', 'sim_ctrl', ctrl);
sim('sim_online');

% solution always feasible?
mbg_asserttrue(all(sim_reg ~= 0));
% solution converged to reference?
mbg_asserttrue(norm(sim_y(end, :)' - ref) < 5e-4);



%----------------------------------------------------------
% tracking=2, ny~=nx, mpt_ownmpc()
clear sysStruct probStruct
Double_Integrator
sysStruct.C = [1 0]; sysStruct.D = 0; sysStruct.ymax=5; sysStruct.ymin=-5;
probStruct.Qy = 10;
probStruct.N = 2;
probStruct.tracking = 2;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl = mpt_ownmpc(sysStruct, probStruct, C, O, V, 'online');
assignin('base', 'sim_ctrl', ctrl);
sim('sim_online');

% solution always feasible?
mbg_asserttrue(all(sim_reg ~= 0));
% solution converged to reference?
mbg_asserttrue(norm(sim_y(end, :)' - ref) < 1.3e-3);

