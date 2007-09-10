function mpt_ownmpc_test7


% tests mpt_ownmpc for on-line controllers

fprintf('"standard" setup, no deltaU constraints, no tracking...\n');
Double_Integrator
[C,O,V]=mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl=mpt_ownmpc(sysStruct,probStruct,C,O,V,'online');
[x,u] = sim(ctrl, [2;2], 5);
mbg_asserttolequal(x(end, :), [2 -0.5]);


fprintf('\n\nnow test tracking...\n');
% (in fact the algorithm should automatically switch to
% tracking=2 since we have no deltaU constraints)
reference = [1; 0];
Double_Integrator
probStruct.tracking = 1;
[C,O,V]=mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl=mpt_ownmpc(sysStruct,probStruct,C,O,V,'online');
[x,u] = sim(ctrl, [2;2], 20, struct('reference', reference));
mbg_asserttolequal(x(end, :)', reference, 5e-4);


fprintf('\n\nnow test tracking=2...\n');
% (should be identical to tracking=1 becuase we have no deltaU constraints)
reference = [1; 0];
Double_Integrator
probStruct.tracking = 2;
[C,O,V]=mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl=mpt_ownmpc(sysStruct,probStruct,C,O,V,'online');
[x,u] = sim(ctrl, [2;2], 20, struct('reference', reference));
mbg_asserttolequal(x(end, :)', reference, 5e-4);


fprintf('\n\nnow test deltaU constraints...\n');
Double_Integrator
sysStruct.dumax = 0.4;
sysStruct.dumin = -0.3;
[C,O,V]=mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl=mpt_ownmpc(sysStruct,probStruct,C,O,V,'online');
[x,u] = sim(ctrl, [1;1], 10);
mbg_asserttrue(max(diff(u)) <= sysStruct.dumax+sqrt(eps));
mbg_asserttrue(min(diff(u)) >= sysStruct.dumin-sqrt(eps));


fprintf('\n\nnow test probStruct.Rdu...\n'); 
% we should automatically switch to deltaU formulation
Double_Integrator
probStruct.Rdu = 1;
[C,O,V]=mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl=mpt_ownmpc(sysStruct,probStruct,C,O,V,'online');
[x,u] = sim(ctrl, [1;1], 10);
mbg_asserttrue(isfield(V, 'uprev') | length(V.x{1}==3)); % this is an indicator that we take Rdu into account


fprintf('\n\nnow test tracking with deltaU constraints...\n');
reference = [1; 0];
Double_Integrator
sysStruct.dumax = 0.4;
sysStruct.dumin = -0.3;
probStruct.tracking = 1;
[C,O,V]=mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl=mpt_ownmpc(sysStruct,probStruct,C,O,V,'online');
[x,u] = sim(ctrl, [1;1], 20, struct('reference', reference));
mbg_asserttrue(max(diff(u)) <= sysStruct.dumax+sqrt(eps));
mbg_asserttrue(min(diff(u)) >= sysStruct.dumin-sqrt(eps));
mbg_asserttolequal(x(end, :)', reference, 5e-4);


fprintf('\n\nnow test tracking=1 with probStruct.Rdu...\n');
reference = [1; 0];
Double_Integrator
probStruct.tracking = 1;
probStruct.Rdu = 1;
[C,O,V]=mpt_ownmpc(sysStruct, probStruct, 'online');
mbg_asserttrue(isfield(V, 'uprev') | length(V.x{1}==5)); % this is an indicator that we take Rdu into account
mbg_asserttrue(isfield(V, 'ref') | length(V.x{1}==5)); % this is an indicator that we take tracking=1 into account
ctrl=mpt_ownmpc(sysStruct,probStruct,C,O,V,'online');
[x,u] = sim(ctrl, [1;1], 20, struct('reference', reference));
mbg_asserttolequal(x(end, :)', reference, 5e-4);

