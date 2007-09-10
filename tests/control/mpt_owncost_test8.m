function mpt_ownmpc_test8

% same as mpt_ownmpc_test7 but this time with a PWA system

% tests mpt_ownmpc for on-line controllers

% with YALMIP never than from 21.4.2006 state constraints must be defined for
% this particular test, otherwise CPLEX will fail due to loose big-M relaxation
fprintf('"standard" setup, no deltaU constraints, no tracking...\n');
pwa_DI; probStruct.N=3; probStruct.subopt_lev = 0; probStruct.Tconstraint = 0;
probStruct.norm = 1;
sysStruct.xmax = sysStruct.ymax;
sysStruct.xmin = sysStruct.ymin;
[C,O,V]=mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl=mpt_ownmpc(sysStruct,probStruct,C,O,V,'online');
[x,u,y,cost] = sim(ctrl, [2;2], 5);
mbg_asserttolequal(cost, 24);
mbg_asserttolequal(x(end, :), [3 0]);


fprintf('\n\nnow test tracking...\n');
% (in fact the algorithm should automatically switch to
% tracking=2 since we have no deltaU constraints)
reference = [1; 0];
pwa_DI; probStruct.N=3; probStruct.subopt_lev = 0; probStruct.Tconstraint = 0;
probStruct.norm = 1;
probStruct.tracking = 1;
[C,O,V]=mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl=mpt_ownmpc(sysStruct,probStruct,C,O,V,'online');
[x,u,y,c] = sim(ctrl, [1;1], 5, struct('reference', reference));
mbg_asserttolequal(x(end, :)', reference);
mbg_asserttolequal(c, 3.3, 5e-3);


fprintf('\n\nnow test tracking=2...\n');
% (should be identical to tracking=1 becuase we have no deltaU constraints)
reference = [1; 0];
pwa_DI; probStruct.N=3; probStruct.subopt_lev = 0; probStruct.Tconstraint = 0;
probStruct.norm = 1;
probStruct.tracking = 2;
[C,O,V]=mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl=mpt_ownmpc(sysStruct,probStruct,C,O,V,'online');
[x,u,y,c] = sim(ctrl, [1;1], 5, struct('reference', reference));
mbg_asserttolequal(c, 3.8750, 1e-4);


fprintf('\n\nnow test deltaU constraints...\n');
pwa_DI; probStruct.N=3; probStruct.subopt_lev = 0; probStruct.Tconstraint = 0;
probStruct.norm = 1;
sysStruct.xmax = sysStruct.ymax;
sysStruct.xmin = sysStruct.ymin;
sysStruct.dumax = 0.4;
sysStruct.dumin = -0.3;
[C,O,V]=mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl=mpt_ownmpc(sysStruct,probStruct,C,O,V,'online');
[x,u,y,c] = sim(ctrl, [1;1], 6);
mbg_asserttrue(max(diff(u)) <= sysStruct.dumax+sqrt(eps));
mbg_asserttrue(min(diff(u)) >= sysStruct.dumin-sqrt(eps));
mbg_asserttolequal(c, 14.2, 1e-4);


fprintf('\n\nnow test probStruct.Rdu...\n'); 
% we should automatically switch to deltaU formulation
pwa_DI; probStruct.N=3; probStruct.subopt_lev = 0; probStruct.Tconstraint = 0;
probStruct.norm = 1;
sysStruct.xmax = sysStruct.ymax;
sysStruct.xmin = sysStruct.ymin;
probStruct.Rdu = 1;
[C,O,V]=mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl=mpt_ownmpc(sysStruct,probStruct,C,O,V,'online');
[x,u,y,c] = sim(ctrl, [1;1], 3);
mbg_asserttrue(isfield(V, 'uprev') | length(V.x{1})==3); % this is an indicator that we take Rdu into account
mbg_asserttolequal(c, 6.3333, 1e-4);


fprintf('\n\nnow test tracking with deltaU constraints...\n');
reference = [1; 0];
pwa_DI; probStruct.N=3; probStruct.subopt_lev = 0; probStruct.Tconstraint = 0;
probStruct.norm = 1;
sysStruct.xmax = sysStruct.ymax; sysStruct.xmin = sysStruct.ymin;
sysStruct.dumax = 0.4;
sysStruct.dumin = -0.3;
probStruct.tracking = 1;
[C,O,V]=mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl=mpt_ownmpc(sysStruct,probStruct,C,O,V,'online');
[x,u,y,cost] = sim(ctrl, [1;1], 6, struct('reference', reference));
mbg_asserttrue(max(diff(u)) <= sysStruct.dumax+sqrt(eps));
mbg_asserttrue(min(diff(u)) >= sysStruct.dumin-sqrt(eps));
mbg_asserttolequal(cost, 6.46, 1e-2);


fprintf('\n\nnow test tracking=1 with probStruct.Rdu...\n');
reference = [1; 0];
pwa_DI; probStruct.N=3; probStruct.subopt_lev = 0; probStruct.Tconstraint = 0;
probStruct.norm = 1;
probStruct.tracking = 1;
probStruct.Rdu = 0.1;
sysStruct.dumax = 10; sysStruct.dumin = -10;
sysStruct.xmax = sysStruct.ymax; sysStruct.xmin = sysStruct.ymin;
sysStruct.xrefmax = sysStruct.xmax; sysStruct.xrefmin = sysStruct.xmin;
sysStruct.yrefmax = sysStruct.ymax; sysStruct.yrefmin = sysStruct.ymin;
[C,O,V]=mpt_ownmpc(sysStruct, probStruct, 'online');
mbg_asserttrue(isfield(V, 'uprev') | length(V.x{1})==5); % this is an indicator that we take Rdu into account
mbg_asserttrue(isfield(V, 'ref') | length(V.x{1})==5); % this is an indicator that we take tracking=1 into account
ctrl=mpt_ownmpc(sysStruct,probStruct,C,O,V,'online');
[x,u,y,c] = sim(ctrl, [1;1], 5, struct('reference', reference));
mbg_asserttolequal(c, 1.9688, 1e-4);
