function mpt_owncost_test1

% tests polytopic constraints

[H, K] = double(unitbox(2, 2));   % polytopic constraints
Double_Integrator
mpt_ownmpc(sysStruct, probStruct);
for k = 1:length(VAR.x)
    % k==1 corresponds to x0, k==2 to x1 etc.
    CON = CON + set(H*VAR.x{k} <= K);
end
ctrl = mpt_ownmpc(sysStruct, probStruct, CON, OBJ, VAR);
mbg_assertequal(length(ctrl), 11);

% test the same with an on-line controller
[H, K] = double(unitbox(2, 2));   % polytopic constraints
Double_Integrator
mpt_ownmpc(sysStruct, probStruct, 'online');
for k = 1:length(VAR.x)
    % k==1 corresponds to x0, k==2 to x1 etc.
    CON = CON + set(H*VAR.x{k} <= K);
end
ctrl = mpt_ownmpc(sysStruct, probStruct, CON, OBJ, VAR, 'online');
