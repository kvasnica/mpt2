function mpt_ownmpc_test18

% YALMIP after May-12-2006 extracts bounds on variables directly from
% constraints, therefore the bounds() operator is deprecated. that's why this
% test is skipped
disp('Test no longer necessary...');
return


global mptOptions

% tests that we properly set bounds on slack variables:
%

Double_Integrator
probStruct.Tconstraint = 0;
probStruct.P_N = zeros(2);
sysStruct.xmax = sysStruct.ymax;
sysStruct.xmin = sysStruct.ymin;

probStruct.symax = [1;1];
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
for k = 1:length(V.sy),
    b = yalmip('getbounds', getvariables(V.sy{k}));
    mbg_assertequal(b, [0 1; 0 1]);
end

probStruct.sumax = 2;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
for k = 1:length(V.su),
    b = yalmip('getbounds', getvariables(V.su{k}));
    mbg_assertequal(b, [0 2]);
end

probStruct.sxmax = [3;4];
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
for k = 1:length(V.sx),
    b = yalmip('getbounds', getvariables(V.sx{k}));
    mbg_assertequal(b, [0 3; 0 4]);
end

probStruct = rmfield(probStruct, 'sxmax');
probStruct.Sx = 1;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
for k = 1:length(V.sx),
    b = yalmip('getbounds', getvariables(V.sx{k}));
    mbg_assertequal(b, [0 mptOptions.infbox; 0 mptOptions.infbox]);
end

probStruct = rmfield(probStruct, 'symax');
probStruct.Sy = 1;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
for k = 1:length(V.sy),
    b = yalmip('getbounds', getvariables(V.sy{k}));
    mbg_assertequal(b, [0 mptOptions.infbox; 0 mptOptions.infbox]);
end

probStruct = rmfield(probStruct, 'sumax');
probStruct.Su = 1;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
for k = 1:length(V.su),
    b = yalmip('getbounds', getvariables(V.su{k}));
    mbg_assertequal(b, [0 mptOptions.infbox]);
end
