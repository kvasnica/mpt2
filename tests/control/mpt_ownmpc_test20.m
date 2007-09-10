function mpt_ownmpc_test20

% tests that we properly convert empty ctrl.Ai{r} fields to zero matrices
% of appropriate dimension. If probStruct.norm=2 was used but the objective
% was set to zero manually, such change was not performed until this patch:
%
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x/rev/1e4875db9eac

Double_Integrator
probStruct.N = 2;
probStruct.Tconstraint = 0;


%--------------------------------------------------------------
% 2-norm in probStruct, zero objective
probStruct.norm = 2;
[F, obj, V] = mpt_ownmpc(sysStruct, probStruct);
obj = 0;
ctrl = mpt_ownmpc(sysStruct, probStruct, F, obj, V);
for i = 1:length(ctrl.Ai)
    mbg_assertfalse(isempty(ctrl.Ai{i}));
end


%--------------------------------------------------------------
% 1-norm in probStruct, zero objective
probStruct.norm = 1;
[F, obj, V] = mpt_ownmpc(sysStruct, probStruct);
obj = 0;
ctrl = mpt_ownmpc(sysStruct, probStruct, F, obj, V);
for i = 1:length(ctrl.Ai)
    mbg_assertfalse(isempty(ctrl.Ai{i}));
end


%--------------------------------------------------------------
% 1-norm in probStruct, non-zero objective
probStruct.norm = 1;
[F, obj, V] = mpt_ownmpc(sysStruct, probStruct);
ctrl = mpt_ownmpc(sysStruct, probStruct, F, obj, V);
for i = 1:length(ctrl.Ai)
    mbg_assertfalse(isempty(ctrl.Ai{i}));
end


%--------------------------------------------------------------
% 2-norm in probStruct, non-zero objective, quadratic terms should be
% preserved
probStruct.norm = 2;
[F, obj, V] = mpt_ownmpc(sysStruct, probStruct);
ctrl = mpt_ownmpc(sysStruct, probStruct, F, obj, V);
for i = 1:length(ctrl.Ai)
    mbg_assertfalse(isempty(ctrl.Ai{i}));
    mbg_assertfalse(any(any(ctrl.Ai{i}==0)));
end

