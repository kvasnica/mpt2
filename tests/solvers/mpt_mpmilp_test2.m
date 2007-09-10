function mpt_mpmilp_test2

% test that mpt_mplp and mpt_mpmilp return the same solution if all variables
% are continuous

Double_Integrator
probStruct.norm = 1; probStruct.N = 2;
M=mpt_constructMatrices(sysStruct,probStruct);

% mplp
[Pn,Fi,Gi,AC,Ph,D]=mpt_mplp(M);

% mpMILP
M.vartype = repmat('C', size(M.G, 2), 1);
S = mpt_mpmilp(M);

mbg_assertequal(length(Pn), 28);
mbg_assertequal(length(Pn), length(S.Pn));

% mpt_mpmilp and mpt_mplp can return regions in different order. here we detect
% which region of S.Pn corresponds to which region of Pn
E = eq(Pn, S.Pn, struct('elementwise',1));
map = [];
for ii = 1:length(Pn),
    map = [map; ii find(E(ii, :))];
end

% test that optimizer and cost are identical in both solutions
for ii = 1:length(Pn),
    mbg_asserttolequal(Fi{ii}, S.Fi{map(ii, 2)});
    mbg_asserttolequal(Gi{ii}, S.Gi{map(ii, 2)});
    mbg_asserttolequal(D.Bi{ii}, S.Bi{map(ii, 2)});
    mbg_asserttolequal(D.Ci{ii}, S.Ci{map(ii, 2)});
end
