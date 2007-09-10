function mpt_mpmiqp_test2

% test that mpt_mpqp and mpt_mpmiqp return the same solution if all variables
% are continuous

Double_Integrator
M=mpt_constructMatrices(sysStruct, probStruct);

% mpqp
[Pn,Fi,Gi,AC,Ph,D]=mpt_mpqp(M);

% mpMIQP
M.vartype = repmat('C', size(M.G, 2), 1);
S = mpt_mpmiqp(M);

mbg_assertequal(length(Pn), 25);
mbg_assertequal(length(Pn), length(S.Pn));

% mpt_mpmiqp and mpt_mpqp can return regions in different order. here we detect
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
    mbg_asserttolequal(D.Ai{ii}, S.Ai{map(ii, 2)});
    mbg_asserttolequal(D.Bi{ii}, S.Bi{map(ii, 2)});
    mbg_asserttolequal(D.Ci{ii}, S.Ci{map(ii, 2)});
end
