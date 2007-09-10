function mpt_owncost_test10

Double_Integrator
probStruct.N = 1;
probStruct.norm = 1;
sysStruct.xmax = [5; 5];
sysStruct.xmin = [-5; -5];

Pavoid = unitbox(2, 0.3);
[Ha, Ka] = double(Pavoid);

[C, O, V] = mpt_ownmpc(sysStruct,probStruct);
for k = 1:length(V.x),
    d = binvar(1,1);
    C = C + set(iff(d, Ha*V.x{k}<=Ka));
    O = O + 10*d;
end

ctrl = mpt_ownmpc(sysStruct, probStruct, C, O, V);
