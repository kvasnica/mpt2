function mpt_mpmiqp_test1

% test binary inputs
lti1d
probStruct.N = 3; probStruct.norm = 2; probStruct.Tconstraint=0;
M = mpt_constructMatrices(sysStruct, probStruct);
Pfeas = projection(polytope([-M.E M.G], M.W), 1:size(M.E, 2));  % feasible set
M.vartype = repmat('C', size(M.G, 2), 1);
M.vartype(1:2) = 'B';
S = mpt_mpmiqp(M);
mbg_assertequal(length(S.Pn), 12);
mbg_asserttrue(S.Pn == Pfeas);


% test inputs from finite alphabet
lti1d
probStruct.N = 3; probStruct.norm = 2; probStruct.Tconstraint=0;
M = mpt_constructMatrices(sysStruct, probStruct);
Pfeas = projection(polytope([-M.E M.G], M.W), 1:size(M.E, 2));  % feasible set
M.vartype = repmat('C', size(M.G, 2), 1);
M.vartype(1:2) = 'A';
M.alphabet{1} = [-1 0 1];
M.alphabet{2} = [0 0.5];
S = mpt_mpmiqp(M);
mbg_assertequal(length(S.Pn), 18);
mbg_asserttrue(S.Pn == Pfeas);


% mixed binary/alphabet inputs
lti1d
probStruct.N = 3; probStruct.norm = 2; probStruct.Tconstraint=0;
M = mpt_constructMatrices(sysStruct, probStruct);
Pfeas = projection(polytope([-M.E M.G], M.W), 1:size(M.E, 2));  % feasible set
M.vartype = repmat('C', size(M.G, 2), 1);
M.vartype(1) = 'B';
M.vartype(2) = 'A';
M.alphabet{2} = [0 0.5];
S = mpt_mpmiqp(M);
mbg_assertequal(length(S.Pn), 12);
mbg_asserttrue(S.Pn == Pfeas);



% 1D system, all inputs binary - yalmip had a problem with this one
lti1d
probStruct.N = 3; probStruct.norm = 2; probStruct.Tconstraint=0;
M = mpt_constructMatrices(sysStruct, probStruct);
Pfeas = projection(polytope([-M.E M.G], M.W), 1:size(M.E, 2));  % feasible set
M.vartype = repmat('B', size(M.G, 2), 1);
S = mpt_mpmiqp(M);
mbg_assertequal(length(S.Pn), 8);
mbg_asserttrue(S.Pn == Pfeas);


% 2D system, all inputs from finite alphabet
Double_Integrator
probStruct.N = 2; probStruct.norm = 2; probStruct.Tconstraint=0;
M = mpt_constructMatrices(sysStruct, probStruct);
Pfeas = projection(polytope([-M.E M.G], M.W), 1:size(M.E, 2));  % feasible set
M.vartype = repmat('A', size(M.G, 2), 1);
for ii = 1:size(M.G, 2),
    M.alphabet{ii} = [-1 0 1];
end
S = mpt_mpmiqp(M);
mbg_assertequal(length(S.Pn), 9);
mbg_asserttrue(S.Pn == Pfeas);
