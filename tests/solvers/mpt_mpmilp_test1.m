function mpt_mpmilp_test1

% test binary inputs
lti1d
probStruct.N = 2; probStruct.norm = 1;
M = mpt_constructMatrices(sysStruct, probStruct);
M.vartype = repmat('C', size(M.G, 2), 1);
M.vartype(1:2) = 'B';
S = mpt_mpmilp(M);

mbg_assertequal(length(S.Pn), 4);
Pfeas = projection(polytope([-M.E M.G], M.W), 1:size(M.E, 2));  % feasible set
mbg_asserttrue(S.Pn == Pfeas);




% test inputs from finite alphabet
lti1d
probStruct.N = 2; probStruct.norm = 1;
M = mpt_constructMatrices(sysStruct, probStruct);
M.vartype = repmat('C', size(M.G, 2), 1);
M.vartype(1:2) = 'A';
M.alphabet{1} = [-1 0 1];
M.alphabet{2} = [0 0.5];
S = mpt_mpmilp(M);

mbg_assertequal(length(S.Pn), 6);
Pfeas = projection(polytope([-M.E M.G], M.W), 1:size(M.E, 2));  % feasible set
mbg_asserttrue(S.Pn == Pfeas);



% mixed binary/alphabet inputs
lti1d
probStruct.N = 2; probStruct.norm = 1;
M = mpt_constructMatrices(sysStruct, probStruct);
M.vartype = repmat('C', size(M.G, 2), 1);
M.vartype(1) = 'B';
M.vartype(2) = 'A';
M.alphabet{2} = [0 0.5];
S = mpt_mpmilp(M);

mbg_assertequal(length(S.Pn), 4);
Pfeas = projection(polytope([-M.E M.G], M.W), 1:size(M.E, 2));  % feasible set
mbg_asserttrue(S.Pn == Pfeas);
