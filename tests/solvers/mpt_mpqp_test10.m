function mpt_mpqp_test10

% http://control.ee.ethz.ch/~mpt/bugzilla/show_bug.cgi?id=68

clear sysStruct probStruct
sysStruct.A = eye(8); sysStruct.A(2, 1) = 0.1; sysStruct.A(8, 1) = -1.5;
sysStruct.B = ones(8, 1);
sysStruct.C = eye(1, 8);
sysStruct.D = 0;
sysStruct.ymax = 100;
sysStruct.ymin = -100;
sysStruct.umax = .1;
sysStruct.umin = -.1;

probStruct.Q = eye(8);
probStruct.R = 100;
probStruct.P_N = probStruct.Q;
probStruct.Tconstraint = 0;
probStruct.norm = 2;
probStruct.N = 5;

M = mpt_constructMatrices(sysStruct, probStruct);
Pn = mpt_mpqp(M);
% R13 - 299 regions, R2006b - 429 regions
mbg_asserttrue(length(Pn) > 290);

% Pn should have no empty regions
[x, r] = chebyball(Pn);
mbg_asserttrue(isempty(find(isinf(r))));
