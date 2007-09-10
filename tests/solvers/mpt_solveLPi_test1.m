function mpt_solveLPi_test1

% tests handling of sparse arguments by mpt_solveLPi as by Bug 35:
% http://control.ee.ethz.ch/~mpt/bugzilla/show_bug.cgi?id=35

A = sparse([eye(5); -eye(5)]);
B = ones(10, 1);
solver = 3;   % CDD

[a,b,c,d,e]=mpt_solveLPi(eye(1, size(A, 1)), A, B, [], [], [], solver);
