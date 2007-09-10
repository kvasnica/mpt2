function mpt_mpqp_test8

% tests that we properly display a warning when the mpQP is not positive
% definite:
%

M.H = -2;           % negative definite mpQP
M.F = 0; M.Y = 0; M.Cf = 2; M.Cx = 0; M.Cc = -1;
M.G = [-1; 1; 0; 0];
M.W = [5; 5; 0; 1];
M.E = [1; 1; 1; -1];
M.bndA = []; M.bndb = [];

fprintf('Expect warning that the mpQP is not positive definite:\n\n');
Pn = mpt_mpqp(M);

fprintf('\n\nExpect warning that the mpQP close to being negative definite:\n\n');
M.H = 1e-6;
Pn = mpt_mpqp(M);


fprintf('\n\nExpect warning that the mpQP lacks strict convexity:\n\n');
M.H = 0;
w = warning;
warning off
try
    Pn = mpt_mpqp(M);
end
warning(w);
