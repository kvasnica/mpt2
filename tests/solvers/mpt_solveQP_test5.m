function mpt_solveQP_test5

% tests that we properly handle quadratic terms which are not symmetric
% (causes a nasty warning in quadprog)
%

f = [1 1];
A = [eye(2); -eye(2)];
B = ones(4, 1);

% problem only occurs in quadprog
solver = 1;

fprintf('No warning should be issued:\n');
% a slightly non-symmetric hessian
H = [1 1e-15; 0 1];
lastwarn('');
mpt_solveQP(H, f, A, B, [], [], [], solver);
% quadprog should not give any warnings now
mbg_asserttrue(isempty(lastwarn));


fprintf('\n\nWarning should be issued by quadprog:\n\n');
% a clearly non-symmetric hessian => quadprog should give a warning
H = [1 1; 0 1];
lastwarn('');
mpt_solveQP(H, f, A, B, [], [], [], solver);
% quadprog should not give any warnings now
mbg_assertfalse(isempty(lastwarn));
