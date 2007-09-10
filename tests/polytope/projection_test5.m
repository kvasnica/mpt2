function projection_test5

% test mex fourier motzkin

% for this particural polytope, method 1 (matlab fourier-motzkin) and 5 (C-code
% fourier-motzkin) give different results than other method. the difference is
% very small, though.
%
% with mptOptions.abs_tol=1e-7 both method 1 and method 5 fail.
% with mptOptions.abs_tol=1e-8 only method 5 fails - but removing randomization
%                                                    fixes it!!!

% following patch fixes it:
% 

load projection_S1_S5_fail

% project on 3rd and 1st dimension, we must preserve this order!
dim = [3 1];

% use vertex enumeration + convex hull -> this is the simplest method, we can't
% do anything wrong there
Q0 = projection(P, dim, struct('projection', 0));

% test C-code fourier motzkin several times (because we do randomization inside)
QQ = [];
for run = 1:5,
    Q = projection(P, dim, struct('projection', 5));
    % all projections must be equal 
    [5 Q==Q0]
end
mbg_asserttrue(Q==Q0);
