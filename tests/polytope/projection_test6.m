function projection_test6
% SKIP

return

% test matlab fourier-motzkin (fails as of 21-Nov-2005)

% for this particural polytope, method 1 (matlab fourier-motzkin) and 5 (C-code
% fourier-motzkin) give different results than other method. the difference is
% very small, though.
%
% with mptOptions.abs_tol=1e-7 both method 1 and method 5 fail.
% with mptOptions.abs_tol=1e-8 only method 5 fails - but removing randomization
%                                                    fixes it!!!
% does it have something to do with reduce()?

load projection_S1_S5_fail

% project on 3rd and 1st dimension, we must preserve this order!
dim = [3 1];

% use vertex enumeration + convex hull -> this is the simplest method, we can't
% do anything wrong there
Q0 = projection(P, dim, struct('projection', 0));

% test matlab fourier-motzkin
Q = projection(P, dim, struct('projection', 1));

areequal = (Q==Q0);
try
    mbg_asserttrue(areequal);
catch
    error('Expected Fourier-motzkin (matlab version) failure.');
end
