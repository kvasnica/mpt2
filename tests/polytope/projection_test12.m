function projection_test12

% tests the improved heuristics:
%
% also we have had an error with verbose=2, becuase the mplp-based method was
% not described.

load projection_test11

% note that there is something wrong with CDD in this example (projection=3). it
% very often crashes matlab.
Q = projection(P, dim, struct('verbose', 2, 'projection', [4 7 6]));
