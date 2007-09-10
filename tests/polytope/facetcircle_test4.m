function facetcircle_test4

% tests that we can cope with a problem where we explore several facets and for
% some of them there are errors:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=edc96f2c1fc3

load facetcircle_prob

[X,R] = facetcircle(P, 1:10, struct('abs_tol', 1e-9));

% for this polytope and this particular value of absolute tolerance, center of
% the 3d facet violates constraints. but we should continue further and
% correctly compute centers of remaining facets.
%
% without the patch we would return NaN for facets 3-10:
%    -0.0399   -0.0399       NaN       NaN       NaN       NaN       NaN       NaN       NaN       NaN
%    -0.0392   -0.0166       NaN       NaN       NaN       NaN       NaN       NaN       NaN       NaN
%     0.0398    0.0398       NaN       NaN       NaN       NaN       NaN       NaN       NaN       NaN
%     0.9515    1.1242       NaN       NaN       NaN       NaN       NaN       NaN       NaN       NaN
%    -0.3167   -0.1592       NaN       NaN       NaN       NaN       NaN       NaN       NaN       NaN
%
% with the patch, only 3d facet remains NaN:
%    -0.0399   -0.0399       NaN    0.0283    0.0039    0.0039    0.0767   -1.0900    0.0067   -1.0892
%    -0.0392   -0.0166       NaN   -0.0555   -0.0150   -0.0438   -0.0953   -0.0177   -0.0465   -0.0251
%     0.0398    0.0398       NaN    1.0284   -0.0039    0.3755    0.7795    1.0900   -0.0067    1.0892
%     0.9515    1.1242       NaN   -0.0283    0.3879    1.0871    1.8043    0.3998    1.5798    0.6413
%    -0.3167   -0.1592       NaN   -0.1609   -0.2319   -0.1417   -0.5747   -0.0585   -0.3298   -0.0614

mbg_assertfalse(any(any(isnan(X(:, 4:end)))));
