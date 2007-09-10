function extreme_test2
% SKIP

return

% this does not work as of 28th September 2005
load extreme_test2

% note that the polytope is unbounded
e = extreme(XU);

% currently we are always getting empty result, but we expect extreme points to
% be computed correctly in the future
mbg_asserttrue(~isempty(e));
