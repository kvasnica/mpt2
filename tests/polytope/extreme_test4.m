function extreme_test4

% test extreme points enumeration on some polytopes for which the following test
% fails:
%   hull(extreme(P(ii))) ~= P(ii)

clear
load extreme_test4

notgood = [];
tic
for ii = 1:length(P),
    E = extreme(P(ii), struct('extreme_solver', 3, 'roundat', Inf));
    H = hull(E);
    %areequal = ~isfulldim(H\P(ii)) & ~isfulldim(P(ii)\H);
    
    % we need higher tolerance when testing equality, otherwise we run into
    % numerical problems
    areequal = eq(H, P(ii), struct('abs_tol', 1e-5));
    if ~areequal,
        notgood = [notgood ii];
    end
end
toc
notgood
