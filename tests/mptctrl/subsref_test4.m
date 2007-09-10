function subsref_test4
% SKIP

return

% [ctrl.Fi{:}] returned only one element while it should return all
%
% however we cannot fix it due to a known matlab bug:
% http://www.mathworks.com/support/solutions/data/1-1ABOD.html?solution=1-1ABOD

load ctrl1
Fi = [ctrl.Fi{:}];
mbg_assertequal(length(Fi), dimension(ctrl.Pn)*length(ctrl.Pn));
    
