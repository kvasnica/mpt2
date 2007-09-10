function projection_issue387

% without a patch, mex fourier would crash matlab because it is called with
% an empty matrix (catched properly by the patch):
%
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x/rev/02cffaa1088d

load projection_issue387
proj = projection(P, 1:nx, struct('projection', 5));
mbg_asserttrue(isfulldim(proj));
% try
%     proj = projection(P, 1:nx, struct('projection', 5))
%     worked = 1;
% catch
%     worked = 0;
% end
% mbg_assertfalse(worked)


proj = projection(P, 1:nx, struct('projection', 6));
mbg_asserttrue(isfulldim(proj))
