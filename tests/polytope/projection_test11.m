function projection_test11

% keep the mplp-based method silent, let it not display any intermediate
% warnings/errors:
%

load projection_test11

lasterr('');
try
    q = projection(P, dim, struct('projection', 7));
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'PROJECTION: couldn''t compute projection, selected method failed!');
