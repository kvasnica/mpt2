function mpt_mplp_test16
% SKIP

return

% unbounded mplp does not work as of 25.4.2006
% requires NAG?

mat = [];
load mplp_unbounded2
M = mat;

lasterr('');
try
    Pn = mpt_mplp(M);
    worked = 1;
catch
    worked = 0;
end

T = lasterr;
if worked==0,
    error(T);
end

