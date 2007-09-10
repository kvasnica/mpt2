function mpt_mplp_test12

mpt_options('qpsolver', 'nag');

% following error is obtained as of 23-Nov-2005 with NAG:
%
% XXX  THE BOUNDS ON  LNCON  2  ARE INCONSISTENT.   BL =  -0.1000000E+09   BU =  -0.2784529E+09
% 
% XXX  THE BOUNDS ON  LNCON  8  ARE INCONSISTENT.   BL =  -0.1000000E+09   BU =  -0.1357685E+09
% 
% XXX  THE BOUNDS ON  LNCON  9  ARE INCONSISTENT.   BL =  -0.1000000E+09   BU =  -0.1387925E+09
% 
% XXX  THE BOUNDS ON  LNCON 10  ARE INCONSISTENT.   BL =  -0.1000000E+09   BU =  -0.3074565E+09
% 
% XXX  THE BOUNDS ON  LNCON 11  ARE INCONSISTENT.   BL =  -0.1000000E+09   BU =  -0.3023004E+09
% ??? Error using ==> mpt_solveqp
% mpt_solveQP: An input parameter is invalid.
%
% this is due to the problem being unbounded

load mplp_unbounded
lasterr('');
try
    Pn = mpt_mplp(Matrices);
    worked = 1;
catch
    worked = 0;
end

T = lasterr;
if worked==0 & ~isempty(findstr(T, 'mpt_solveQP: An input parameter is invalid.')),
    fprintf('Expected failure due to unboundeness.\n');
elseif ~isempty(findstr(T, 'Undefined function or method ''e04naf''')),
    fprintf('Expected failure: NAG solver not available.\n');
elseif worked==0,
    error('unkown failure.');
end
