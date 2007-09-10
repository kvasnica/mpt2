function horzcat_test2

% there was a bug in polytope/horzcat causing a crash if the leading
% element was an empty polytope
%
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x/rev/dc05ac9e5ee3

C = { polytope, unitbox(2), polytope };
P = [C{:}];
mbg_assertequal(length(P), 1);
mbg_asserttrue(P == unitbox(2));


lasterr('');
try
    C = { polytope, unitbox(2), polytope, unitbox(1) };
    P = [C{:}];
    worked = 1;
catch
    worked = 0;
end
disp(lasterr);
mbg_assertfalse(worked);
