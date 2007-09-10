function reduce_test1

% if there are no feasible constraints, reduce() should throw an error:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=9be079edfd3d


% note that if the third input argument is 0 (default), we throw an error at the
% begining of reduce(), since K will contain Inf terms.
%
% if third input argument is 1, we have no constraints left
try
    P = polytope([0 0], 2, 1);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'REDUCE: Polytope P = R^nx');
