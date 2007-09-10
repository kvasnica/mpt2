function mpt_removeOverlaps_test7

% test whether we properly reject partitions with quadratic costs
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=1154cb6767f3

Double_Integrator
probStruct.N=1; 
probStruct.Tconstraint=0; 
probStruct.P_N=zeros(2); 
probStruct.norm=2;
ctrl = mpt_control(sysStruct, probStruct, struct('verbose', -1));

ctrl = set(ctrl, 'overlaps', 1);

% we now have partition with quadratic cost, mpt_removeOverlaps() should reject
% them
%fprintf('The "No quadratic cost terms are allowed" error should be given:\n');
lasterr('');
try
    a = mpt_removeOverlaps(ctrl);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr,  'Quadratic cost terms are not allowed.');
