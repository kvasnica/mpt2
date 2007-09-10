function mpt_yalmipcftoc_test1

% tests that we properly reject cases with tracking=1 and boolean inputs even
% for on-line controllers

Double_Integrator
probStruct.tracking = 1;
sysStruct.Uset = [1 0];

lasterr('');
try
    ctrl = mpt_control(sysStruct, probStruct, 'online');
    worked = 1;
catch
    worked = 0;
end
disp(lasterr);
mbg_assertfalse(worked);
