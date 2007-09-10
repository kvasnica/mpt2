function mpt_yalmipTracking_test1

% tests that we immediately exit if probStruct.tracking=0 is given:
%

Double_Integrator
probStruct.tracking = 0;
[sst, pst] = mpt_yalmipTracking(sysStruct, probStruct);

mbg_assertfalse(isfield(pst, 'tracking_augmented'));
