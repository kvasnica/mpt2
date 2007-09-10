function mpt_verifySysStruct_test3

% tests that we do not require output constraints if we have state constraints
%

Double_Integrator
sysStruct.xmax = sysStruct.ymax;
sysStruct.xmin = sysStruct.ymin;
sysStruct = rmfield(sysStruct, 'ymax');
sysStruct = rmfield(sysStruct, 'ymin');

ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 25);
