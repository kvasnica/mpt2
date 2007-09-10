function mpt_optControl_test3

% tests noise in H/V representations

Double_Integrator_addU
probStruct.N = 2;
V = extreme(sysStruct.noise);
V = V';

% noise in H-representation
ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 19);

% full-dimensional noise in V-representation
sysStruct.noise = V;
ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 19);

% lower-dimensional noise in V-representation
sysStruct.noise = [0.1 0; -0.1 0]';
ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 15);
