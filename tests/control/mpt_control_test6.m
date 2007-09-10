function mpt_control_test6

% tests LTI systems w/ parametric and additive uncertainties

% parametric uncertainty
clear sysStruct probStruct
Double_Integrator_parU
sysStruct.noise = polytope;
ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 35);

% additive uncertainty
clear sysStruct probStruct
Double_Integrator_addU
ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 25);

% parametric and additive uncertainty
clear sysStruct probStruct
Double_Integrator_parU
sysStruct.noise = unitbox(2, 0.01);
ctrl = mpt_control(sysStruct, probStruct)
mbg_assertequal(length(ctrl), 39);
