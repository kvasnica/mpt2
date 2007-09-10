function isinvariant_test2

Double_Integrator
probStruct.N = 1;
ctrl = mpt_control(sysStruct, probStruct)
a = isinvariant(ctrl);
mbg_assertequal(a, 1);


opt_sincos
probStruct.N = 1;
probStruct.norm = 2;
ctrl = mpt_control(sysStruct, probStruct)
a = isinvariant(ctrl);
mbg_assertequal(a, 1);
