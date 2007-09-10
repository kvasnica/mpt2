function extendx0_test2

Double_Integrator
probStruct.Tconstraint = 0;
x0 = [1;2];
uprev = 3;
ref = [4;5];

probStruct.tracking=0;
ctrl = mpt_control(sysStruct, probStruct, 'online');
[xn,dumode]=extendx0(ctrl, x0, uprev, ref);
mbg_assertequal(dumode, 0);
mbg_assertequal(xn, x0);

probStruct.tracking=1;
ctrl = mpt_control(sysStruct, probStruct, 'online');
[xn,dumode]=extendx0(ctrl, x0, uprev, ref);
mbg_assertequal(dumode, 1);
mbg_assertequal(xn, [x0; uprev; ref]);


probStruct.tracking=2;
ctrl = mpt_control(sysStruct, probStruct, 'online');
[xn,dumode]=extendx0(ctrl, x0, uprev, ref);
mbg_assertequal(dumode, 0);
mbg_assertequal(xn, [x0; ref]);
