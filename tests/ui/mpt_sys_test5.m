function mpt_sys_test5

opt_sincos
sysStruct.B{1} = zeros(2, 0);
sysStruct.B{2} = zeros(2, 0);
sysStruct.D{1} = zeros(2, 0);
sysStruct.D{2} = zeros(2, 0);
sysStruct.umax = [];
sysStruct.umin = [];
sysStruct.dumax = [];
sysStruct.dumin = [];

M = mpt_pwa2mld(sysStruct);
S = mpt_sys(M, 'nopwa');