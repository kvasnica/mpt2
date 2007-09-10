function mpt_mldsim_test1

% 

sysStruct = mpt_sys('turbo_car', 'nopwa');
S = sysStruct.data.MLD;
u = [0.5; 1];
x0 = [0; 0; 5];

xn = mpt_mldsim(S, x0, u);
mbg_asserttolequal(xn, [1; 0.5; 4]);
