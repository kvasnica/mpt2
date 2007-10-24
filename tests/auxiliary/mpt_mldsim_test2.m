function mpt_mldsim_test2

% tests mpt_mldsim with no "z" and "d" variables, but with ne>0

sysStruct = mpt_sys('mpt_mldsim_test2_hys', 'nopwa');
S = sysStruct.data.MLD;

% x0 infeasible
u = 1;
x0 = [0.1; 0.3];
[xn, y, z, d, f] = mpt_mldsim(S, x0, u);
mbg_assertfalse(f);

% x0 feasible
u = 1;
x0 = [-0.1; 0.3];
[xn, y, z, d, f] = mpt_mldsim(S, x0, u);
mbg_asserttrue(f);
mbg_asserttolequal(xn, [1.2; 0.2]);
