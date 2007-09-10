function mpt_getInput_test7

% mpt_getInput should return a N*nux1 vector of NaN if the problem is
% infeasible (for on-line controllers)
%
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x/rev/1c87ceabdc21

ThirdOrder;     % 3 states, 2 inputs
N = 5;
[nx, nu] = mpt_sysStructInfo(sysStruct);
x0 = zeros(nx, 1);
probStruct.Tconstraint = 0;
probStruct.N = 5;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
C = C + (V.u{1} > sysStruct.umax+1);   % infeasible constraint
ctrl = mpt_ownmpc(sysStruct, probStruct, C, O, V, 'online');
u_cl = ctrl(x0);
u_ol = ctrl(x0, struct('openloop', 1));

mbg_assertequal(size(u_ol), [nu*N 1]);
mbg_assertequal(size(u_cl), [nu 1]);
mbg_asserttrue(all(isnan(u_ol)));
mbg_asserttrue(all(isnan(u_cl)));
