function mpt_ownmpc_test16

% tests that we always add constraints on x_N
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=0b72c666d2fa

Double_Integrator
probStruct.Tconstraint = 0;
probStruct.P_N = zeros(2);
sysStruct.xmax = sysStruct.ymax;
sysStruct.xmin = sysStruct.ymin;

% by default, probStruct.{x0bounds|y0bounds|xNbounds} should all be enabled
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
mbg_asserttrue(sub_isconstraint(C, 'xmin < x_5 < xmax'));
mbg_asserttrue(sub_isconstraint(C, 'xmin < x_4 < xmax'));
mbg_asserttrue(sub_isconstraint(C, 'xmin < x_3 < xmax'));
mbg_asserttrue(sub_isconstraint(C, 'xmin < x_2 < xmax'));
mbg_asserttrue(sub_isconstraint(C, 'xmin < x_1 < xmax'));
mbg_asserttrue(sub_isconstraint(C, 'xmin < x_0 < xmax'));
mbg_asserttrue(sub_isconstraint(C, 'ymin < y_0 < ymax'));
mbg_asserttrue(sub_isconstraint(C, 'ymin < y_1 < ymax'));
mbg_asserttrue(sub_isconstraint(C, 'ymin < y_2 < ymax'));
mbg_asserttrue(sub_isconstraint(C, 'ymin < y_3 < ymax'));
mbg_asserttrue(sub_isconstraint(C, 'ymin < y_4 < ymax'));

% now switch off constraints on y_0
probStruct.y0bounds = 0;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
mbg_asserttrue(sub_isconstraint(C, 'xmin < x_5 < xmax'));
mbg_asserttrue(sub_isconstraint(C, 'xmin < x_4 < xmax'));
mbg_asserttrue(sub_isconstraint(C, 'xmin < x_3 < xmax'));
mbg_asserttrue(sub_isconstraint(C, 'xmin < x_2 < xmax'));
mbg_asserttrue(sub_isconstraint(C, 'xmin < x_1 < xmax'));
mbg_asserttrue(sub_isconstraint(C, 'xmin < x_0 < xmax'));
mbg_assertfalse(sub_isconstraint(C, 'ymin < y_0 < ymax'));
mbg_asserttrue(sub_isconstraint(C, 'ymin < y_1 < ymax'));
mbg_asserttrue(sub_isconstraint(C, 'ymin < y_2 < ymax'));
mbg_asserttrue(sub_isconstraint(C, 'ymin < y_3 < ymax'));
mbg_asserttrue(sub_isconstraint(C, 'ymin < y_4 < ymax'));



%-------------------------------------------------
function isthere = sub_isconstraint(F, tag)

try
    a = F(tag);
    isthere = 1;
catch
    isthere = 0;
end
