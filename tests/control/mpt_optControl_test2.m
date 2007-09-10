function mpt_optControl_test2

% test various control strategies for LTI systems w/ 2-norm


%=========================================================

% tracking=0, dumode=0
[sysStruct, probStruct] = di_data;
ctrl_tr0_du0 = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl_tr0_du0), 7);


% tracking=0, dumode=0, yref
[sysStruct, probStruct] = di_data;
probStruct.yref = 1;
probStruct.Qy = 1;
ctrl_tr0_du0_yref = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl_tr0_du0_yref), 2);

% tracking=0, dumode=0, xref
[sysStruct, probStruct] = di_data;
probStruct.xref = [1;0];
ctrl_tr0_du0_xref = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl_tr0_du0_xref), 9);

%=========================================================

% tracking=0, dumode=1
[sysStruct, probStruct] = di_data;
sysStruct.dumax = 1; sysStruct.dumin = -1;
ctrl_tr0_du1 = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl_tr0_du1), 11);
mbg_assertequal(dimension(ctrl_tr0_du1.Pn), 3)

% tracking=0, dumode=1, yref
[sysStruct, probStruct] = di_data;
probStruct.yref = 1;
probStruct.Qy = 1;
sysStruct.dumax = 1; sysStruct.dumin = -1;
ctrl_tr0_du1_yref = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl_tr0_du1_yref), 4);

% tracking=0, dumode=1, xref
[sysStruct, probStruct] = di_data;
probStruct.xref = [1;0];
sysStruct.dumax = 1; sysStruct.dumin = -1;
ctrl_tr0_du1_xref = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl_tr0_du1_xref), 16);

%=========================================================

% probStruct.tracking = 1
[sysStruct, probStruct] = di_data;
probStruct.tracking = 1;
probStruct.Qy = 1;
ctrl_tr1 = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl_tr1), 5);
mbg_assertequal(dimension(ctrl_tr1.Pn), 4)

% probStruct.tracking = 2
[sysStruct, probStruct] = di_data;
probStruct.tracking = 2;
probStruct.Qy = 1;
ctrl_tr2 = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl_tr2), 5);
mbg_assertequal(dimension(ctrl_tr2.Pn), 3)


%save ctrl_data ctrl_tr0_du0 ctrl_tr0_du0_yref ctrl_tr0_du0_xref ctrl_tr0_du1 ctrl_tr0_du1_yref ctrl_tr0_du1_xref ctrl_tr1 ctrl_tr2


%----------------------------------------------------------
function [sysStruct, probStruct] = di_data

Double_Integrator
probStruct.N = 2;
sysStruct.C = [1 0];
sysStruct.D = 0;
sysStruct.ymax = 2;
sysStruct.ymin = - 2;
