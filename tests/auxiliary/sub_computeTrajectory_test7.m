function sub_computeTrajectory_test7

% tests whether we fixed the problem when Options.sysStruct is used and the
% controller was designed either with tracking or with deltaU constraints (then
% number of states of Options.sysStruct does not correspond to number of states
% of the controller):
%

Double_Integrator;
load ctrl_data
ctrl = ctrl_tr2;

x = sub_computeTrajectory(ctrl_tr2, [0.1; 0.1], 2, struct('sysStruct', sysStruct, 'reference', [1]));
x = sub_computeTrajectory(ctrl_tr1, [0.1; 0.1], 2, struct('sysStruct', sysStruct, 'reference', [1]));
x = sub_computeTrajectory(ctrl_tr0_du1, [0.1; 0.1], 2, struct('sysStruct', sysStruct));
