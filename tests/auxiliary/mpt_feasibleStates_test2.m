function mpt_feasibleStates_test1

% test tracking and deltaU controllers, dimension of datapoints should be 2

%----------------------------------------------------------
% tracking 1
clear sysStruct probStruct
Double_Integrator;
probStruct.Tconstraint = 0;
probStruct.N = 2;
probStruct.tracking = 1;
ctrl = mpt_control(sysStruct, probStruct);
x = mpt_feasibleStates(ctrl, 10);
mbg_assertequal(size(x), [80 2]);

%----------------------------------------------------------
% tracking 2
clear sysStruct probStruct
Double_Integrator;
probStruct.Tconstraint = 0;
probStruct.N = 2;
probStruct.tracking = 2;
ctrl = mpt_control(sysStruct, probStruct);
x = mpt_feasibleStates(ctrl, 10);
mbg_assertequal(size(x), [80 2]);

%----------------------------------------------------------
% deltaU constraints
clear sysStruct probStruct
Double_Integrator;
probStruct.Tconstraint = 0;
probStruct.N = 2;
sysStruct.dumax = 1; sysStruct.dumin = -1;
ctrl = mpt_control(sysStruct, probStruct);
x = mpt_feasibleStates(ctrl, 10);
mbg_assertequal(size(x), [80 2]);
