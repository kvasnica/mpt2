function mpt_plotPartition_test2

clear sysStruct probStruct
Double_Integrator; probStruct.N = 1; probStruct.Tconstraint = 0;
ctrl = mpt_control(sysStruct, probStruct);
plot(ctrl); 
close

clear sysStruct probStruct
Double_Integrator; probStruct.N = 1; probStruct.Tconstraint = 0;
sysStruct.dumax = 1;
sysStruct.dumin = -1;
ctrl = mpt_control(sysStruct, probStruct);
plot(ctrl);
close

clear sysStruct probStruct
Double_Integrator; probStruct.N = 1; probStruct.Tconstraint = 0;
probStruct.tracking = 2;
ctrl = mpt_control(sysStruct, probStruct);
plot(ctrl); 
close

clear sysStruct probStruct
Double_Integrator; probStruct.N = 1; probStruct.Tconstraint = 0;
probStruct.tracking = 1;
ctrl = mpt_control(sysStruct, probStruct);
plot(ctrl); 
close
