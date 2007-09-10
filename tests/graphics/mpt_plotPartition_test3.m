function mpt_plotPartition_test3

% http://control.ee.ethz.ch/~mpt/hg/mpt-26x/rev/49ead19a7b44

clear sysStruct probStruct
FourthOrder; probStruct.N = 1; probStruct.Tconstraint = 0;
ctrl = mpt_control(sysStruct, probStruct);
plot(ctrl); 
close

clear sysStruct probStruct
FourthOrder; probStruct.N = 1; probStruct.Tconstraint = 0;
sysStruct.dumax = [10; 10];
sysStruct.dumin = [-10; -10];
ctrl = mpt_control(sysStruct, probStruct);
plot(ctrl);
close

clear sysStruct probStruct
FourthOrder; probStruct.N = 1; probStruct.Tconstraint = 0;
probStruct.tracking = 2;
ctrl = mpt_control(sysStruct, probStruct);
plot(ctrl); 
close

clear sysStruct probStruct
FourthOrder; probStruct.N = 1; probStruct.Tconstraint = 0;
probStruct.tracking = 1;
ctrl = mpt_control(sysStruct, probStruct);
plot(ctrl); 
close
