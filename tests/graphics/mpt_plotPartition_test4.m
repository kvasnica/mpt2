function mpt_plotPartition_test4

% http://control.ee.ethz.ch/~mpt/hg/mpt-26x/rev/49ead19a7b44

clear sysStruct probStruct
lti1d; probStruct.N = 1; probStruct.Tconstraint = 0;
sysStruct.StateName = {'x1'};
ctrl = mpt_control(sysStruct, probStruct);
plot(ctrl); 
close

clear sysStruct probStruct
lti1d; probStruct.N = 1; probStruct.Tconstraint = 0;
sysStruct.StateName = {'x1'};
sysStruct.dumax = 1;
sysStruct.dumin = -1;
ctrl = mpt_control(sysStruct, probStruct);
plot(ctrl);
close

clear sysStruct probStruct
lti1d; probStruct.N = 1; probStruct.Tconstraint = 0;
sysStruct.StateName = {'x1'};
probStruct.tracking = 2;
ctrl = mpt_control(sysStruct, probStruct);
plot(ctrl); 
close

clear sysStruct probStruct
lti1d; probStruct.N = 1; probStruct.Tconstraint = 0;
sysStruct.StateName = {'x1'};
probStruct.tracking = 1;
ctrl = mpt_control(sysStruct, probStruct);
plot(ctrl); 
close
