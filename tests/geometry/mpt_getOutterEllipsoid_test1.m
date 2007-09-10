function mpt_getOutterEllipsoid_test1

[e,x0]=mpt_getOutterEllipsoid(unitbox(2,1)); 
mpt_plotellip(e,x0); 
close all

% test also higher dimensions
[e,x0]=mpt_getOutterEllipsoid(unitbox(3,1)); 
