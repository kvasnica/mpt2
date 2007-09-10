function mpt_getInnerEllipsoid_test1

[e,x0]=mpt_getInnerEllipsoid(unitbox(2,1)); 
mpt_plotellip(e,x0); 
close all

% test also higher dimensions. there was a bug in mpt_getInnerEllipsoid:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=fd06bc0cf1ee

[e,x0]=mpt_getInnerEllipsoid(unitbox(3,1)); 
