function mpt_removeOverlaps_test6

% mpt_removeOverlaps() called on mptctrl objects was failing
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=9ed2846429e0

Double_Integrator
probStruct.N=1; 
probStruct.Tconstraint=0; 
probStruct.P_N=zeros(2); 
probStruct.norm=1;
ctrl = mpt_control(sysStruct, probStruct);

ctrl = set(ctrl, 'overlaps', 1);
a = mpt_removeOverlaps(ctrl);
mbg_assertfalse(a.overlaps);
