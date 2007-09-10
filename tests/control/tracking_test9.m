function tracking_test9

% bug in mpt_yalmipcftoc rejected tracking=1 for mld systems converted to pwa:
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x?cs=cab227035eee
%
% this also tests fix to a bug in mpt_yalmiptracking when no output constraints
% have been defined (because we used sysStruct.ymin/ymax when creating a
% polytope, but in case of no constraints we set them to +/-Inf:
%
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x?cs=6a515655994c

sysStruct = mpt_sys('pwa_car.hys');
probStruct.Q = eye(2); 
probStruct.R = 1; 
probStruct.N = 1; 
probStruct.tracking = 1;
probStruct.norm = 2;

ctrl = mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl), 13);
