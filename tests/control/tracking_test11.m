function tracking_test11

% bug in mpt_yalmipcftoc rejected tracking=1 for mld systems converted to pwa:
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x?cs=cab227035eee
%
% this also tests fix to a bug in mpt_yalmiptracking when no output constraints
% have been defined (because we used sysStruct.ymin/ymax when creating a
% polytope, but in case of no constraints we set them to +/-Inf:
%
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x?cs=6a515655994c

% MLD system with an equivalent PWA representation
T = evalc('sysStruct = mpt_sys(''pwa_car.hys'');');
probStruct.Q = eye(2); 
probStruct.R = 1; 
probStruct.N = 1; 
probStruct.norm = 2;

probStruct.tracking = 1;
ctrl = mpt_control(sysStruct, probStruct, 'online');
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl2 = mpt_ownmpc(sysStruct, probStruct, C, O, V);

probStruct.tracking = 2;
ctrl = mpt_control(sysStruct, probStruct, 'online');
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl2 = mpt_ownmpc(sysStruct, probStruct, C, O, V);
