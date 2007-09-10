function tracking_test10

% tests tracking=1/2 for MLD systems with no equivalent MLD formulation.
% related to tracking_test11.m

% MLD system with no PWA representation
sysStruct = mpt_sys('pwa_car.hys', 'nopwa');
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
