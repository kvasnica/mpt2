function mpt_ownmpc_test3

% tests that we properly set the quadratic cost term to zero for linear norm
% solutions:
% http://control.ee.ethz.ch/~mpt/hg/mpt-yalmip?cs=11d65d80c6f8

Double_Integrator
probStruct.norm = 1;
probStruct.N = 1;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
ctrl = mpt_ownmpc(sysStruct,probStruct,C,O,V);

% test the same for on-line controllers:
Double_Integrator
probStruct.norm = 1;
probStruct.N = 1;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl = mpt_ownmpc(sysStruct,probStruct,C,O,V, 'online');
