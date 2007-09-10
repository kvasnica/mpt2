function mpt_ownmpc_test13

% we now store a flag in the "V" structure retuned by [C,O,V]=mpt_ownmpc() which
% tells whether the constraints have been designed for on-line or for explicit
% control. therefore here we test that such flag is respected:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=8b8f2d338948

Double_Integrator
probStruct.N = 2;
probStruct.Tconstraint = 0;
probStruct.P_N = zeros(2);

% constraints designed for explicit control
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
mbg_assertequal(V.type, 'explicit');


% constraints designed for on-line control
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
mbg_assertequal(V.type, 'online');


% constraints designed for on-line control
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'on-line');
mbg_assertequal(V.type, 'online');



% if constraints are designed for on-line control, we should be able to call
% ctrl=mpt_ownmpc() without the additional 'online' flag
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl = mpt_ownmpc(sysStruct, probStruct, C, O, V);
mbg_assertfalse(isexplicit(ctrl));


% however we should be able to override the default behavior by specifying the
% controller type flag:
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl = mpt_ownmpc(sysStruct, probStruct, C, O, V, 'explicit');
mbg_asserttrue(isexplicit(ctrl));


% however we should be able to override the default behavior by specifying the
% controller type flag:
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
ctrl = mpt_ownmpc(sysStruct, probStruct, C, O, V, 'online');
mbg_assertfalse(isexplicit(ctrl));
