function mpt_owncost_test9

% tests mpt_ownmpc() on MLD systems

% as a by-product we also test handling of systems without output constraints,
% since mpt_sys() will return a system structure with just state constraints.
sysStruct = mpt_sys('turbo_car.hys');

probStruct.Q = eye(3);
probStruct.R = eye(2);
probStruct.norm = 1;
probStruct.N = 4;

% first test mpt_ownmpc()
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl = mpt_ownmpc(sysStruct, probStruct, C, O, V, 'online');
[x, u] = sim(ctrl, [10;1;5], 5);
mbg_assertequal(x(end, :), [0 -2 1]);

% now test mpt_control()
ctrl = mpt_control(sysStruct, probStruct, 'online');
[x, u] = sim(ctrl, [10;1;5], 5);
mbg_assertequal(x(end, :), [0 -2 1]);
