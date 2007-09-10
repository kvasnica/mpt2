function mpt_owncost_test2

%----------------------------------------------------------------------
% mixed output/input constraints
Double_Integrator
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
for k = 1:probStruct.N,
    C = C + set(-0.8 < V.y{k}(2) + V.u{k} < 0.8);
end
ctrl = mpt_ownmpc(sysStruct,probStruct,C,O,V);
[x, u, y] = sim(ctrl, [4; -1.8], struct('openloop', 1));
mbg_asserttrue(min(y(:, 2)+u) >= -0.8);
mbg_asserttrue(max(y(:, 2)+u) <= 0.8);
mbg_assertequal(length(ctrl), 39);

% test the same with an on-line controller
% mixed output/input constraints
Double_Integrator
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
for k = 1:probStruct.N,
    C = C + set(-0.8 < V.y{k}(2) + V.u{k} < 0.8);
end
ctrl = mpt_ownmpc(sysStruct,probStruct,C,O,V, 'online');
[x, u, y] = sim(ctrl, [4; -1.8], struct('openloop', 1));
mbg_asserttrue(min(y(:, 2)+u) >= -0.8 - sqrt(eps));
mbg_asserttrue(max(y(:, 2)+u) <= 0.8+sqrt(eps));



%----------------------------------------------------------------------
% move blocking (mixing constraints at different prediction steps
Double_Integrator
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
% u_0=u_1, u_2=u_3, u_4 free
C = C + set(V.u{1}==V.u{2}) + set(V.u{3}==V.u{4});
ctrl = mpt_ownmpc(sysStruct,probStruct,C,O,V);
[x, u] = sim(ctrl, [1; -1], struct('openloop', 1));
mbg_asserttolequal(u(1), u(2));
mbg_asserttolequal(u(3), u(4));
mbg_asserttrue(abs(u(2)-u(3))>0.1); % assure that u_1 and u_2 are different
mbg_asserttrue(abs(u(4)-u(5))>0.1); % assure that u_3 and u_4 are different
mbg_assertequal(length(ctrl), 11);

% test the same with an on-line controller
Double_Integrator
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
% u_0=u_1, u_2=u_3, u_4 free
C = C + set(V.u{1}==V.u{2}) + set(V.u{3}==V.u{4});
ctrl = mpt_ownmpc(sysStruct,probStruct,C,O,V, 'online');
[x, u] = sim(ctrl, [1; -1], struct('openloop', 1));
mbg_asserttolequal(u(1), u(2));
mbg_asserttolequal(u(3), u(4));
mbg_asserttrue(abs(u(2)-u(3))>0.1); % assure that u_1 and u_2 are different
mbg_asserttrue(abs(u(4)-u(5))>0.1); % assure that u_3 and u_4 are different


%----------------------------------------------------------------------
% variable reference
% min (y_0 - 1) + (y_1 - 0.8) + (y_2 - 0.6) + (y_3 - 0.4) + (y_4 - 0.2)
Double_Integrator
[C, obj, V] = mpt_ownmpc(sysStruct, probStruct);
obj = 0;
Qy = 10; R = 0.1;
refs = [1; 0.8; 0.6; 0.4; 0.2];
for k = 1:probStruct.N,
    obj = obj + (V.y{k}(1)-refs(k))'*Qy*(V.y{k}(1)-refs(k)) + V.u{k}'*R*V.u{k};
end
ctrl = mpt_ownmpc(sysStruct,probStruct,C,obj,V);
[x, u, y] = sim(ctrl, [1; 0], struct('openloop', 1));
mbg_asserttolequal(y(:, 1), refs, 5e-3);  % we must be close to the reference

% test the same with an on-line controller
Double_Integrator
[C, obj, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
obj = 0;
Qy = 10; R = 0.1;
refs = [1; 0.8; 0.6; 0.4; 0.2];
for k = 1:probStruct.N,
    obj = obj + (V.y{k}(1)-refs(k))'*Qy*(V.y{k}(1)-refs(k)) + V.u{k}'*R*V.u{k};
end
ctrl = mpt_ownmpc(sysStruct,probStruct,C,obj,V,'online');
[x, u, y] = sim(ctrl, [1; 0], struct('openloop', 1));
mbg_asserttolequal(y(:, 1), refs, 5e-3);  % we must be close to the reference
