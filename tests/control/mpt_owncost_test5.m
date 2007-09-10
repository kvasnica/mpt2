function mpt_owncost_test5

% tests more complex constraints involving binary variables

Double_Integrator
probStruct.norm = 1;
probStruct.N = 3;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);

% without this constraint, the open-loop optimizer for initial state x0=[1;1] is
% u=[-1; -1; 0]. with the constraint the sum of all elements should be less than
% 1.5
C = C + set(norm([V.u{:}], 1) < 1.5);

% note! it is also possible to implement the same constraints using
%   C = C + set(sum(abs([V.u{:}])) < 1.5);
% but it is far less efficient since it introduces more binary variables.

ctrl = mpt_ownmpc(sysStruct,probStruct,C,O,V);
[x,u,y,c]=sim(ctrl, [1;1], struct('openloop',1));
mbg_asserttrue(sum(u) <= 1.5);
mbg_assertequal(length(ctrl), 32);
