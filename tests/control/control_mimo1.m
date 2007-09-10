function control_mimo1

% if the system to be controlled has more than one input, mpt_yalmipcftoc
% and mpt_ownmpc used wrong concatenation of elements, resulting to an
% open-loop sequence of wrong dimension

% system with 3 states and 2 inputs
ThirdOrder;
probStruct.Tconstraint = 0;
probStruct.norm = 2;
probStruct.subopt_lev = 0;
probStruct.N = 3;
x0 = [0.5; 0.5; 0.5];
ugood = [-0.4171925132;0.14628497505;-0.15850165045;-0.0357738161;0;0];

% mpt_yalmipcftoc / explicit solutions
%
ctrl = mpt_yalmipcftoc(sysStruct, probStruct);
u = ctrl(x0, struct('openloop', 1));
% open-loop solution should have 6 elements (nu=2, N=3)
mbg_assertequal(length(u), 6);
mbg_asserttolequal(u, ugood);
% closed-loop solution should be identical to first element of the
% open-loop sequence
mbg_asserttolequal(ctrl(x0), u(1:2));

% mpt_control / explicit solution
%
ctrl = mpt_control(sysStruct, probStruct);
u = ctrl(x0, struct('openloop', 1));
% open-loop solution should have 6 elements (nu=2, N=3)
mbg_assertequal(length(u), 6);
mbg_asserttolequal(u, ugood);
% closed-loop solution should be identical to first element of the
% open-loop sequence
mbg_asserttolequal(ctrl(x0), u(1:2));

% mpt_control / on-line solution
%
ctrl = mpt_control(sysStruct, probStruct, 'online');
u = ctrl(x0, struct('openloop', 1));
% open-loop solution should have 6 elements (nu=2, N=3)
mbg_assertequal(length(u), 6);
mbg_asserttolequal(u, ugood);
% closed-loop solution should be identical to first element of the
% open-loop sequence
mbg_asserttolequal(ctrl(x0), u(1:2));

% mpt_ownmpc / explicit solution
%
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
ctrl = mpt_ownmpc(sysStruct, probStruct, C, O, V);
u = ctrl(x0, struct('openloop', 1));
% open-loop solution should have 6 elements (nu=2, N=3)
mbg_assertequal(length(u), 6);
mbg_asserttolequal(u, ugood);
% closed-loop solution should be identical to first element of the
% open-loop sequence
mbg_asserttolequal(ctrl(x0), u(1:2));

% mpt_ownmpc / on-line solution
%
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
ctrl = mpt_ownmpc(sysStruct, probStruct, C, O, V, 'online');
u = ctrl(x0, struct('openloop', 1));
% open-loop solution should have 6 elements (nu=2, N=3)
mbg_assertequal(length(u), 6);
mbg_asserttolequal(u, ugood);
% closed-loop solution should be identical to first element of the
% open-loop sequence
mbg_asserttolequal(ctrl(x0), u(1:2));
