function mpt_yalmipcftoc_test1

% test probStruct.Qdyn
clear
sysStruct.A = {0, 6};
sysStruct.B = {1, 1};
sysStruct.C = {1, 1};
sysStruct.D = {0, 0};
sysStruct.guardX = {1, -1};
sysStruct.guardC = {0, 0};
sysStruct.umax = 5;
sysStruct.umin = -5;
sysStruct.ymax = 5;
sysStruct.ymin = -5;

probStruct.Q = 1;
probStruct.R = 1;
probStruct.norm = 1;
probStruct.N = 2;

ctrl = mpt_yalmipcftoc(sysStruct, probStruct);
% here we will get infeasibility
[x, u] = sim(ctrl, 0.5, 10);
mbg_asserttolequal(x, [0.5;1.5;4.5]);
mbg_asserttolequal(u, [-1.5;-4.5]);


probStruct.Qdyn = diag([0, 100]);
ctrl2 = mpt_yalmipcftoc(sysStruct, probStruct);
% here we tell that we should avoid dynamics 2 as much as possible (dynamics 2
% is such that x>0). as a consequence we will get stability...
[x, u] = sim(ctrl2, 0.5, 10);
mbg_asserttolequal(x, [0.5;0;0;0;0;0;0;0;0;0;0]);
mbg_asserttolequal(u, [-3;0;0;0;0;0;0;0;0;0]);
