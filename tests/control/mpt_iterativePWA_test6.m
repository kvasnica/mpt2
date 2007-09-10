function mpt_iterativePWA_test1

% prior to 2.0.4 there was a bug which caused mpt_iterativePWA to break with
% error if user-defined target set was given, but no dynamics contained the
% origin in it's interior.

sysStruct.A = { eye(2), eye(2) };
sysStruct.B = { eye(2), eye(2) };
sysStruct.C = { eye(2), eye(2) };
sysStruct.D = { eye(2), eye(2) };
sysStruct.umax = ones(2,1);
sysStruct.umin = -ones(2,1);
sysStruct.ymax = ones(2,1);
sysStruct.ymin = -ones(2,1);

% define dynamics such that they do not contain the origin
H = [eye(2); -eye(2)];
K = [3; 1; -1; 1];
sysStruct.guardX = { H, H };
sysStruct.guardC = { K, K };

probStruct.subopt_lev = 1;
probStruct.N = Inf;
probStruct.Q = eye(2);
probStruct.R = eye(2);
probStruct.norm = 2;

% define custom Tset
probStruct.Tset = unitbox(2,1);

% this test should NOT break
ctrl = mpt_control(sysStruct, probStruct, struct('maxiterations', 1));
