function projection_test7

% example taken from the PhD thesis of Colin Jones, section 6.2.2
% note that ESP sometimes fails and sometimes gives wrong result.
% this ESP problem seems to be fixed by:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=7b3e7a75cd8f

opt = mpt_options;
if ~ismember(0, opt.solvers.lp)
    % no NAG, skip this test
    warning('This test requires NAG');
    return
end

load projection_test7

% the polytope was generated as follows:
% A = [-0.0489 0.4040 0.2306; 0.0682 0.3540 -0.4081; -0.4602 -0.0895 -0.0368];
% B = [-0.4326 0.2877; 0 -1.1465; 0.1253 1.1909];
% sysStruct.A = A;
% sysStruct.B = B;
% sysStruct.C = eye(3);
% sysStruct.D = zeros(3,2);
% sysStruct.ymax = repmat(10, 3, 1);
% sysStruct.ymin = repmat(-10, 3, 1);
% sysStruct.umax = [1; 1];
% sysStruct.umin = [-1; -1];
% probStruct.norm = 2;
% probStruct.Q = eye(3);
% probStruct.R = eye(2);
% probStruct.y0bounds = 0;
% probStruct.N = 5;
% M = mpt_constructMatrices(sysStruct, probStruct);
% P = polytope([-M.E M.G], M.W);

% ESP under linux does not work with CDD, use NAG instead
mpt_options('lpsolver', 'nag');

for method = [4 5 6 7],
    Qi = projection(P, 1:3, struct('projection', method));
    
    % compare the projection to a given sample
    mbg_asserttrue(Qi==Q);
end
