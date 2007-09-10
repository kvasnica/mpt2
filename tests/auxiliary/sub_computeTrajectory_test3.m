function sub_computeTrajectory_test3

% tests if we avoid a bug in sub_computeTrajectory which caused closed-loop cost
% to be incorrect for cases with:
%   * deltaU constraints
%   * output reference regulation (probStruct.yref)

% this also tests another bug -- cost computation was aborted with an error if
% we had deltaU formulation, becuase Q was not cropped down to a proper
% dimension.

load ctrl_1d_du_yref
[x,u,y,d,c]=sub_computeTrajectory(ctrl, [1;1], 10, struct('reference', [0;0])); 

% 54.61115 is the correct closed-loop cost for this example
mbg_asserttolequal(c, 54.61115, 1e-5);
