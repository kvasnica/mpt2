function mpt_reachSets_test1

% tests mpt_reachSets() when input is a controller

% 2-norm case (all reach sets are polytope objects)
load di_ctrl_N3_norm2

% clear the ctrl.dynamics field to test whether we correctly assign
% dynamics to regions
cs = struct(ctrl);
cs.dynamics = 0*cs.dynamics;

% make sure the LTI system is traqnsformed to a PWA system with 1 dynamics,
% otherwise we cannot properly test dynamics-to-regions assignment
sysStruct = cs.sysStruct;
if ~iscell(sysStruct.A)
    cs.sysStruct = mpt_lti2pwa(sysStruct);
end
ctrl = mptctrl(cs);

R=mpt_reachSets(ctrl,unitbox(2,1)+[4;0],5);
mbg_assertequal(length(R), 20);

% 1-norm case (some elements may be V-represented polytopes)
load di_ctrl_N3_norm1
[R,V]=mpt_reachSets(ctrl,unitbox(2,1)+[4;0],5);
mbg_asserttrue(isa(R, 'polytope'));
mbg_asserttrue(iscell(V));

% we should have 23 H-represented polytopes and 17 V-represented ones
mbg_assertequal(length(R), 23);
mbg_assertequal(length(V), 17);
