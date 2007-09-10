function mpt_optControlPWA_test5

% check that we set ctrl.overlaps to zero for systems with just one dynamics:
%

lti1d
% use affine term
sysStruct.f = 1;

% convert the LTI system to PWA form
sysStruct = mpt_lti2pwa(sysStruct);

% CFTOC solution
probStruct.N = 2; probStruct.norm = 1;

% prior to Nov-11-2005, ctrl.overlaps was set to 1 if we had just one dynamics
ctrl = mpt_control(sysStruct, probStruct);

% ctrl.overlaps must be set to 0
mbg_assertequal(ctrl.overlaps, 0);
