function mpt_sysStructInfo_test1

% tests whether we display information about LTI/PWA systems correctly:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=84289275a45c

% LTI systems
Double_Integrator
mpt_sysStructInfo(sysStruct)

% PWA systems
two_tanks
mpt_sysStructInfo(sysStruct)

% PWA system - 3D
pwa3d
mpt_sysStructInfo(sysStruct)
