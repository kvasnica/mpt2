function mpt_sys_test1

% test mpt_sys(..., 'nosimulator'):
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=62a6bc85df58

% there are algebraic loops in 'dcboost.hys' which prevent to generate the
% simulator. moreover, also PWA translation fails for the same reason. bottom
% line, without the 'nosimulator' flag we would have an error in mpt_sys:

sst = mpt_sys('dcboost', 'nosimulator', 'nopwa');

% check if data.SIM.code is empty
mbg_asserttrue(isempty(sst.data.SIM.code));

% check whether we generate a valid system structure:
sst.ymax = 10;
sst.ymin = -10;
mpt_verifySysStruct(sst);
