function mpt_mplp_ver7_test1

% tests the "Point xBeyond violates more than one bound." problem
% we used to have problems on some examples, such as:
%
% two_tanks
% sysStruct.Uset{1} = [0 0.3 0.6];
% ctrl = mpt_control(sysStruct, probStruct);
%
% most of the troubles are fixed by
% http://control.ee.ethz.ch/~mpt/hg/mpt-20x?cmd=changeset;node=5aadea99833ca71d33d1276a5e4dc3a82f617020

load xbeyond_prob
Options = [];
Options.verbose = 0;

% before 28.9.2005 this failed with an error:
Pn = mpt_mplp(M1, Options);
mbg_assertequal(length(Pn), 2);

% before 28.9.2005 this failed with an error:
Pn = mpt_mplp(M2, Options);
mbg_assertequal(length(Pn), 1);
