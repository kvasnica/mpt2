function mpt_ownmpc_test19

% mpt_ownmpc() should allow the objective to be provided either as zero or
% as an empty matrix:
%
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x/rev/cdcb360a9cf9

Double_Integrator;
probStruct.Tconstraint = 0; probStruct.N = 1;
sysStruct.xmax = sysStruct.ymax; sysStruct.xmin = sysStruct.ymin;
[C, obj, V] = mpt_ownmpc(sysStruct, probStruct, 'online');

% zero should work
obj = 0;
ctrl = mpt_ownmpc(sysStruct, probStruct, C, obj, V);

% empty matrix should work
obj = [];
ctrl = mpt_ownmpc(sysStruct, probStruct, C, obj, V);

% objective must not be anything else but either sdpvar or a double
lasterr('');
obj = 'dummy';
try
    ctrl = mpt_ownmpc(sysStruct, probStruct, C, obj, V);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Fourth input must be an optimization objective.');
