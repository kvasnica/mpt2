function mpt_verifySysProb_test1

% tests whether we properly assiging default values of Options.sysstructname and
% Options.probstructname if they can't be deduced by inputname():
%
% http://control.ee.ethz.ch/~mpt/hg/mpt-noise?cs=5721860ac902
%
% before this bug was fixed, the output was something like '".xmax" not defined'

Double_Integrator

% corrupt system structure
sst{1} = sysStruct;
sst{1}.xmax = [1;1];
% the output must contain "sysStruct." since 'sst{1}' is not returned by
% inputname()
try
    mpt_verifySysStruct(sst{1});
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Both "sysStruct.xmin" and "sysStruct.xmax" must be defined!');


% corrupt problem structure
pst{1} = probStruct;
pst{1}.N = 0;
% the output must contain "sysStruct." since 'sst{1}' is not returned by
% inputname()
try
    mpt_verifyProbStruct(pst{1});
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Prediction horizon "probStruct.N" must be greater zero!');
   

% test also mpt_verifySysProb:
sst{1} = sysStruct;
pst{1} = probStruct;
pst{1}.Q = eye(3);
try
    mpt_verifySysProb(sst{1}, pst{1});
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, '"probStruct.Q" must be a 2x2 matrix!');
    
