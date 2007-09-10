function mpt_verifySysStruct_test1

% tests whether we catch incorrect dimensions of noise polytopes correctly:
% http://control.ee.ethz.ch/~mpt/hg/mpt-noise?cs=d6838ac73a98

Double_Integrator

% first test noise in H-representation
%disp('Noise has correct dimension');
sysStruct.noise = unitbox(2,1);         % noise has correct dimension
sst = mpt_verifySysStruct(sysStruct);


%disp('Noise has wrong dimension');
sysStruct.noise = unitbox(1,1);         % noise has wrong dimension
try
    % this must give an error
    sst = mpt_verifySysStruct(sysStruct);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, '"sysStruct.noise" has wrong dimension.');


% now test noise in V-representation
%disp('Noise has correct dimension');
sysStruct.noise = rand(2,3);             % noise has correct dimension
sst = mpt_verifySysStruct(sysStruct);


%disp('Noise has wrong dimension');
sysStruct.noise = rand(3,2);             % noise has wrong dimension
try
    % this must give an error
    sst = mpt_verifySysStruct(sysStruct);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, '"sysStruct.noise" has wrong dimension.');
