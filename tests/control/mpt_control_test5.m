function mpt_control_test5

% cases where control horizon is used for PWA systems with minimum-time
% strategies should be allowed in mpt_control. i.e. instead of:
%
% Cannot solve this setup because of conflicting objectives/constraints:
%  * prediction horizon is infinity
%  * control horizon for PWA systems
% 
% Please modify your system/problem setup to remove one of the conflicts.
% 
% ??? Error using ==> mpt_control
% Cannot handle given setup, see message above.
%
% we should let mpt_verifyProbStruct() give the proper error:
% %
% ??? Error using ==> mpt_verifyProbStruct
% Control horizon must be equal to prediction horizon for low-complexity strategies!
% 
% Error in ==> mpt_verifySysProb at 90
%     probStruct = mpt_verifyProbStruct(probStruct, Options);

pwa_sincos;
probStruct.Nc = 1;
lasterr('');
try
    ctrl = mpt_control(sysStruct, probStruct);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Control horizon must be equal to prediction horizon for low-complexity strategies!');
