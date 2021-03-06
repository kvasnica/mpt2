function mpt_sys_test4

% tests whether we give a nice error message when input is an MLD structure
% generated by mpt_pwa2mld():
%

opt_sincos
S = mpt_pwa2mld(sysStruct);
lasterr('');
try
    % this should not work
    s = mpt_sys(S);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Cannot generate PWA representation, see message above.');


% after disabling the PWA conversion it should work
s = mpt_sys(S, 'nopwa');
