function mpt_getCommonSOSLyapFct_test1

% test that we correctly inform that noise in V-representation is not supported
% by this function:
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x/rev/b4c5b0271002

load ctrl_lowdimnoise_inv

try
    a=mpt_getCommonSOSLyapFct(ctrl,2);
    worked = 1;
catch
    worked = 0;
end

mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Only polytopic noise is supported by this function.');
