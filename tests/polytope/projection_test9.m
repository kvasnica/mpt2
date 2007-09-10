function projection_test9
% this test requires NAG

% tests improvement of ESP:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=7b3e7a75cd8f

opt = mpt_options;
if ~ismember(0, opt.solvers.lp)
    % no NAG, skip this test
    warning('This test requires NAG');
    return
end

load esp_prob

% without the above patch, following used to fail (and still fails on linux
% unless NAG is used as an LP solver)
mpt_options('lpsolver', 'nag');

try
    Q = projection(P, dim, struct('projection', 4));
    worked=1;
catch
    rethrow(lasterror);
    worked = 0;
end
mbg_asserttrue(worked);
Q7 = projection(P, dim, struct('projection', 7));

mbg_asserttrue(Q==Q7);
