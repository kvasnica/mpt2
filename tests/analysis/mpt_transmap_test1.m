function mpt_transmap_test1

% tests whether both CDD and analytic extreme point enumneration lead to the
% same result (related to problem of Raphael Cagienard, 17-Nov-2005 where
% invariant set was computed incorrectly with CDD)

load transmap_cddprob

extsol = mpt_options('extreme_solver');

mpt_options('extreme_solver', 'matlab');
tmap0 = mpt_transmap(Pn, Acell, Fcell, tmapOptions);

mpt_options('extreme_solver', 'cdd');
tmap3 = mpt_transmap(Pn, Acell, Fcell, tmapOptions);

mpt_options('extreme_solver', extsol);

opt = mpt_options;
if ~ismember(0, opt.solvers.lp)
    % no NAG, skip this test
    warning('This test requires NAG');
    return
end

% transition maps must be equal with both extreme enumerations
mbg_asserttrue(all(all((tmap3-tmap0)==0)));
