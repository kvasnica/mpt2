function mpt_simplexContr_test1

% tests computation of simplex controllers
lti1d
ctrl=mpt_simplexContr(sysStruct, probStruct);

mbg_assertequal(length(ctrl), 1);

% mpt_simplexContr must return ctrl.details.volume
mbg_asserttolequal(ctrl.details.volume, 7.2361, 1e-4);
