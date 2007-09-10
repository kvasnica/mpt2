function subsref_test1

load ctrl1
a = ctrl.Pn;
mbg_assertequal(isa(a, 'polytope'), 1);

a = ctrl.details;
mbg_assertequal(isstruct(a), 1);

a = ctrl.details.runTime;
mbg_asserttolequal(a, 0.441);
