function isexplicit_test1

load ctrl1
a = isexplicit(ctrl);
mbg_assertequal(a, 1);
