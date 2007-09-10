function isinvariant_test1

load ctrl1
a = isinvariant(ctrl);
mbg_assertequal(a, 1);
