function isstabilizable_test1

load ctrl1
a = isstabilizable(ctrl);
mbg_assertequal(a, 1);
