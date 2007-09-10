function runtime_test1

load ctrl1
a = runtime(ctrl)
mbg_asserttolequal(a, 0.441);
