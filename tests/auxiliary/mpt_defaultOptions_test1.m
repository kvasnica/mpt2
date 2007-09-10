function mpt_defaultOptions_test1

Opt = mpt_defaultOptions([], 'a', 1, 'b', 2);
mbg_asserttrue(isstruct(Opt));
mbg_assertequal(Opt.a, 1);
mbg_assertequal(Opt.b, 2);

Opt = mpt_defaultOptions(Opt, 'c', 3);
mbg_asserttrue(isstruct(Opt));
mbg_assertequal(Opt.a, 1);
mbg_assertequal(Opt.b, 2);
mbg_assertequal(Opt.c, 3);
