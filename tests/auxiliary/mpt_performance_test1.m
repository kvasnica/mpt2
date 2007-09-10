function mpt_performance_test1

% tests if we are correctly updating ctrl.Pfinal if Options.Pfinal is given:

load ctrl_data

% first test mpt_performance without additional options
c = mpt_performance(ctrl_tr0_du0, 2);
mbg_asserttolequal(c, 35.3447179, 1e-5);

% now try to use custom Pfinal
Opt.Pfinal = unitbox(2, 2);
c = mpt_performance(ctrl_tr0_du0, 2, Opt);
mbg_asserttolequal(c, 31.9706981, 1e-5);


% now try the same with controller converted to a structure
c = mpt_performance(struct(ctrl_tr0_du0), 2, Opt);
