function analyticCenter_test1


P=polytope([1 0; -1e-4 1+1e-4; 0 -1], [10; 0; 0]);
x = analyticCenter(P);

mbg_asserttolequal(x, [9.51422098945784; 0.00047566348312], 1e-8)
