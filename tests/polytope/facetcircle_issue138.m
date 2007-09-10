function issue138
%PROFILEME

load issue138
x = facetcircle(P, ind, struct('lpsolver',3));
xgood = [   0.00040144144203
  -0.00027440064080
   2.99999352740280
   0.29999352740280
   0.49656955453986
   0.11372641353342
   2.82247490765263
   2.99223445448263]*1e2;

mbg_asserttolequal(x, xgood, 1e-5);
