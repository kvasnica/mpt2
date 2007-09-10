function isbounded_test1

% as of Nov-16-2005 isbounded(Pn) returns only one true/false flag, not one flag
% per region of a polyarray. verify that.

p = unitbox(2,1);
P = [p p];
isb = isbounded(P);
mbg_assertequal(length(isb), 1);
mbg_asserttrue(isb);
