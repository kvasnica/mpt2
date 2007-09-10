function mpt_removeOverlaps_test2

% bug reported by Johan Loefberg on 4-Nov-2005. correct solution should
% consist of 51 regions. note that there are some parititions which are
% identical. 

load rmovlpsbug
S = mpt_removeOverlaps(model);
mbg_assertequal(length(S.Pn), 51);
