function mpt_simplify_test1

load ctrl_with_hole

% first, tests simplification of controller objects:
A = mpt_simplify(ctrl_with_hole);
mbg_assertequal(length(A.Pn), 12);


% now test simplification of arbitrary partitions:
S = struct(ctrl_with_hole);
S = rmfield(S, 'sysStruct');
S = rmfield(S, 'probStruct');
A = mpt_simplify(ctrl_with_hole);
mbg_assertequal(length(A.Pn), 12);
