function mpt_mplp_test6

% for these problems we have always been getting following error:
% ??? Error using ==> mpt_mplp_ver7
% Region 2 outside the feasible set.
%
% fixed by:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=2362d7e093fb

load mplp_regionoutside

goodPn = [2 4 4 4 4];
for ii = 1:5,
    Pn = mpt_mplp(M{ii});
    Pfeas = projection(polytope([-M{ii}.E M{ii}.G], M{ii}.W), 1:size(M{ii}.E, 2));
    mbg_asserttrue(Pn==Pfeas);
    %mbg_assertequal(length(Pn), goodPn(ii));
    mbg_asserttrue(isbounded(Pn));
end
