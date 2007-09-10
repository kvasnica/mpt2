function projection_issue38

% http://autsun04.ee.ethz.ch:8080/mpt/issue38
% The result from to consecutive calls to projection is different. Is there a 
% randomization strategy somewhere?

load projection_issue38

% projection of P on 1:4 should lead a polytope with 29 constraints

dim = 1:4;

for method = [2 7]
    q = projection(P, dim, struct('projection', method));
    mbg_assertequal(nconstr(q), 29);
end
