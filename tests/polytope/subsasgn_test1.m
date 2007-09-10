function subsasgn_test1

% from Mato Baotic on 16-Nov-2005:
% > 6. in the old verion one could for example define (1x4) polyarray PA in 
% > R^2, and then change PA(2) with say PA(2)=PA(2)*PA(2) (i.e. polytope in 
% > R^4) without any error. not good. if we do not fix this then the 
% > following are errors:
% > - dimension(PA) would return the wrong result (because MPT checks 
% > dimension(PA(1))).

p = unitbox(2,1);
P = [p p];
q = unitbox(4,1);

% this should give an error because P has dimension 2 and we are trying to
% include a 4D polytope:
try
    P(2) = q;
    worked = 1;
catch
    worked = 0;
end
disp(lasterr);
mbg_assertfalse(worked);
