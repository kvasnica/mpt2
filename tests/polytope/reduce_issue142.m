function issue142_reduce

% here we had problem with CDD CC kicking out wrong constraints in reduce()
% fixed by increasing tolerance when checking bounding boxes:
% http://control.ee.ethz.ch/~mpt/hg/mpt-20x?cmd=changeset;node=bf5ecf0a59b49a180b2c0c07b7df1a28ebf131fb

load issue142

o = mpt_options;
mpt_options('lpsolver', 'cdd');

try
    pol1=polytope(h1,k1);
    [h2,k2]=double(pol1);
    pol2=polytope(h2,k2);
    
    p1b = isbounded(pol1);
    p2b = isbounded(pol2);
    p1c = nconstr(pol1);
    p2c = nconstr(pol2);
    
    mbg_assertequal(p1b, 1);
    mbg_assertequal(p2b, 1);
    mbg_assertequal(p1c, 51);
    mbg_assertequal(p2c, 51);
    
catch
    mpt_options('lpsolver', o.lpsolver);
    rethrow(lasterr);
end

mpt_options('lpsolver', o.lpsolver);
