function sub_uniqueOpt_test1

% default nu, nu == size(Fi{1}, 1), size(Fi{1}, 1) == 1
Fi = {1, 2, 1, 1, 0, 0, 1};
Gi = {0, 1, 1, 0, 1, 1, 0};
t = sub_uniqueOpt(Fi, Gi);
mbg_assertequal(t.Reg, [1 2 3 1 4 4 1]);
mbg_assertequal(length(t.Table), 4);
mbg_assertequal(length(t.Fi), 4);
mbg_assertequal(length(t.Gi), 4);
mbg_assertequal([t.Table{:}], [1 4 7 2 3 5 6])
mbg_assertequal([t.Fi{:}], [1 2 1 0])
mbg_assertequal([t.Gi{:}], [0 1 1 1])


% default nu, nu == size(Fi{1}, 1), size(Fi{1}, 1) == 2
Fi = {1, 2, 1, 1, 0, 0, 1};
Gi = {0, 1, 1, 0, 1, 1, 0};
for i = 1:length(Fi)
    Fi{i} = [Fi{i}; 0];
    Gi{i} = [Gi{i}; 0];
end
t = sub_uniqueOpt(Fi, Gi);
mbg_assertequal(t.Reg, [1 2 3 1 4 4 1]);
mbg_assertequal(length(t.Table), 4);
mbg_assertequal(length(t.Fi), 4);
mbg_assertequal(length(t.Gi), 4);
mbg_assertequal([t.Table{:}], [1 4 7 2 3 5 6])
mbg_assertequal([t.Fi{:}], [1 2 1 0; 0 0 0 0])
mbg_assertequal([t.Gi{:}], [0 1 1 1; 0 0 0 0])


% nu given, nu == 1, size(Fi{1}, 1) == 2
nu = 1;
Fi = {1, 2, 1, 1, 0, 0, 1};
Gi = {0, 1, 1, 0, 1, 1, 0};
for i = 1:length(Fi)
    Fi{i} = [Fi{i}; 0];
    Gi{i} = [Gi{i}; 0];
end
t = sub_uniqueOpt(Fi, Gi, nu);
mbg_assertequal(t.Reg, [1 2 3 1 4 4 1]);
mbg_assertequal(length(t.Table), 4);
mbg_assertequal(length(t.Fi), 4);
mbg_assertequal(length(t.Gi), 4);
mbg_assertequal([t.Table{:}], [1 4 7 2 3 5 6])
mbg_assertequal([t.Fi{:}], [1 2 1 0])
mbg_assertequal([t.Gi{:}], [0 1 1 1])


% error expected when nu > size(Fi, 1)
nu = size(Fi{1}, 1)+1;
lasterr('');
try
    t = sub_uniqueOpt(Fi, Gi, nu);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Index exceeds matrix dimension.');

% error expected when nu < 1
nu = 0;
lasterr('');
try
    t = sub_uniqueOpt(Fi, Gi, nu);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Index exceeds matrix dimension.');
