function identifyRegion_test


p1=polytope([0 0; 6 0; 8 -2; 0 -2]);
p2=polytope([0 0; 4 0; 6 2; 0 2]);
p3=polytope([4 0; 6 0; 6 2; 8 2]);
p4=polytope([6 0; 8 2; 8 -2]);
P = [p1 p2 p3 p4];

% this should of course work
plot(P); identifyRegion(P); close all


% this should also work
plot(P); identifyRegion(P, [1 3]); close all

% this should also work, we transpose second argument such that it is always a
% row vector
plot(P); identifyRegion(P, [1; 2]); close all

% this should give a nice error
lasterr('');
try
    plot(P); identifyRegion(P, [0; 2]); 
    waserror = 0;
catch
    waserror = 1;
end
close all
if waserror,
    mbg_assertcontains(lasterr,  'IDENTIFYREGION: Idx must represent a possible list of indices of P.');
end

% this should also give a nice error
lasterr('');
try
    plot(P); identifyRegion(P, [1; 2; 5]); 
    waserror = 0;
catch
    waserror = 1;
end
close all
if waserror, 
    mbg_assertcontains(lasterr,  'IDENTIFYREGION: Idx must represent a possible list of indices of P.');
end
