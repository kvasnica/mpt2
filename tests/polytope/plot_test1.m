function plot_test1

% test plotting with the new 'colormap' options:

p1=polytope([0 0; 6 0; 8 -2; 0 -2]);
p2=polytope([0 0; 4 0; 6 2; 0 2]);
p3=polytope([4 0; 6 0; 6 2; 8 2]);
p4=polytope([6 0; 8 2; 8 -2]);
P = [p1 p2 p3 p4];

plot(P, struct('colormap', 'winter'));
