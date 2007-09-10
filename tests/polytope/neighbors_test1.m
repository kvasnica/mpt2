p1=polytope([0 0; 6 0; 8 -2; 0 -2]);
p2=polytope([0 0; 4 0; 6 2; 0 2]);
p3=polytope([4 0; 6 0; 6 2; 8 2]);
p4=polytope([6 0; 8 2; 8 -2]);
P = [p1 p2 p3 p4];

p1=polytope([0 0; 2 0; 2 2; 0 2]);
p2=polytope([2 0; 2 2; 4 2; 4 0]);
p3=polytope([4 2; 4 0; 6 0; 6 2]);
p=[p1 p2 p3];
P = [p p+[0;-2]];

p1=polytope([0 2; 0 0; 5 0; 5 2]);
p2=polytope([5 2; 10 2; 5 0; 10 0]);
p3=polytope([0 0; 0 -2; 6 0; 6 -2]);
p4=polytope([6 0; 10 0; 10 -2; 6 -2]);
P = [p1 p2 p3 p4];