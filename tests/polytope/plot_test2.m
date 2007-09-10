function plot_test2

% tests Options.zvalue
%

P = unitbox(2, 1);
plot(P, struct('zvalue', 1));
view(3);
close all
