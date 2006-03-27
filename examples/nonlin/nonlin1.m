close all
echo on

% import a nonlinear system
sysStruct = mpt_sys(@duffing_oscilator);

% penalize quadratic cost objective
probStruct.norm = 2;
probStruct.Q = eye(2);
probStruct.R = 1/10;

% set prediction horizon
probStruct.N = 3;

% calculate an on-line controller
ctrl = mpt_control(sysStruct, probStruct, 'online');

% simulate the closed-loop system for a period of 10 seconds
Options.nlsolver = 'local';

% you can also use a "global" solver which is slower, but gives higher chance of
% finding global optima
%   Options.nlsolver = 'global';
x0 = [2.5; 1];
Tfinal = 3;
simsteps = Tfinal / sysStruct.Ts;

% simulate and plot the closed-loop trajectory
simplot(ctrl, x0, simsteps, Options);

echo off
