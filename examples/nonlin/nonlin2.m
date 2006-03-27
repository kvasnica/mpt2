close all
echo on

% import a piecewise nonlinear system
sysStruct = mpt_sys(@pw_nonlin);

% penalize linear cost objective
probStruct.norm = 1;
probStruct.Q = eye(2);
probStruct.R = 1;
probStruct.P_N = 10*eye(2);

% set prediction horizon
probStruct.N = 2;

% calculate an on-line controller
ctrl = mpt_control(sysStruct, probStruct, 'online');

% simulate the closed-loop system for a period of 10 seconds
Options.nlsolver = 'global';

% you can also use a "global" solver which is slower, but gives higher chance of
% finding global optima
%   Options.nlsolver = 'global';
x0 = [2.5; 1];

% number of simulation steps
simsteps = 6;

% simulate and plot the closed-loop trajectory
simplot(ctrl, x0, simsteps, Options);

echo off
