clear sysStruct probStruct Options
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
x0 = [2.5; 1];
Tfinal = 3;
simsteps = Tfinal / sysStruct.Ts;

% use a local nonlinear solver
Options.nlsolver = 'local';

% simulate and plot the closed-loop trajectory
simplot(ctrl, x0, simsteps, Options);

echo off
