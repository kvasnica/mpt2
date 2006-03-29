clear sysStruct probStruct Options
close all
echo on

% import a nonlinear system
sysStruct = mpt_sys(@duffing_oscillator);

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

% For safety, monitor what is happening in YALMIP
Options.verbose = 2;

% use a local nonlinear solver
Options.nlsolver = 'local';

% simulate and plot the closed-loop trajectory
simplot(ctrl, x0, simsteps, Options);

figure

% use YALMIPs global nonlinear solver
Options.nlsolver = 'global';
Options.lowersolver = 'cdd';
Options.nliter = 5;

% This takes longer time but aims at a globally optimal solution
simplot(ctrl, x0, simsteps, Options);

echo off
