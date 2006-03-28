clear sysStruct probStruct Options
close all
release = str2num(version('-release'));
if release<14,
    fprintf('WARNING! fmincon very often stalls with Matlab R13 and older\n');
    fprintf('         if the "global" nonlinear solver is used. Therefore\n');
    fprintf('         we recommend to set Options.nlsolver=''local'' and\n');
    fprintf('         Options.convertconvex=1 in this demo.\n');
    fprintf('\nPress any key to continue...\n');
    pause
else
    Options = [];
end

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

if release < 14,
    % Use a local nonlinear solver in Matlab R13
    Options.nlsolver = 'local';
    Options.convertconvex = 1;
else
    % Use a global nonlinear solver in Matlab R14
    Options.nlsolver = 'global';
end

% initial state
x0 = [2.5; 1];

% number of simulation steps
simsteps = 6;

% simulate and plot the closed-loop trajectory
simplot(ctrl, x0, simsteps, Options);

echo off
