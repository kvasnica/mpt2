function arno_glpkprob

% with GLPK as an MILP solver we have been getting infeasibility around
% iteration 20 due to:
%
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=38235032f488
%
% and due to not properly imposing constraints on xN:
%
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=fcb0b39d8073

% Parameters
Je = 0.2;               % Engine Inertia
Jv = 0.8;               % Vehicle-side Inertia
mu = 1;                 % Friction Coefficient
b = 0.67;               % Load is dependent on velocity
ts = 1e-2;              % Sample Time 
reltol = 1;             % Stick boundary

wemin = 80;             % Engine Stall constraint (rad/s)
wemax = 200;            % Engine Max. rotational velocity (rad/s) 
gammamin = -50;         % Min. relative velocity (rad/s)
gammamax = 200;         % Max. relative velocity

Temin = 0;              % Min. Engine Torque
Temax = 150;            % Max. Engine Torque
Fnmin = 0;              % Min. Clutch Normal Force
Fnmax = 180;            % Max. Clutch Normal Force

Tegrad = 500;           % max gradient engine torque per second
Fngrad = 800;           % max gradient normal force clutch per second

Teincmin = -Tegrad*ts;  % Min. increment Engine Torque
Teincmax = Tegrad*ts;   % Max. increment Engine Torque
Fnincmin = -Fngrad*ts;  % Min. increment Clutch Normal Force
Fnincmax = Fngrad*ts;   % Max. increment Clutch Normal Force

% Constraints
umax=[0.99*Teincmax,0.99*Fnincmax]';
umin=[0.99*Teincmin,0.99*Fnincmin]';
xmin=[1.01*Temin,1.01*Fnmin,1.01*wemin,1.01*gammamin]';
xmax=[0.99*Temax,0.99*Fnmax,0.99*wemax,0.99*gammamax]';

NN = 10;

TT = evalc('sysStruct = mpt_sys(''arno_glpkprob.hys'',0.01)');
sysStruct.ymax=[inf inf]';      %must be defined!!
sysStruct.ymin=[-inf -inf]';
sysStruct.umax=umax;
sysStruct.umin=umin;
sysStruct.xmax=xmax;
sysStruct.xmin=xmin;

probStruct.N=NN;                %prediction horizon
probStruct.Q=diag([0 0 0 0]);   %weight on states
probStruct.Qy=diag([1 4]);      %weight on outputs
probStruct.R=diag([0 0]);       %weight on inputs
probStruct.norm=Inf;            %cost objective
probStruct.subopt_lev=0;        %0 - cost optimal solution; 1 - time optimal solution; 2 - low complexity solution
%probStruct.tracking=1;          %0 - no tracking; 1 - tracking (offset-free); 2 - tracking (offset-free not guaranteed)
probStruct.yref=[100;0];        %output reference

ctrlStruct = mpt_control(sysStruct,probStruct,'online')

x0 = [0;0;150;150];
Options.reference = [100;0];
milpsolver = mpt_options('milpsolver');
mpt_options('milpsolver', 2); % use glpk
try
    [X,U,Y] =mpt_computeTrajectory(ctrlStruct, x0 , 50, Options); 
catch
    mpt_options('milpsolver', milpsolver);
    rethrow(lasterror);
end
mpt_options('milpsolver', milpsolver);
    
% with GLPK as an MILP solver we have been getting infeasibility around
% iteration 20.
mbg_asserttrue(size(X, 1) >= 50);
