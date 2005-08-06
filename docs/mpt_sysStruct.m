%MPT_SYSSTRUCT System structure description
%
%
% System structure (sysStruct) is a structure which describes the system to be
% controlled. MPT can deal with two types of systems:
% 1. Discrete-time linear time-invariant (LTI) systems
% 2. Discrete-time Piecewise Affine (PWA) Systems
%
% Both system types can be subject to constraints imposed on control inputs and system
% outputs. In addition, constraints on slew rate of the control inputs can also
% be given.
%
%
% ---------------------------------------------------------------------------
% LTI SYSTEMS
% ---------------------------------------------------------------------------
%
% In general, a constrained linear time-invariant system is defined by the
% following relations:
%
%    x(k+1) = A x(k) + B u(k)
%     y(k)  = C x(k) + D u(k)
%
%  subject to
%
%    xmin <= x(k) <= xmax
%    ymin <= y(k) <= ymax
%    umin <= u(k) <= umax
%    dumin <= u(k) - u(k+1) <= dumax
%    dymin <= y(k) - y(k+1) <= dymax
%
% Such an LTI system is defined by the following mandatory fields:
%
%    sysStruct.A = A;
%    sysStruct.B = B;
%    sysStruct.C = C;
%    sysStruct.D = D;
%    sysStruct.ymax = ymax;
%    sysStruct.ymin = ymin;
%    sysStruct.umax = umax;
%    sysStruct.umin = umin;
%
% State constraints are optional:
%
%    sysStruct,xmax = xmax;
%    sysStruct.xmin = xmin;
%
% Constraints on slew rate of the control input u(k) can also be imposed by:
%
%    sysStruct.dumax = dumax;
%    sysStruct.dumin = dumin;
%
% which enforces   dumin <= u(k)-u(k+1) <= dumax
%
% Similarly, constraints on slew rate of the output variable can be defined by:
%
%    sysStruct.dymax = dymax;
%    sysStruct.dymin = dymin;
%
% which adds constraint  dymin <= y(k) - y(k+1) <= dymax
%
% NOTE: If no constraints are present on certain inputs/states, set the associated values to Inf.
%
% It is also possible to specify bounds on reference signals (for tracking
% problems) in:
% 
%    sysStruct.yrefmax, sysStruct.yrefmin
%    sysStruct.xrefmax, sysStruct.xrefmin
%
% An uncertain LTI system is driven by the following set of relations:
%
%    x(k+1) = Aunc x(k) + Bunc u(k) + w(k)
%     y(k)  = C x(k) + D u(k)
%
% where w(k) is an unknown, but bounded additive disturbance, i.e.
%    w(n) \in W  \forall n = 0...Inf
%
% To specify an additive disturbance, set "sysStruct.noise = W"
% where W is a polytope bounding the disturbance.
%
% A polytopic uncertainty can be specified by a cell array of matrices Aunc and
% Bunc as follows:
%
%    sysStruct.Aunc = {A1, ..., An};
%    sysStruct.Bunc = {B1, ..., Bn}; 
%
% ---------------------------------------------------------------------------
% PWA SYSTEMS
% ---------------------------------------------------------------------------
%
% PWA systems are models for describing hybrid systems. Dynamical behavior of
% such systems is captured by relations of the following form:
%
%    x(k+1) = A_i x(k) + B_i u(k) + f_i
%     y(k)  = C_i x(k) + D_i u(k) + g_i
%
% subject to
%
%    ymin  <=     y(k)    <= ymax
%    umin  <=     u(k)    <= umax
%    dumin <= u(k)-u(k-1) <= dumax
%
% Each dynamics "i" is defined in a polyhedral partition bounded by the
% so-called guardlines:
%
%   guardX_i x(k) + guardU_i u(k) <= guardC_i
%
% that means dynamics "i" will be applied if the above inequality is satisfied.
%
% Fields of sysStruct describing a PWA system are listed below:
%
%   sysStruct.A = {A1, ..., An}
%   sysStruct.B = {B1, ..., Bn}
%   sysStruct.C = {C1, ..., Cn}
%   sysStruct.D = {D1, ..., Dn}
%   sysStruct.f = {f1, ..., fn}
%   sysStruct.g = {g1, ..., gn}
%   sysStruct.guardX = {guardX1, ..., guardXn}
%   sysStruct.guardU = {guardU1, ..., guardUn}
%   sysStruct.guardC = {guardC1, ..., guardCn}
%
% Note that all fields have to be cell arrays of matrices of compatible
% dimensions, "n" stands for total number of different dynamics. If
% sysStruct.guardU is not provided, it is assumed to be zero.
%
% System constraints are given by:
%
%   sysStruct.ymax  = ymax;
%   sysStruct.ymin  = ymin;
%   sysStruct.xmax  = xmax;
%   sysStruct.xmin  = xmin;
%   sysStruct.umax  = umax;
%   sysStruct.umin  = umin;
%   sysStruct.dumax = dumax;
%   sysStruct.dumin = dumin;
%
% Constraints on slew rate are optional and can be omitted.
%
% It is also possible to specify bounds on reference signals (for tracking
% problems) in:
% 
%    sysStruct.yrefmax, sysStruct.yrefmin
%    sysStruct.xrefmax, sysStruct.xrefmin
%
% MPT is able to deal also with PWA systems which are affected by bounded
% additive disturbances:
%
%   x(k+1) = A_i x(k) + B_i u(k) + f_i + w(k)
%
% where the disturbance w(k) is assumed to be bounded for all time instances by
% some polytope W. To indicate that your system is subject to such a
% disturbance, set 
% 
%   sysStruct.noise = W;
%
% where W is a polytope object of appropriate dimension.
%
%
% Text labels can be attached to state, input and output variables:
%
% sysStruct.StateName = {'position', 'speed'};
% sysStruct.InputName = 'force';
% sysStruct.OutputName = 'position';
%
% These labels will be used as axis labels when plotting polyhedral partition
% of the explicit controller, or when visualizing trajectories.
%

% Copyright is with the following author(s):
%
%(C) 2003 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%         kvasnica@control.ee.ethz.ch
%

% ---------------------------------------------------------------------------
% Legal note:
%          This program is free software; you can redistribute it and/or
%          modify it under the terms of the GNU General Public
%          License as published by the Free Software Foundation; either
%          version 2.1 of the License, or (at your option) any later version.
%
%          This program is distributed in the hope that it will be useful,
%          but WITHOUT ANY WARRANTY; without even the implied warranty of
%          MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
%          General Public License for more details.
% 
%          You should have received a copy of the GNU General Public
%          License along with this library; if not, write to the 
%          Free Software Foundation, Inc., 
%          59 Temple Place, Suite 330, 
%          Boston, MA  02111-1307  USA
%
% ---------------------------------------------------------------------------

error('This script is not executable. Type "edit mpt_sysStruct" to see more information.');

%% Template for system structure
% * Read help description of this file for more details
% * Remove all fields marked as optional if you don't need them.
% * Certain combinations are not allowed (e.g. parametric uncertainty for PWA
%   systems) 
% * Note that constraints on slew rate of manipulated variables are only
%   guaranteed in open loop (consult MPT in 15 minutes section of the manual for
%   more details) 

%% State-space representation of linear time invariant (LTI) system

% x(k+1) = A x(k) + B u(k)
sysStruct.A = [];
sysStruct.B = [];

% y(k) = C y(k) + D u(k)
sysStruct.C = [];
sysStruct.D = [];


%% State-space representation of piecewise affine (PWA) system

% x(k+1) = A_i x(k) + B_i u(k) + f_i
sysStruct.A = { [], [], ..., [] };
sysStruct.B = { [], [], ..., [] };
sysStruct.f = { [], [], ..., [] };

% y(k) = C y(k) + D u(k) + g_i
sysStruct.C = { [], [], ..., [] };
sysStruct.D = { [], [], ..., [] };
sysStruct.g = { [], [], ..., [] };

% Each PWA system consists of 'n' affine dynamics defined over a partition in
% the state-input space guardX_i x(k) + guardU_i u(k) <= guardC_i
% Define these regions in the following fields:
sysStruct.guardX = { [], [], ..., [] };
sysStruct.guardU = { [], [], ..., [] };
sysStruct.guardC = { [], [], ..., [] };

% note that all fields are cell arrays for PWA systems. Consult Modeling section
% of MPT manual for more details.


%% System constraints

% Min/Max constraints on manipulated variables:
sysStruct.umin = [];
sysStruct.umax = [];

% Min/Max constraints on system outputs
sysStruct.ymax = [];
sysStruct.ymin = [];

%% Optional system constraints

% Min/Max constraints on system states
sysStruct.xmax = [];
sysStruct.xmin = [];

% Min/Max constraints on rates of manipulated variables
% (optional; assuming +/- Inf if bounds not provided)
sysStruct.dumin = [];
sysStruct.dumax = [];

% Min/Max constraints on rates of manipulated variables
% (optional; assuming +/- Inf if bounds provided)
sysStruct.dymin = [];
sysStruct.dymax = [];

% Polytopic constraints on state variables
% Define them as a polytope object H x <= K
% (optional, assuming R^n if no bounds provided)
sysStruct.Pbnd = polytope(H,K);


%% Text labels of state, input and output variables
% (optional; will be replaced by x_1, x_2, ... if not provided)
sysStruct.StateName = { 'label of state 1', 'label of state 2', ... };
sysStruct.InputName = { 'label of input 1', 'label of input 2', ... };
sysStruct.OutputName = { 'label of output 1', 'label of output 2', ... };

% Length of the above parameters must be consistent with dimension of your
% system.


%% Bounds on unknown, but bounded, additive disturbance
% (optional; if not given, assuming nominal dynamics)
sysStruct.noise = polytope(Hn,Kn)


%% Definition of parametric uncertainty
% (optional; only for LTI systems)
% Assuming the dynamics is driven by a convex hull of the following parameters:
sysStruct.Aunc = { [], [], ..., [] };
sysStruct.Bunc = { [], [], ..., [] };