%PWA1D 1st order PWA example with 2 PWA dynamics
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% none
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% sysStruct, probStruct - system and problem definition structures stores
%                         in the workspace
%

% Copyright is with the following author(s):
%
% (C) 2005 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%          kvasnica@control.ee.ethz.ch

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


clear sysStruct probStruct

% PWA description 1: x(k+1)=A{1} x(k) + B{1} u(k) + f{1}
sysStruct.A{1} = 1.5;
sysStruct.B{1} = 1;
sysStruct.f{1} = 0;
% y(k) = C{1} x(k) + D{1} u(k) + g{1}
sysStruct.C{1} = 1;
sysStruct.D{1} = 0;
sysStruct.g{1} = 0;
% guardX{1}*x(k)+guardU{1}*u(k)<=guardC{1}
sysStruct.guardX{1} = -1;
sysStruct.guardC{1} = 0;

% PWA description 2: x(k+1)=A{2} x(k) + B{2} u(k) + f{2}
sysStruct.A{2} = -1.5;
sysStruct.B{2} = 1;
sysStruct.f{2} = 0;
% y(k) = C{2} x(k) + D{2} u(k) + g{2}
sysStruct.C{2} = 1;
sysStruct.D{2} = 0;
sysStruct.g{2} = 0;
% guardX{2}*x(k)+guardU{2}*u(k)<=guardC{2}
sysStruct.guardX{2} = 1;
sysStruct.guardC{2} = 0;

sysStruct.ymin = -1.5;
sysStruct.ymax = 1.5;
sysStruct.umin = -1;
sysStruct.umax = 1;

sysStruct.noise = unitbox(1, 0.1);

probStruct.Q  = 1;  % weight on the state (nx x nx)
probStruct.R  = 1;         % weight on the input (nu x nu)
probStruct.N = Inf;        % prediction horizon
probStruct.norm = 2;
probStruct.subopt_lev=1;
