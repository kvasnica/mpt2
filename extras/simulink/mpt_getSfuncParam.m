function [H, K, F, G, nc, nr, nx, nu, ny, nxt, nref, Ts, dumode, tracking, abstol] = mpt_getSfuncParam(ctrl)
%MPT_GETSFUNCPARAM Prepares parameters for explicit controller S-function
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Creates parameters to be passed to mpt_getInput_sfunc_parm, a
% S-function which simulates a given explicit controller.
%
% Internal function
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
%

% $Id: mpt_getSfuncParam.m,v 1.3 2005/04/11 09:15:02 kvasnica Exp $
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
    
error(nargchk(1,1,nargin)); 

global mptOptions
if ~isstruct(mptOptions)
    mpt_error;
end

if isa(ctrl, 'polytope'),
    % input is a polytope
    P = ctrl;
    nx = dimension(P);
    nr = length(P);
    abstol = mptOptions.abs_tol;
    [Hn,Kn] = double(P);
    if ~iscell(Hn),
        Hn = {Hn};
        Kn = {Kn};
    end
    H = []; K = []; nc = [];
    for ii = 1:length(Hn),
        h = Hn{ii}';
        k = Kn{ii};
        H = [H; h(:)];
        K = [K; k(:)];
        nc = [nc; size(Hn{ii}, 1)];
    end
    HH = H;
    KK = K;
    NCC = nc;
    
    % map computed parameters to output arguments
    H = nr;
    K = nx;
    F = abstol;
    G = HH;
    nc = KK;
    nr = NCC;
    return
end
    
if ~isa(ctrl, 'mptctrl')
    error('Input must be an MPTCTRL controller object!');
end
if ~isexplicit(ctrl)
    error('Only explicit controllers can be exported to C code!');
end

if isfield(ctrl.probStruct, 'feedback')
    if ctrl.probStruct.feedback~=0,
        error('Controllers with feedback prestabilization are not supported.');
    end
end

nr = length(ctrl);
Pn = ctrl.Pn;
[Hn,Kn] = double(Pn);
if ~iscell(Hn),
    Hn = {Hn};
    Kn = {Kn};
end
nctotal = 0;
for ii=1:length(Pn),
    nctotal = nctotal + nconstr(Pn(ii));
end
nr = length(Pn);
Fi = ctrl.Fi;
Gi = ctrl.Gi;
nx = ctrl.details.dims.nx;
nu = ctrl.details.dims.nu;
ny = ctrl.details.dims.ny;
nref = 0;
nxt = nx;

for ii=1:length(Fi),
    Fi{ii} = Fi{ii}(1:nu,:);
    Gi{ii} = Gi{ii}(1:nu);
end
dumode = isfield(ctrl.sysStruct, 'dumode')+0;
tracking = ctrl.probStruct.tracking;
if tracking==1,
    nxt = ctrl.sysStruct.dims.nx;
elseif dumode,
    nxt = nx - nu;
end
if tracking>0,
    nxt = ctrl.sysStruct.dims.nx;
    if isfield(ctrl.probStruct, 'Qy'),
        nref = ctrl.sysStruct.dims.ny;
    else
        nref = ctrl.sysStruct.dims.nx;
    end
end

if isfield(ctrl.sysStruct, 'Ts'),
    Ts = ctrl.sysStruct.Ts;
else
    Ts = 1;
end

abstol = mptOptions.abs_tol;

H = []; K = []; F = []; G = []; nc = [];
for ii = 1:length(Hn),
    h = Hn{ii}';
    k = Kn{ii};
    f = Fi{ii}';
    g = Gi{ii};
    H = [H; h(:)];
    K = [K; k(:)];
    F = [F; f(:)];
    G = [G; g(:)];
    nc = [nc; size(Hn{ii}, 1)];
end
