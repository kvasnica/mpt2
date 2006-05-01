function [x0, dumode] = extendx0(ctrl, x0, uprev, reference)
%EXTENDX0 Extends the initial state if needed
%
% [x0, dumode] = extendx0(ctrl, x0, uprev, reference)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% If tracking and/or deltaU constraints have been used to design a particular
% controller, the state vector needs to be augmented in order to be able to deal
% with these goals in closed-loop. This implies that the value of the initial
% state must also be augmented accordingly when a given controller is evaluated
% by
%
%   u = ctrl(x0)
%
% In order to automate this task, one can call this helper as follows:
%
%   [x0, dumode] = extendx0(ctrl, x0, uprev, reference)
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% ctrl       - MPT controller object
% x0         - Current measured state
% uprev      - Previous value of the control action u(k-1)
% reference  - Reference trajectory (if tracking was used)
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% x0         - The augmented state vector
% dumode     - If true, the controller returns deltaU=u(k)-u(k-1), i.e. the
%              "true" control action must be additionaly obtained by
%                utrue = u + uprev
%

% Copyright is with the following author(s):
%
% (C) 2006 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
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

x0 = x0(:);
sysStruct = ctrl.sysStruct;
probStruct = ctrl.probStruct;
include_uprev = 0;
include_ref = 0;

%=====================================================================
% first determine what is the needed dimension of x0
if isexplicit(ctrl),
    x0_required = dimension(ctrl.Pn);
    
elseif isfield(ctrl.details, 'yalmipMatrices'),
    x0_required = size(ctrl.details.yalmipMatrices.E, 2);
    if isfield(ctrl.details.yalmipMatrices, 'uprev_length'),
        x0_required = x0_required - ctrl.details.yalmipMatrices.uprev_length;
    end
    if isfield(ctrl.details.yalmipMatrices, 'reference_length'),
        x0_required = x0_required - ctrl.details.yalmipMatrices.reference_length;
    end
    
elseif isfield(ctrl.details, 'yalmipData'),
    x0_required = length(ctrl.details.yalmipData.vars.x{1})
    if isfield(ctrl.details.yalmipData, 'uprev_length'),
        x0_required = x0_required - ctrl.details.yalmipData.uprev_length;
    end
    if isfield(ctrl.details.yalmipData, 'reference_length'),
        x0_required = x0_required - ctrl.details.yalmipData.reference_length;
    end
    
elseif isfield(ctrl.details, 'Matrices'),
    if ~isempty(ctrl.details.Matrices),
        x0_required = size(ctrl.details.Matrices.E, 2);
    else
        x0_required = ctrl.details.dims.nx;
    end
    
else
    error('Unknown controller.');
    
end

%---------------------------------------------------------------------
% do we need to include u(k-1) ?
include_uprev = (probStruct.tracking==0 & isfield(sysStruct, 'dumode')) | ...
    (probStruct.tracking==1 & (isfield(probStruct, 'tracking_augmented') | ...
    isexplicit(ctrl))) | isfield(ctrl.details, 'uprev_in_x0');
dumode = include_uprev;

%---------------------------------------------------------------------
% do we need to include the reference
include_ref = isfield(ctrl.details, 'reference_in_x0') | ((probStruct.tracking > 0) & ...
    (isfield(probStruct, 'tracking_augmented') | isexplicit(ctrl)));


%=====================================================================
% exit quickly if the state already has correct length
if length(x0) == x0_required,
    return
end


%=====================================================================
% augment the state vector if need
if include_uprev,
    x0 = [x0; uprev(:)];
end
if include_ref,
    x0 = [x0; reference(:)];
end
if length(x0) ~= x0_required,
    error('Wrong dimension of u(k-1) and/or the reference.');
end
