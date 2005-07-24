function result = isinvariant(ctrl)
%ISINVARIANT Decides if a given explicit controller is invariant
%
%   result = isinvariant(ctrl)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Returns 1 if a given explicit controller is invariant, i.e. if it satysfies
% system constraints for all time in closed loop. Returns 0 otherwise
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% ctrl   - explicit controller
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% result - 1 if the controller is invariant, 0 otherwise
%

% $Id: isinvariant.m,v 1.2 2005/03/21 22:43:49 kvasnica Exp $
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

if ~isexplicit(ctrl)
    result = -1;
    disp('ISINVARIANT: no invariance statement can be given for on-line controllers!');
    return
end

probStruct = ctrl.probStruct;
result = 0;
if probStruct.tracking>0,
    % no invariance guarantees for tracking problems
    return
elseif isfield(probStruct, 'Qy')
    % no invariance gurantees for output regulation problems
    return
end
if probStruct.norm==2 & probStruct.Tconstraint==1 & probStruct.subopt_lev==0
    % invariance guaranteed for 2-norm problems with LQR target set
    result = 1;
elseif probStruct.subopt_lev>0
    % invariance guaranteed for minimum time and low-complexity setups
    result = 1;
elseif probStruct.subopt_lev==0 & isinf(probStruct.N)
    % invariance guaranteed for infinite-time solutions
    result = 1;
elseif isfield(ctrl.details, 'isinvariant')
    % invariance also guaranteed if invariant(ctrl) has been used
    result = ctrl.details.isinvariant;
end
if isfield(ctrl.probStruct, 'Nc'),
    if ctrl.probStruct.Nc==ctrl.probStruct.N & result==1,
        return
    else
        result = 0;
    end
end
if isfield(ctrl.probStruct, 'inputblocking') | isfield(ctrl.probStruct, 'deltablocking')
    % no stability guarantees for move blocking
    result = 0;
end
