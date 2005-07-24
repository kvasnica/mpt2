function [flag, N, Vreach] = mpt_verify(object, X0, Xf, N, U0, Options)
%MPT_VERIFY Verifies if states enter a given set in a given number of steps
%
% [flag, N, Vreach] = mpt_verify(sysStruct, X0, Xf, N, U0)
% [flag, N, Vreach] = mpt_verify(controller, X0, Xf, N)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Checks if states of a dynamical system subject to:
%  1. inputs driven by an explicit control law
%  2. inputs which belong to a set of admissible inputs U0
% enter a given set Xf, assuming x0 \in X0 in N steps.
%
% USAGE:
%   flag = mpt_verify(sysStruct, X0, Xf, N, U0)
%   flag = mpt_verify(controller, X0, Xf, N)
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% object       - either a sysStruct structure, or an explicit controller
% X0           - set of initial states (a polytope or a polyarray)
% Xf           - target set (a polytope or a polyarray)
% N            - number of steps over which to compute reachable sets
% U0           - set of admissible inputs (polytope object)
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% flag        - 1 if Xf is reachable from X0, 0 otherwise
% N           - number of steps in which Xf is reachable from X0 ([] if Xf is
%               not reachable)
% Vreach      - V-representation of reachable sets
%
% see also MPT_REACHSETS, MPT_REACHXU
%

% $Id: mpt_verify.m,v 1.1 2005/03/13 16:56:37 kvasnica Exp $
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

error(nargchk(4,6,nargin));

if isa(object, 'mptctrl'),
    if ~isexplicit(object),
        error('This function supports only explicit controllers!');
    end
end

if ~isa(X0, 'polytope'),
    error('Set of initial states must be a polytope!');
end
if ~isa(N, 'double'),
    error('Number of steps must be a positive integer!');
end
if N<1 | any(size(N)~=1),
    error('Number of steps must be a positive integer!');
end
if ~isa(Xf, 'polytope'),
    error('Set of final states must be a polytope!');
end
if ~isfulldim(X0),
    error('Set of initial states must be a fully dimensional polytope!');
end
if ~isfulldim(Xf),
    error('Set of final states must be a fully dimensional polytope!');
end

Options.Xf = Xf;

if nargin==4,
    [res, Vreach] = mpt_reachSets(object, X0, N, Options);
else
    if ~isa(U0, 'polytope'),
        error('Set of admissible inputs must be a polytope!');
    end
    if ~isfulldim(U0),
        error('Set of admissible inputs must be a fully dimensional polytope!');
    end
    [res, Vreach] = mpt_reachSets(object, X0, U0, N, Options);
end

if isa(res, 'double'),
    N = res;
    flag = 1;
else
    N = [];
    flag = 0;
end