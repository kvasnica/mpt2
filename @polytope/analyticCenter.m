function x = analyticCenter(P)
%ANALYTICCENTER Computes an analytic center of a polytope
%
% x = center(P)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Computes an analytic center of a polytope. The input polytope must be bounded
% and fully dimensional! An SDP programming based technique is used, YALMIP is
% required.
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% P        - Polytope
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% x        - Analytic center of the polytope P
%
% see also CHEBYBALL
%

% $Id: analyticCenter.m,v 1.2 2005/03/01 11:16:43 kvasnica Exp $
%
% (C) 2004 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
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

global mptOptions

if ~isa(P,'polytope')
    error('ANALYTICCENTER: Input argument must be a polytope!');
end

if ~isempty(P.Array),
    error('ANALYTICCENTER: This function does not work with polytope arrays!');
end

if ~isminrep(P)
    P = reduce(P);
end

if ~isnormal(P)
    P = normalize(P);
end

if ~isfulldim(P)
    error('ANALYTICCENTER: Polytope must be fully dimensional!');
end

if ~isbounded(P)
    error('ANALYTICCENTER: Polytope must be bounded!');
end

nx = dimension(P);
nc = nconstr(P);

xsdp = sdpvar(nx,1);
G = diag(-(P.H*xsdp-P.K));

options = mptOptions.sdpsettings;

solution=solvesdp(set(G>0), -logdet(G), options);

if solution.problem==0
    % solution is feasible
    x = double(xsdp);
elseif solution.problem==4
    % numerical problems
    warning('ANALYTICCENTER: Numerical problems');
    x = double(xsdp);
else
    error('ANALYTICCENTER: Problem is infeasible!');
end
