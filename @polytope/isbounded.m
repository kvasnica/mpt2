function status = isbounded(P,Options)
%ISBOUNDED Checks if a polytope is bounded
%
% status = isbounded(P,Options)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% STATUS = ISBOUNDED(P) returns TRUE (1) if P is a bounded polytope
%
% Check is performed using Minkowski theorem on polytopes.
%
% USAGE:
%   isbounded(P)
%   isbounded(P,Options)
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% P                - polytope
% Options.abs_tol  - absolute tolerance
% Options.lpsolver - LP solver to use
% Options.verbose  - level of verbosity
%
% Note: If Options is missing or some of the fields are not defined, the default
%       values from mptOptions will be used
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% status           - Logical statement
%
% see also ISFULLDIM
%

% $Id: isbounded.m,v 1.2 2005/04/04 09:22:21 kvasnica Exp $
%
% (C) 2003 Miroslav Baric, Automatic Control Laboratory, ETH Zurich,
%          baric@control.ee.ethz.ch
% (C) 2003 Mato Baotic, Automatic Control Laboratory, ETH Zurich
%          baotic@control.ee.ethz.ch

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


% if ~isa(P, 'polytope')
%   error('ISBOUNDED: Argument MUST be a polytope object');
% end

global mptOptions;

if nargin<2
    Options=[];
end

if ~isstruct(mptOptions),
    mpt_error;
end

if ~isfield(Options,'abs_tol')
    Options.abs_tol=mptOptions.abs_tol;      % absolute tolerance
end
if ~isfield(Options,'lpsolver'),
    Options.lpsolver = mptOptions.lpsolver;  % LP solver to use
end
if ~isfield(Options,'verbose'),
    Options.verbose = mptOptions.verbose;    % level of verbosity
end

lenP = length(P.Array);
if lenP>1,
    for ii=1:lenP,
        Q = P.Array{ii};
        status = isbounded(Q,Options);
        if status==0,
            return
        end
    end
else
    [ChebyC,ChebyR] = chebyball(P,Options);
    
    if ChebyR == Inf,   % don't trust this one too much
        status = 0;
        return;
    elseif ChebyR <= Options.abs_tol,
        status = 1;
        return;
    end;
    
    A = P.H;
    b = P.K;
    
    % check the rank of matrix of vector normals
    %
    [n,m] = size(A);
    
    if m >= n,     % we have more variables than constraints: unbounded
        status = 0;
        return;
    end;
    
    if rank(A) < m,
        status = 0;
        return;
    end;
    
    AA = -eye(n);
    bb = -ones(n,1);
    %LPi%f = zeros(n,1);
    f = zeros(1,n);
    [xopt,fval,lambda,exitflag,how] = mpt_solveLPi(f,AA,bb,A',zeros(m,1),[],Options.lpsolver);
    
    if how(1)=='i' | how(1)=='I',   % problem is infeasible
        status = 0;                 %  => polyhedron is unbounded
        return;
    elseif strcmp(how,'ok')         % problem is feasible (unconstrained case
        status = 1;                   % cannot actually happen
        return;                       %  => polyhedron is bounded
    else
        if Options.verbose>0,
            disp('ISBOUNDED: Problem detected!');
        end
        status=0;
    end
end
