function [x,R]=facetcircle(P,ind,Options)
%FACETCIRCLE Returns largest circle inside facet 'ind' of polytope P
%
% [x,R]=facetcircle(P,ind,Options);
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Given the polyhedron P={Hx<=K}, returns the center x and the radius R of the
% largest circle inside facet 'ind' of P. Note that R indicates how 'small' the
% facet 'ind' is, while x is a good 'center' for this facet. This corresponds to
% the chebychev ball in lower dimensional stpace. The routine returns R<0 if 
% facet 'ind' is "empty", i.e. the facet is redundant.
%
% ---------------------------------------------------------------------------
% INPUT
% --------------------------------------------------------------------------- 
% P                - Polytope
% ind              - index of facet; Lower dimensional chebyball will be 
%	         	     contained in this facet.
% Options.lpsolver - LP solver to use (see help mpt_solveLP)
% Options.abs_tol  - absolute tolerance
% Options.verbose  - level of verbosity
%
% Note: If Options is missing or some of the fields are not defined, the default 
%       values from mptOptions will be used
%
% ---------------------------------------------------------------------------
% OUTPUT
% ---------------------------------------------------------------------------
%  x, R   -  Center and radius of the largest circle inside the given facet
%
% see also CHEBYBALL
%

% ---------------------------------------------------------------------------
% Copyright is with the following author(s):
%
% (C) 2005 Miroslav Baric, Automatic Control Laboratory, ETH Zurich,
%          baric@control.ee.ethz.ch  
% (C) 2003-2005 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%               kvasnica@control.ee.ethz.ch
% (C) 2003 Mato Baotic, Automatic Control Laboratory, ETH Zurich,
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
% ---------------------------------------------------------------------------

% 
global mptOptions;

error(nargchk(2,3,nargin));

if ~isa(P,'polytope')
    error('ANALYTICCENTER: First input argument must be a polytope!');
end

if nargin<3,
    Options=[];
end

if ~isfield(Options,'abs_tol')
    abs_tol=mptOptions.abs_tol;    % absolute tolerance
else
    abs_tol = Options.abs_tol;
end
if ~isfield(Options,'lpsolver')
    lpsolver=mptOptions.lpsolver;
else
    lpsolver = Options.lpsolver;
end
if ~isfield(Options, 'verbose'),
    Options.verbose = mptOptions.verbose;
end

if length(P.Array)>0,
    error('FACETCIRCLE: this function does not work for array of polytopes!');
end

A=P.H;
B=P.K;
xCheb = P.xCheb;
rCheb = P.RCheb;

[nb,n] = size(A);

if(isempty(A) | isempty(B))
    error('FACETCIRCLE: Constraint matrix is empty')
end

if ( ind > nb ),
    error('FACETCIRCLE: Specified facet index exceed the number of facets.');
end

% project Chebyshev center onto the facet
%
facetH    = A(ind,:);
facetK    = B(ind,:);
facetDist = facetK - facetH * xCheb;
normFacet = norm(facetH);
x0 = xCheb + facetDist * facetH'/normFacet;

Hi0 = null(facetH);
idxOther = 1:nb;
idxOther(ind) = [];
Afacet = A(idxOther,:);
normFacets = sqrt(diag(Afacet * Afacet'));
auxH = [Afacet*Hi0 normFacets];
auxK = B(idxOther,:) - Afacet * x0;

% obtain list of available LP solvers
solvers = mptOptions.solvers.lp_all;

% delete current solver from it's position among prefered solvers
solvers(find(solvers==lpsolver)) = [];

% move current solver to the top
solvers = [lpsolver solvers];

for ii = 1:length(solvers),
    % try other solvers
    [xopt,fval,lambda,exitflag,how]=mpt_solveLP([zeros(1,n-1) -1],auxH,auxK,[],[],[], solvers(ii));
    
    x = x0 + Hi0 * xopt(1:end-1); % Chebyshev center of the facet
    R = xopt(end);                % Chebyshev radius of the facet
    
    problem_infeasible = strcmpi(how, 'infeasible');
    
    % NOTE! before we have been checking (R < -abs_tol) here, but it seems that
    % this is not tight enough. sometimes (especially CDD) LP solver gives R
    % -1e-16, hence negative, but not catched by the old condition. and since in
    % mpt_mpqp we check (R < 0), we use the same tolerance here.
    problem_negativeR = (R < 0);  
    problem_constraints = (max(A*x - B) > abs_tol);

    if problem_infeasible | problem_negativeR | problem_constraints,
        % use different solver in case of problems
        if Options.verbose > 1,
            if ii > 1,
                % finish new line from previous output
                fprintf('\n');
            end
            if ii < length(solvers)
                fprintf('FACETCIRCLE: Solver "%s" failed, trying "%s"...', ...
                    mpt_solverInfo('lp', solvers(ii)), mpt_solverInfo('lp', solvers(ii+1)));
            else
                fprintf('FACETCIRCLE: Last chance solver "%s" failed, panic.\n', mpt_solverInfo('lp', solvers(ii)));
            end
        end
    else
        % otherwise we have a feasible solution
        if ii > 1 & Options.verbose > 1,
            fprintf(' success\n');
        end
        break
    end
end

if problem_infeasible | problem_negativeR | problem_constraints,
    fprintf('FACETCIRCLE: All available LP solvers failed:\n');
end
if problem_infeasible,
    % try different LP solver
    disp('FACETCIRCLE:   ERROR: No feasible solution found (1)');
    return;
elseif problem_negativeR,
    disp('FACETCIRCLE:   ERROR: Numerical problems with LP solver (2)');
    return;
elseif problem_constraints,
    disp('FACETCIRCLE:   ERROR: Numerical problems with LP solver (3)');
    return;
end

% project the point to the facet, this is just to annulate all numerical
% errors (never trust to an LP solver)
%
xSlack = facetK - facetH * x;
x = x + xSlack * facetH' / normFacet;
