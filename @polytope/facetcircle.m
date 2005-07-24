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
% $Id: facetcircle.m,v 1.2 2004/12/04 13:03:04 kvasnica Exp $
%
% (C) 2003 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%          kvasnica@control.ee.ethz.ch
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

% error(nargchk(2,3,nargin));
% 
global mptOptions;

% if ~isstruct(mptOptions)
%     mpt_error;
% end

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

if length(P.Array)>0,
    error('FACETCIRCLE: this function does not work for array of polytopes!');
end


A=P.H;
B=P.K;

% nb=size(B,1);
% n=size(A,2);
% if nb~=size(A,1),
%    error('FACETCIRCLE: The H and K matrices must have the same number of rows.')
% end

[nb,n] = size(A);

if(isempty(A) | isempty(B))
    error('FACETCIRCLE: Constraint matrix is empty')
end

ii=[1:ind-1,ind+1:nb];
Aeq=A(ind,:);
Beq=B(ind);
A=A(ii,:);
B=B(ii);

Tnorm=zeros(nb-1,1);
aux1=Aeq*Aeq';
for i=1:nb-1,
   aux2=A(i,:)*Aeq';
   Tnorm(i)=sqrt(abs(A(i,:)*A(i,:)'-aux2*aux2/aux1)); % abs because of rounding error (-zero) case
end

% use 'rescue' function - resolve an LP automatically if it is infeasible
% with default solver
[xopt,fval,lambda,exitflag,how]=mpt_solveLPi([zeros(1,n) 1],[A,-Tnorm],B,[Aeq,0],Beq,[],lpsolver,1);

if ~strcmp(how,'ok')
    % problem infeasible with default solver, re-solve it using initial values for x
    x0 = [zeros(n,1); 1000];         % hard-coded initial conditions
    [xopt,fval,lambda,exitflag,how]=mpt_solveLPi([zeros(1,n) 1],[A,-Tnorm],B,[Aeq,0],Beq,x0,lpsolver,1);
end

R=-xopt(n+1); % This is radius of the ball
x=xopt(1:n);  % This is center of the ball

if strcmp(how,'infeasible'),
      disp('FACETCIRCLE:   ERROR: Facet problem should be feasible (1)');
elseif(R<-abs_tol)
      disp('FACETCIRCLE:   ERROR: Facet problem should be feasible (2)');
elseif(max(A*x-B)>abs_tol)
      disp('FACETCIRCLE:   ERROR: Facet problem should be feasible (3)');
end
