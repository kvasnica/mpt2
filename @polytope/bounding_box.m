function [R,l,u,lv,uv]=bounding_box(P,Options,lookahead,A,b)
%BOUNDING_BOX Compute a bounding box for a given polytope
%
% [R,l,u,lv,uv]=bounding_box(P,Options,lookahead,A,b)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Given a polytope P={x: H*x<=K}, compute the bounding box. A bounding box
% is the smallest hypercube which contains the given polytope.
% 
% The algorithm uses LP to compute maximal hyperrectangle containing a 
% polytope
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% P                     - polytope
% Options.lpsolver      - LP solver to use (see help mpt_solveLP)
% Options.noPolyOutput  - if set to 1, the bounding box will NOT be returned as
%                         a polytope object
% lookahead         - specify at which future iteration to compute the box, if
%                     the dynamics given by (A,b) are applied. Set 0 by default.
% A,b               - System dynamics  x(k+1)=Ax+b, only needed if lookahead > 0;
%
% Note: If Options is missing or some of the fields are not defined, the default
%	values from mptOptions will be used
%
% ---------------------------------------------------------------------------
% OUTPUT
% ---------------------------------------------------------------------------
% R                 - Bounding Box Polytope (=Hypercube)
% "l,u"             - The two extreme vertices of the hypercube R, i.e. "l" is 
%		      the vertex which minimizes the LP objective "x" (smallest
%		      state) and "u" is the vertex which minimizes the LP 
%		      objective "-x" (largest state).
% "lv,uv"           - extreme points if the original polytope
%                     LV(i,:) is the argmin(x_i), x \in P 
%                     UV(i,:) is the argmax(x_i), x \in P 
%
% see also ENVELOPE, HULL, UNION

% ---------------------------------------------------------------------------
% $Id: bounding_box.m,v 1.1.1.1 2004/11/24 10:09:57 kvasnica Exp $
%
% (C) 2003 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%          kvasnica@control.ee.ethz.ch
% (C) 2003 Pascal Grieder, Automatic Control Laboratory, ETH Zurich,
%          grieder@control.ee.ethz.ch
% (C) 2002 by F.D. Torrisi, Zurich, 2002/08/16
% (C) 2000 by F.D. Torrisi, Siena, 2000/01/10
% (C) 1999 by A. Bemporad and F.D. Torrisi, Zurich, September 22, 1999

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

error(nargchk(1,5,nargin));

global mptOptions;
if nargin<2
    Options=[];
    if ~isstruct(mptOptions),
        mpt_error;
    end
end
if ~isfield(Options,'lpsolver')
    Options.lpsolver=mptOptions.lpsolver;
end
if ~isfield(Options,'noPolyOutput'),
    % if set to 1, the bounding box will NOT be returned as a polytope object
    Options.noPolyOutput = 0;
end
if(nargin<3)
    lookahead=0;
end

if length(P.Array)>0
    error('bounding_box: This function does not support polyarrays!');
end

if ~isempty(P.bbox) & lookahead==0 & nargout < 4,
    % quick return if bounding box was already computed and stored
    l = P.bbox(:,1);
    u = P.bbox(:,2);
    n = size(P.H,2);
    if Options.noPolyOutput == 0,
        R=polytope([eye(n); -eye(n)],[u;-l]);
    else
        R=[];
    end
    return
end

%Initialize Values
[m,n] = size(P.H);

In=eye(n);
l=zeros(n,1);               % Lower bounds
u=zeros(n,1);               % Upper bounds


%Build Objective Matrices
if(lookahead>0)
        Bb=zeros(n,1);
        for j=0:lookahead-1,
          Bb=Bb+A^((lookahead-1)-j)*b;
        end
      Aa=A^lookahead;
else
      Bb=zeros(n,1);  
      Aa=eye(n);
end

% Determine external box ; minimize x_i
for i=1:n,
   if(lookahead==0)
       f=In(:,i);
   else
       f=Aa;
       f=f(i,:)';
   end
   [x,fval,lambda,exitflag,how]=mpt_solveLPi(f(:)',P.H,P.K,[],[],[],Options.lpsolver);
   x=Aa*x+Bb;
   l(i)=x(i);
   lv(i,:) = x(:)'; 
end
   
% Determine external box ; maximize x_i
for i=1:n,
   if(lookahead==0)
       f=-In(:,i);
   else
       f=Aa;
       f=-f(i,:)';
   end
   [x,fval,lambda,exitflag,how]=mpt_solveLPi(f(:)',P.H,P.K,[],[],[],Options.lpsolver);
   x=Aa*x+Bb;
   u(i)=x(i);
   uv(i,:) = x(:)'; 
end

%compute polytope
if Options.noPolyOutput == 0,
    R=polytope([eye(n); -eye(n)],[u;-l]);
    R.bbox = [l u];
else
    R=[];
end
