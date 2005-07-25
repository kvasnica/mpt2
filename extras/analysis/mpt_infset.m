function [Oinf,tstar,fd,isemptypoly] = mpt_infset(A,X,tmax,Pnoise,Options)
%MPT_INFSET Calculates the maximal positively invariant set for an LTI system
%
% [Oinf,tstar,fd,isemptypoly] = mpt_infset(A,X,tmax,Pnoise,Options)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Calculates the maximal positively invariant set for an autonomous discrete-time
% LTI system
% Takes into account polytopic and additive system uncertainty, if defined
% in "sysStruct" / "Pnoise", respectively.
%
% ---------------------------------------------------------------------------
% INPUT
% --------------------------------------------------------------------------- 
% A                - The A matrix of the discrete-time LTI system x{k+1} = Ax{k}.
% X                - State constraints given as a polytope (x \in X)_
% tmax             - Maximum number of iterations allowed.
% Options.lpsolver - Solver for LPs (see help mpt_solveLP for details)
% Options.verbose  - level of verbosity (see help mpt_init for details)
% Options.abs_tol  - absolute tolerance
%
% Note: If Options is missing or some of the fields are not defined, the default
%       values from mptOptions will be used
%
% ---------------------------------------------------------------------------
%   OUTPUT:
% ---------------------------------------------------------------------------
% Oinf         - Maximal positively invariant set contained in X, i.e.
%                  Oinf = {x_k \in X: x_{k+1} = Ax_k \in X}.
% tstar        - Determinedness index.
% fd           - 1 if Oinf is finitely determined (tstar <= tmax).
%                0 if Oinf tstar > tmax.
% isemptypoly  - 1 if resulting polyhedron is empty; 0 otherwise
%
% ---------------------------------------------------------------------------
%   Literature:
% ---------------------------------------------------------------------------
% "Theory and Computation of Disturbance Invariant Sets for Discrete-Time Linear Systems"
% I. Kolmanovsky and E. G. Gilbert, Mathematical Problems in Egineering, vol (4), 1998, 
% pages 317-367
%
% AND
%
% "Linear systems with state and control constraints: the theory and
% applications of maximal output admissible sets", E. G. Gilbert and K. Tin Tan,
% IEEE Transcations on Automatic Control, 1991, vol 36, number 9, pages 1008--1020
%
%
% see also MPT_INFSETPWA

% Copyright is with the following author(s):
%
% (C) 2003 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%          kvasnica@control.ee.ethz.ch
% (C) 2003 Pascal Grieder, Automatic Control Laboratory, ETH Zurich,
%          grieder@control.ee.ethz.ch
%
%  This function is based on a script by Eric Kerrigan. Thanks Eric, for letting us
%  use you code:)

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

error(nargchk(3,5,nargin));

global mptOptions;
if ~isstruct(mptOptions),
    mpt_error;
end
if(nargin<5)
    Options=[];
end
if(~isfield(Options,'lpsolver'))
    Options.lpsolver=mptOptions.lpsolver;          %0: NAG ; 1: LINPROG ; 2: CPLEX; 3:CDD
end
if(~isfield(Options,'debug_level'))
    Options.debug_level=mptOptions.debug_level;
end
if ~isfield(Options,'abs_tol'),
    Options.abs_tol=mptOptions.abs_tol;
end
if ~isfield(Options,'verbose'),
    Options.verbose=mptOptions.verbose;
end
if(nargin<4 | isempty(Pnoise))
    addNoise=0;
elseif(isfulldim(Pnoise))
    [Hnoise,Knoise]=double(Pnoise);
    addNoise=1;
    X=X-Pnoise;
else
    addNoise=0;
end
if(~isfulldim(X))
    disp('No invariant set exists')
    Oinf=polytope;
    isemptypoly=1;
    fd=1;
    tstar=0;
    return
end
if(~iscell(A))
    tmpA=A;     %make cell out of A
    clear A;
    A{1}=tmpA;
end

[H,h] = double(X);
s = length(h);
noiseVal=zeros(s,1);

HH = H;
hh = h;
t = 0;

fd = 0;
while (fd == 0) & (t+1 <= tmax)
    if Options.verbose>1,
        disp(sprintf('\nCalculating O_%d, tmax = %d',t+1,tmax));
    end
    fd = 1;
    for k=1:length(A) %got through all dynamics in cell array A
        Hnew = H*A{k}^(t+1);
        for i = 1:s
             if(~addNoise)
                %do nothing
             else
               HAnew=H*(A{k}^t);
               %LPi%[xopt,fval,lambda,exitflag,how]=mpt_solveLPi(HAnew(i,:)',Hnoise,Knoise,[],[],[],Options.lpsolver);
               [xopt,fval,lambda,exitflag,how]=mpt_solveLPi(HAnew(i,:),Hnoise,Knoise,[],[],[],Options.lpsolver);
               noiseVal(i)=noiseVal(i)+HAnew(i,:)*xopt;
             end
             isred=sub_isredundant([HH; Hnew(i,:)],[hh; h(i)+noiseVal(i)],length(hh)+1,Options.abs_tol,Options.lpsolver);
             if(isred==0)
                fd = 0; % if new constraint not redundant then not finished yet
                HH = [HH; Hnew(i,:)]; % and add new constraint to previous constraints
                hh = [hh; h(i)+noiseVal(i)];
             elseif(isred==2)
                disp('mpt_infset: No invariant set exists');   
                Oinf=polytope;
                isemptypoly=1;
                tstar=t;
                return
             end
        end
    end
    t = t + 1;
end

if Options.verbose>1,
    disp(sprintf('\nFinished. Final check for and removal of redundant inequalities...\n'))
end
Oinf = polytope(HH,hh);
isemptypoly = ~isfulldim(Oinf);

if fd == 1
  tstar = t-1;
else
  tstar = t;
end

if(t+1>=tmax)
    disp('mpt_infset WARNING: ITERATION TERMINATED BECAUSE OF THE MAX LOOP COUNTER. THE SET MAY NOT BE INVARIANT.')
end

%===========================================================================
function [how] = sub_isredundant(A,b,row,tolerance,lpsolver)
% [how] = ifa_isredundant(A,b,row,tolerance,lpsolver)
%
%+++++++++++++++++++++++++++++++++++++++++++
% Syntax
%
%    [how] = isredundant(A,b,row,tolerance,lpsolver)
%
% Description
%   
%    Check if constraint defined by f*x<=g (f=A(row,:), g=b(row)) isredundant.
%
% where
%    f,g   - additional constraint,
%    A,b   - original problem.
%    how   = 1 (TRUE) redundant
%          = 0 (FALSE) non-redundant
%          = 2 empty polyhedron.
%
% (C) 2002 by Pascal Grieder, modifications/bug fix
% (C) 2001 by M. Baotic, Zurich, 21.03.2001
% Version 1.0
%+++++++++++++++++++++++++++++++++++++++++++

% nobody is perfectzz
if nargin<4 | isempty(tolerance),
   tolerance=1e-6; % default
end
if nargin<5,
   lpsolver=0; %E04MBF.M
end

[m,n]=size(A);

if m ~= size(b,1) | row > m | row < 1,
   error('mpt_infset: Bad sizes in ISREDUNDANT.'); 
end

%LPi%f  = A(row,:)';
f = A(row,:);

ii = [1:row-1, row+1:m];

if ~isempty(ii),
   [xopt,fval,lambda,exitflag,status]=mpt_solveLPi(-f,A(ii,:),b(ii),[],[],[],lpsolver);
   %LPi%obj=f'*xopt-b(row);
   obj=f*xopt-b(row);
else
   status='unbounded';
end

if strcmp(status,'unbounded') | (obj>tolerance & strcmp(status,'ok')),  
   how=0; % non-redundant
elseif(strcmp(status,'infeasible'))
   %double check to make sure that the problem is really infeasible
   [x,R]=chebyball(polytope(A(ii,:),b(ii),0,2));
   if(R<tolerance)
     how=2; % infeasible
   else
     how=1;
   end   
else 
   how=1; % redundant
end


return
