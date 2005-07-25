function R=minus(P,Q,Options)
%MINUS Pontryagin difference
%
% MINUS  Minkowski (Pontryagin) difference
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
%  The algorithm for efficiently computing the Minkowski difference between a
%  union of polytopes and a polytope is based on a talk by 
%  S. Rakovic and D. Mayne entitled "Constrained Control Computations"
%  It was the keynote address at the GBT Meeting, London, November, 2002.
%
% USAGE:
%   R=P-Q
%   R=minus(P,Q,Options)
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% P,Q                     - Polytopes or arrays of polytopes
% Options.extreme_solver  - Which method to use for extreme points computation
%                           (see help extreme)
% Options.lpsolver        - Which LP solver to use (see help mpt_solveLP)
%
% Note: If Options is missing or some of the fields are not defined, the default
%       values from mptOptions will be used
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% R     - polytope (or polytope array) describing the Pontryagin difference
%
% see also PLUS, MLDIVIDE
%

% Copyright is with the following author(s):
%
% (C) 2003 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%          kvasnica@control.ee.ethz.ch
% (C) 2003 Pascal Grieder, Automatic Control Laboratory, ETH Zurich,
%          grieder@control.ee.ethz.ch
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
%
% ---------------------------------------------------------------------------

global mptOptions;

if ~isa(P, 'polytope'),
    error('MINUS: First input arguement must be a polytope object!');
end

if ~isstruct(mptOptions),
    mpt_error;
end

if nargin<3,
    Options=[];
end

if ~isfield(Options,'lpsolver')
    Options.lpsolver=mptOptions.lpsolver;
end
if ~isfield(Options,'reduceMD')
    Options.reduceMD=0;
end
if ~isfield(Options,'abs_tol'),
    Options.abs_tol=mptOptions.abs_tol;
end


% MINKOWSKI DIFFERENCE
%=====================

if isa(P, 'polytope') & isa(Q, 'double')
    % special case - second argument is a matrix (possible lower-dimensional
    % polytope given as a set of it's extreme vertices) 
    
    if ~isempty(P.Array),
        
        Phull=envelope(P,Options);          %compute envelope of set (Note: it is also possible to exchange "envelope" with "hull")
        %Phull=hull(P,Options);
        
        PM=minus(Phull,Q,Options);          %Compute minkowski difference
        if(~isfulldim(PM))
            R=polytope;
            return
        end
        E=mldivide(Phull,P,Options);        %E is union of polytopes; sets inside Phull which are not covered by PA
        QM = -Q;
        if ~isfulldim(E),
            Emin = polytope(QM);
        else
            Emin=plus(E,QM,Options);            %Minkowski addition on those sets
        end
        R=mldivide(PM,Emin,Options);        %Compute final polytopes
        return
    end
    
    nx = dimension(P);
    nc = nconstr(P);
    if size(Q, 1)~=nx,
        error(sprintf('The matrix must have %d rows!', nx));
    end
    A = P.H;
    B = P.K;
    for ii = 1:nc,
        a = A(ii, :);
        b = B(ii);
        delta = min(-a*Q);
        B(ii) = B(ii) + delta;
    end
    R = polytope(A, B);
    return
end
    

if isa(P,'polytope') & isa(Q,'polytope')
    [cx,cr]=chebyball(P);
    if isinf(cr) & all(cx==0),
        R=P;
        return
    end
    lenP=length(P.Array);
    if lenP>0,
        Phull=envelope(P,Options);          %compute envelope of set (Note: it is also possible to exchange "envelope" with "hull")
        %Phull=hull(P,Options);
        
        PM=minus(Phull,Q,Options);          %Compute minkowski difference
        if(~isfulldim(PM))
            R=polytope;
            return
        end
        E=mldivide(Phull,P,Options);        %E is union of polytopes; sets inside Phull which are not covered by PA
        QM=uminus(Q);                       %Flip Polytope 
        try
            Emin=plus(E,QM,Options);            %Minkowski addition on those sets
        catch
            disp('polytope/minus: extreme point enumeration failed, trying projection...');
            Emin=plus(E,QM,struct('msolver',1));
        end
        R=mldivide(PM,Emin,Options);        %Compute final polytopes

        if Options.reduceMD,          
            Rest = mptOptions.emptypoly;
            lenEmin = length(Emin.Array);
            if lenEmin==0,
                Rest = PM & Emin;
            else
                for ii=1:lenEmin,
                    Rest = [Rest PM&Emin.Array{ii}];
                end
            end
            Opt = Options;
            Opt.PAdom = PM;
            Opt.PAcompl = Rest;
            Opt.color = ones(length(R),1);
            [PP,cm]=mpt_optMergeDivCon(R,Opt);
            %R = merge(R,struct('greedy',1,'trials',2));
            if isempty(PP.Array),
                R = PP;
            else
                regind = find(cm.Reg==1);
                R = mptOptions.emptypoly;
                for ii=1:length(regind)
                    R = [R PP.Array{regind(ii)}];
                end
            end
            return
            
            diffU=mptOptions.emptypoly;
            for ii=1:lenP,
                diffU=[diffU minus(P.Array{ii},Q,Options)];    
            end
            M = R\diffU;                                       
            [UM,how] = union(M);
            if how,
                M=UM;
            end
            R = [diffU M];
        end
        return
    end
    lenQ=length(Q.Array);
    if lenQ>0,
        R=P;
        for ii=1:lenQ,
            R = minus(R,Q.Array{ii},Options);
        end
        return
    end 
    if ~isfulldim(Q),
        R=P;
        return
    end
    if ~isfulldim(P),
        R=polytope;
        return
    end
    if ~P.minrep
        P=reduce(P);
    end
    if ~Q.minrep
        Q=reduce(Q);
    end
    [ncP,dP]=size(P.H);
    [ncQ,dQ]=size(Q.H);
    if dP~=dQ
        error('MINUS: Polytopes MUST have the same dimension in Minkowski difference');
    end
    H=P.H;
    K=P.K;
    for ii=1:ncP
        [xopt,fval,lambda,exitflag,how]=mpt_solveLPi(-P.H(ii,:),Q.H,Q.K,[],[],[],Options.lpsolver);
        K(ii)=K(ii)+fval;
    end

    R=polytope(H, K, 1);
    return;
end
