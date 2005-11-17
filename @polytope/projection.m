function [P]= projection(PA,dim,Options)
%PROJECTION Projection of a polytope or a polytope array
%
% [P] = projection(P,dim,Options)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% [P] = PROJECTION(P,DIM,OPTIONS) projects polytope P on dimensions defined
%                                 in vector 'dim'
%
% Three different algorithms can be used:
%   - Vertex enumeration/Convex hull based method
%   - Fourier-Motzkin Elimination
%   - Iterative Hull
%   - Block Elimination
%   - Equality Set Projection
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------  
% P                     - Polytope
% dim                   - Dimensions on which to project
% Options.projection=0  - Vertex enumeration/Convex-hull based method
% Options.projection=1  - Fourier-Motzkin Elimination
% Options.projection=2  - Iterative Hull
% Options.projection=3  - Block Elimination
% Options.projection=4  - Equality Set Projection (ESP)
% Options.projection=5  - Fourier-Motzkin Elimination (mex implementation)
% Options.projection=6  - Fourier-Motzkin Elimination (mex implementation) -
%                         fast but eventualy unreliable
% Options.projection=7  - Use approach based on mpLPs
%
% Note: If Options.projection is not set, best method is selected automatically
%
% Note: Options.projection can also be a vector of prefered methods
%
% ---------------------------------------------------------------------------
% OUTPUT
% ---------------------------------------------------------------------------
% P   - Projected Polytope
%

% We would like to acknowledge Sasa Rakovic from Imperial College, UK, for
% initially proposing the "iterative Hull" projection method and Colin Jones
% from Cambridge University for providing ESP and Fourier-Motzkin C
% implementation

% ---------------------------------------------------------------------------
% $Version: 1.1 $ $Date: 2005/06/13 12:30:44 $
%
% (C) 2004-2005 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%               kvasnica@control.ee.ethz.ch
% (C) 2004 Raphael Suard, Automatic Control Laboratory, ETH Zurich,
%          suardr@ee.ethz.ch
%
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
% --------------------------------------------------------------------------

error(nargchk(2,3,nargin));

global mptOptions;
if ~isstruct(mptOptions),
    mpt_error;
end

if ~isa(PA, 'polytope')
    error('PROJECTION: Argument MUST be a polytope object');
end
if nargin<3,
    Options = [];
end
if ~isfield(Options,'abs_tol')
    Options.abs_tol=mptOptions.abs_tol;    % absolute tolerance
end
if ~isfield(Options,'four_tol')
    Options.four_tol = 1e-5;
end
if ~isfield(Options,'lpsolver')
    Options.lpsolver = mptOptions.lpsolver;
end
if ~isfield(Options, 'verbose'),
    Options.verbose = mptOptions.verbose;
end

if ~isfulldim(PA),
    % exit quickly if input polytope is not fully dimensional
    P = mptOptions.emptypoly;
    return
end

d=dimension(PA);
dim = dim(:)';
orig_dim = dim;
dim = setdiff(1:d,dim);   % from this point, dim denotes dimension to eliminate

if isempty(orig_dim),
    error('PROJECTION: Projection dimensions cannot be empty.');
end
if length(orig_dim) > d,
    error('PROJECTION: Dimensions must agree.');
end
if any(orig_dim > d) | any(orig_dim < 1)
    error('PROJECTION: Projection dimensions exceed polytope dimension.');
end

if ~isfield(Options,'psolvers')
    if length(orig_dim)<=2,
        % use iterative hull for lower dimensions
        Options.psolvers = [2 5 7 3 1 4 0];
    elseif length(dim)<=2,
        % only 1 or 2 dimensions to eliminate -> use fourier-motzkin
        Options.psolvers = [5 1 4 7 2 3 0];
    elseif length(dim)<=d/2
        if dimension(PA)<=3,
            % use block-elimination for lower dimensions
            Options.psolvers = [3 0 5 7 2 1 4];
        else
            Options.psolvers = [5 7 4 3 1 2 0];
        end
    else
        Options.psolvers = [5 7 2 3 4 1 0];
    end
end
if isfield(Options,'projection')
    Options.psolvers = Options.projection;
end

if ~isfield(Options,'noReduce'),
    Options.noReduce = 0;
end

p_strings = {'Vertex Enumeration',...
        'Fourier-Motzkin (Matlab version)',...
        'Iterative Hull',...
        'Block Elimination',...
        'ESP',...
        'Fourier-Motzkin (mex version)',...
        'Fourier-Motzkin (mex version, fast)' };

if (isempty(PA.Array) & ~PA.minrep & Options.noReduce==0)
    PA = reduce(PA);
end

if isempty(PA.Array),
    % make a polytope array out of a single polytope
    PA.Array{1} = PA;
end

P = mptOptions.emptypoly;

for reg = 1:length(PA.Array),
    for ii = 1:length(Options.psolvers),
        try
            switch Options.psolvers(ii),
                case 0,
                    % vertex enumeration + convex hull
                    Q = sub_extreme_hull(PA.Array{reg},orig_dim,Options);
                case 1,
                    % matlab implementation of fourier-motzkin
                    Q = sub_fouriermotzkin(PA.Array{reg},orig_dim,dim,Options);
                case 2,
                    % iterative hull
                    if nconstr(PA.Array{reg})<=dimension(PA.Array{reg}),
                        % skip iterativehull if only one constraint - otherwise
                        % an infinite loop can occur
                        continue
                    end
                    Q = sub_iterativehull(PA.Array{reg},orig_dim,dim,Options);
                case 3,
                    % projection-cone method
                    Q = sub_blockelimination(PA.Array{reg},orig_dim,dim,Options);
                case 4,
                    % ESP method
                    Q = mpt_esp(PA.Array{reg},orig_dim);
                case 5,
                    % use mex implementation of fourier-motzkin elimination (safe
                    % version with eliminating one dimension after each other)
                    Opt = Options;
                    Opt.oneshot = 0;
                    Q = sub_mexfourier(PA.Array{reg}, orig_dim, Opt);
                case 6,
                    % use mex implementation of fourier-motzkin elimination (fast
                    % version with eliminating all dimensions at the same time)
                    Opt = Options;
                    Opt.oneshot = 1;
                    Q = sub_mexfourier(PA.Array{reg}, orig_dim, Opt);
                case 7,
                    % use mpLP to solve projection
                    Q = sub_mplp_proj(PA.Array{reg}, orig_dim, dim, Options);
                otherwise,
                    error('PROJECTION: unknown ptojection method selected!');
            end
            if ~isbounded(Q),
                if isbounded(PA.Array{reg}),
                    % if P is bounded, projection of P must be bounded as well
                    error('PROJECTION: projected polytope must be bounded!');
                end
            end
            P = [P Q];
            break
        catch
            if Options.verbose > 1 & length(Options.psolvers) > ii,
                disp(['PROJECTION: ' p_strings{Options.psolvers(ii)+1} ' failed, trying ' p_strings{Options.psolvers(ii+1)+1} '...']);
            elseif Options.verbose > 1,
                disp(['PROJECTION: ' p_strings{Options.psolvers(ii)+1} ' failed...']);
            end 
        end
    end
end

if ~isfulldim(P),
    % projection couldn't be computed
    fprintf('\nLast error: %s\n',lasterr);
    if length(Options.psolvers)==1,
        error('PROJECTION: couldn''t compute projection, selected method failed!');
    else
        error('PROJECTION: couldn''t compute projection, all methods failed!');
    end
elseif length(P) < length(PA),
    fprintf('PROJECTION: projection of %d polytopes failed\n', length(PA)-length(P));
end


%----------------- sub-functions ---------------------------------------

function P = sub_mplp_proj(P, orig_dim, dim, Options)

% solve projection by multi-parametric programming.
% note that an mpLP is stated as:
%
%   min    H U + F x
%   s.t.   G U <= W + E x
%
% if we consider that the polytope to project is written as
%   C*y + D*x <= B
% where C=H(:, dim), D=H(:, orig_dim), B=K, then we can use
% G = C, W = B, E = -D and solve the problem as an mpLP. this would give as two
% things:
%   1. critical regions which we don't need
%   2. characterization of the feasible set, which is exactly equal to the
%   projection of the ([-E G],W) polytope onto the x-space

Opt = Options;
Opt.verbose = -1;    % to keep mpt_mplp() silent
[H, K] = double(P);
M.G = H(:, dim);
M.E = -H(:, orig_dim);
M.W = K;
M.bndA = [];
M.bndb = [];
M.H = rand(1, length(dim));  % use random cost to avoid degeneracies
M.F = zeros(1, length(orig_dim));
[Pn, Fi, Gi, AC, Phard] = mpt_mplp(M, Opt);
if ~isfulldim(Phard)
    % sometimes we can return Phard as an empty polytope if there are missing
    % regions on the boundary
    error('PROJECTION: mplp approach failed.');
else
    P = Phard;   % projection is equal to the feasible set
end


%-------------------------------------------------------------------
function P = sub_mexfourier(P, orig_dim, Options)

if Options.noReduce==0,
    if any(~isminrep(P)),
        P = reduce(P);
    end
end

Pdim = dimension(P);
dim = setdiff(1:Pdim, orig_dim);

% first reorder the H matrix such that dimensions to which we want to project
% become first. e.g. if we want to project on [3 1] and we have a polytope in
% 4D, the new H matrix will consists of 3rd, 1st, 2nd and 4th column of the old
% H matrix. otherwise we would need to fiddle around when calling fourier().

H = P.H(:, [orig_dim dim]);
K = P.K;
ndim = length(orig_dim);
orig_dim = 1:ndim;
dim = ndim+1:Pdim;

if Options.oneshot,
    % compute the projection in one shot (fast but unreliable!)
    HK = [H K];
    projHK = fourier(HK, orig_dim, Options.four_tol);
    if isempty(projHK),
        error('Projected polytope is R^n');
    end
    if Options.noReduce,
        % user doesn't want to remove redundant constraints
        P = polytope(projHK(:, 1:end-1), projHK(:, end), 0, 2);
    else
        P = polytope(projHK(:, 1:end-1), projHK(:, end));
    end
    return
end

% compute the projection by eliminating one dimension after each other (slow but
% quite safe).

% perturb the H-representation by a random value to prevent numerical problems
xrand = randn(size(H,2),1)*10;
HK = [H K+H*xrand];

for qq=length(dim):-1:2,
    % project one dimension at a time
    D = fourier(HK,[orig_dim dim(1:qq-1)],Options.four_tol);
    if isempty(D),
        error('Projected polytope is R^n');
    end
    P = polytope(D(:,1:end-1), D(:,end));
    H = P.H;
    K = P.K;
    HK = [H K];
end
D = fourier(HK,orig_dim,Options.four_tol);
if isempty(D),
    error('Projected polytope is R^n');
end
H = D(:,1:end-1);
K = D(:,end);
K = K - H*xrand([orig_dim]);
if Options.noReduce,
    % user doesn't want to remove redundant constraints    
    P = polytope(H, K, 0, 2);
else
    P = polytope(H, K);
end


%-------------------------------------------------------------------
function P = sub_extreme_hull(P,dim,Options)

Vert=extreme(P,Options);
P=hull(Vert(:, dim),Options);
return


%-------------------------------------------------------------------
function P = sub_fouriermotzkin(P,orig_dim,dim,Options)

% Fourier-Motzkin Elimination
if ~isminrep(P),
    P = reduce(P);
end

for i=dim  
    %%% d=dimension(P);
    positive=find(P.H(:,i) > 0);
    negative=find(P.H(:,i) < 0);
    null=find(P.H(:,i) == 0);
    
    nr=length(null) + length(positive)*length(negative);
    nc=nconstr(P);
    C=sparse(zeros(nr,nc));
    
    % Matrix C for all combinations of inequalities
    % Find a matrix C so that Pnew.H=C*P.H and Pnew.H(:,i)=[]
    A=P.H(:,i);
    row=1;
    for j=(positive)'
        for k=(negative)'       
            C(row,j)=-A(k);
            C(row,k)=A(j);
            row=row+1;            
        end
    end 
    
    for j=(null)'    
        C(row,j)=1;
        row=row+1;
    end
    
    % Compute new Matrix P.H and P.K
    P.H=C*P.H;
    P.K=C*P.K;

    P.minrep=logical(0);
    if Options.noReduce,
        % user doesn't want to remove redundant constaints
    else
        P=reduce(P); 
    end
end

P.bbox = [];
P.H = P.H(:, orig_dim);
if Options.noReduce,
    % user doesn't want to remove redundant constaints
    P=polytope(P.H,P.K,0,2);
else
    P=polytope(P.H,P.K);
end
return;
    
%-------------------------------------------------------------------
function P = sub_iterativehull(P,ydim,dim,Options)

% Iterative Hull

xdim = 1:dimension(P);              % Dimension of polytope
lpsolver = Options.lpsolver;
abs_tol = Options.abs_tol;

if length(ydim)==1
    
    f1=zeros(1,length(xdim));
    f1(ydim)=1;
    f2=-f1;
    
    xopt1=mpt_solveLPi(f1,P.H,P.K,[],[],[],lpsolver);
    Vert(1,:)=xopt1';
    xopt2=mpt_solveLPi(f2,P.H,P.K,[],[],[],lpsolver);
    Vert(2,:)=xopt2';
    
    P=hull(Vert(:,ydim),Options);
    return;
    
else
    
    % Find initial Simplex mit LP in random Directions
    OK=0;
    ind=1;
    Vert=[];
    cnt = 0;
    while ~OK
        f1=randn(1,length(ydim));
        f=zeros(1,length(xdim));
        f(ydim)=f1;
        
        xopt = mpt_solveLPi(f,P.H,P.K,[],[],[],lpsolver);
        
        Vert(ind,:)=xopt';
        Vert1=Vert(:,ydim);
        Vert1=round(Vert1/abs_tol)*abs_tol;
        Vert1=unique(Vert1,'rows');
                
        m=size(Vert,1);
        m1=size(Vert1,1);
        if m==m1
            ind=ind+1;
        else
            Vert(ind,:)=[];     % In matrix Vert are 3 the Eckpunkte of the Polytope
        end 
        
        if m1==(length(ydim)+1)     
            OK=1;     
        end 
    end 
    
    tmpOpt.abs_tol=abs_tol*10; %to compensate for rounding errors
    
    P1=hull(Vert(:,ydim));        
    HP=[];
    % At every loop
    % 1. max in every direction => we have N new extrema
    % 2. hull compute with all the extrema
    % 3. hull(new) ~= hull(old) => go to point 1 ; else, return. 
    while 1
        for ind=1:size(P1.H,1)
            
            f1=round(P1.H(ind,:)/abs_tol)*abs_tol;
            f2=[round(P1.H(ind,:)/abs_tol)*abs_tol, round(P1.K(ind,:)/abs_tol)*abs_tol];
            
            k=[];
            if ~isempty(HP)
                k = find( HP(:,1) == f2(1) );
                for j = 2:size(P1.H, 2)+1
                    k = k( HP(k,j) == f2(j) );
                    if isempty(k)
                        break
                    end
                end
            end
            
            if length(k)==1
                xopt = HP(k,size(P1.H,2)+2 : size(P1.H,2)+size(Vert,2)+1);
            else
                f=zeros(1,length(xdim));
                f(ydim)=f1;
                [xopt]=(mpt_solveLPi(-f,P.H,P.K,[],[],[],lpsolver))';
                
                HP(size(HP,1)+1,:)=[f1, round(P1.K(ind)/abs_tol)*abs_tol, round(xopt/abs_tol)*abs_tol];
            end
            Vert=[Vert; xopt];
            
        end
        
        P2=hull(Vert(:,ydim),Options);
        
        OK=1;
        for i=1:size(Vert,1)
            if ~isinside(P1,(Vert(i,ydim))',tmpOpt)
                OK=0;
                break;
            end
        end
        
        if(OK==1)
            P=P2;
            return;
        else
            P1=P2;
        end
    end
end 


%-------------------------------------------------------------------
function P = sub_blockelimination(P,ydim,dim,Options)

% Block Elimination

% Description of the system
H1=P.H(:,ydim);
H2=P.H(:,dim);

m=nconstr(P);
A=-eye(m);
B=zeros(m,1);
Aeq=(H2)';
Beq=zeros(length(dim),1);

% Compute the rays
H=struct('A',[Aeq;A],'B',[Beq;B],'lin',(1:size(Beq,1))');
V=cddmex('extreme',H);
% eliminate all rows that are nonzero => zB is a zero matrix
zB = V.R*H2;
zB=round(zB*1e6)/1e6;
y=[];
for i=1:size(zB,1)
    if(zB(i,:)==zeros(1,size(zB,2)))
        y=[y;i];
    end
end
if Options.noReduce,
    P=polytope(V.R(y,:)*H1, V.R(y,:)*P.K, 2, 2);
else
    P=polytope(V.R(y,:)*H1, V.R(y,:)*P.K);
end

return;
