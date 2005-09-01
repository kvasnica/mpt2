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
% (C) 2004 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%          kvasnica@control.ee.ethz.ch
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

if ~isfulldim(PA),
    % exit quickly if input polytope is not fully dimensional
    P = mptOptions.emptypoly;
    return
end

d=dimension(PA);
orig_dim = dim;
dim = setdiff(1:d,dim);   % from this point, dim denotes dimension to eliminate

[m,n]=size(dim);
if ~(d>=n & m==1)
    error('PROJECTION: Dimensions must agree');
else
    for i=1:length(dim)
        if dim(i)>d
            error('PROJECTION: Index of dim bigger than dimension of P');
        end
    end
end

if ~isfield(Options,'psolvers')
    if length(orig_dim)<=2,
        % use iterative hull for lower dimensions
        Options.psolvers = [2 5 6 3 1 4 0];
    elseif length(dim)<=2,
        % only 1 or 2 dimensions to eliminate -> use fourier-motzkin
        Options.psolvers = [5 1 4 6 2 3 0];
    elseif length(dim)<=d/2
        if dimension(PA)<=3,
            % use block-elimination for lower dimensions
            Options.psolvers = [3 4 5 6 2 1 0];
        else
            Options.psolvers = [4 5 6 3 1 2 0];
        end
    else
        if length(orig_dim)<=2,
            % use iterative hull for lower dimensions
            Options.psolvers = [2 5 3 6 1 4 0];
        else
            Options.psolvers = [5 4 6 2 3 1 0];
        end
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

if (isempty(PA.Array) & ~PA.minrep & Options.noReduce==0) %| (~P.minrep & Options.projection==1),
    PA = reduce(PA);
end

if length(PA.Array)>0,
    P = polytope;
    for ii=1:length(Options.psolvers),
        try
            for reg=1:length(PA.Array),
                if nconstr(PA.Array{reg})<=dimension(PA.Array{reg}) & Options.psolvers(ii)==2,
                    % skip iterativehull if only one constraint - otherwise
                    % an infinite loop can occur
                    continue
                end
                if Options.psolvers(ii)==0,
                    Q = sub_extreme_hull(PA.Array{reg},dim,Options);
                elseif Options.psolvers(ii)==1
                    Q = sub_fouriermotzkin(PA.Array{reg},dim,Options);
                elseif Options.psolvers(ii)==2
                    Q = sub_iterativehull(PA.Array{reg},dim,Options);
                elseif Options.psolvers(ii)==3
                    Q = sub_blockelimination(PA.Array{reg},dim,Options);
                elseif Options.psolvers(ii)==4,
                    Q = mpt_esp(PA.Array{reg},orig_dim);
                elseif Options.psolvers(ii)==6,
                    % use mex implementation of fourier-motzkin elimination (fast
                    % version)
                    if ~isminrep(PA),
                        PA = reduce(PA);
                    end
                    HK = double(PA);
                    projHK = fourier(HK, orig_dim, Options.four_tol);
                    Q = polytope(projHK(:, 1:end-1), projHK(:, end));
                else
                    % use mex implementation of fourier-motzkin elimination
                    I = PA.Array{reg};
                    if ~isminrep(I),
                        I = reduce(I);
                    end
                    for qq=length(dim):-1:2,
                        D = fourier(double(I),[orig_dim dim(1:qq-1)],Options.four_tol);
                        I = polytope(D(:,1:end-1), D(:,end));
                        I = polytope(H, K);
                    end
                    D = fourier(double(I),orig_dim,Options.four_tol);
                    Q = polytope(D(:,1:end-1), D(:,end));
                end
                P = [P Q];
            end
            return
        catch
            if length(Options.psolvers) > ii,
                disp(['projection: ' p_strings{Options.psolvers(ii)+1} ' failed, trying ' p_strings{Options.psolvers(ii+1)+1} '...']);
            else
                disp(['projection: ' p_strings{Options.psolvers(ii)+1} ' failed...']);
            end 
        end
    end
else
    for ii=1:length(Options.psolvers),
        try
            if nconstr(PA)<=dimension(PA) & Options.psolvers(ii)==2,
                % skip iterativehull if only one constraint - otherwise
                % an infinite loop can occur
                continue
            end
            if Options.psolvers(ii)==0,
                P = sub_extreme_hull(PA,dim,Options);
            elseif Options.psolvers(ii)==1
                P = sub_fouriermotzkin(PA,dim,Options);
            elseif Options.psolvers(ii)==2
                P = sub_iterativehull(PA,dim,Options);
            elseif Options.psolvers(ii)==3
                P = sub_blockelimination(PA,dim,Options);
            elseif Options.psolvers(ii)==4,
                P = mpt_esp(PA,orig_dim);
            elseif Options.psolvers(ii)==6,
                % use mex implementation of fourier-motzkin elimination (fast
                % version)
                if ~isminrep(PA),
                    PA = reduce(PA);
                end
                HK = double(PA);
                projHK = fourier(HK, orig_dim, Options.four_tol);
                P = polytope(projHK(:, 1:end-1), projHK(:, end));
            else
                % use mex implementation of fourier-motzkin elimination
                if ~isminrep(PA),
                    PA = reduce(PA);
                end
                P = PA;
                H = P.H;
                K = P.K;
                xrand = randn(size(H,2),1)*10;
                HK = [H K+H*xrand];
                for qq=length(dim):-1:2,
                    D = fourier(HK,[orig_dim dim(1:qq-1)],Options.four_tol);
                    P = polytope(D(:,1:end-1), D(:,end));
                    H = P.H;
                    K = P.K;
                    HK = [H K];
                end
                D = fourier(HK,orig_dim,Options.four_tol);
                H = D(:,1:end-1);
                K = D(:,end);
                K = K - H*xrand([orig_dim]);
                P = polytope(H, K);
            end
            return
        catch
            if length(Options.psolvers) > ii,
                disp(['projection: ' p_strings{Options.psolvers(ii)+1} ' failed, trying ' p_strings{Options.psolvers(ii+1)+1} '...']);
            else
                disp(['projection: ' p_strings{Options.psolvers(ii)+1} ' failed...']);
            end
            % method failed, try the next one
        end
    end
end

fprintf('\nLast error: %s\n',lasterr);
if length(Options.psolvers)==1,
    error('PROJECTION: couldn''t compute projection, selected method failed!');
else
    error('PROJECTION: couldn''t compute projection, all methods failed!');
end


%----------------- sub-functions ---------------------------------------

function P = sub_extreme_hull(P,dim,Options)

Vert=extreme(P,Options);
Vert(:,dim)=[];
P=hull(Vert,Options);
return;

function P = sub_fouriermotzkin(P,dim,Options)

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
    else
        P=reduce(P); 
    end
    %P=reduce(P); 
end

P.bbox = [];
P.H(:,dim)=[];
if Options.noReduce,
    P=polytope(P.H,P.K,2,2);
else
    P=polytope(P.H,P.K);
end
return;
    

function P = sub_iterativehull(P,dim,Options)

% Iterative Hull
% disp('Method2');

xdim = 1:dimension(P);              % Dimension of polytope
ydim = xdim;                        % Dimension of the projection
ydim(dim) = [];
lpsolver = Options.lpsolver;
abs_tol = Options.abs_tol;

if length(ydim)==1
    
    %LPi%f1=zeros(length(xdim),1);
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
        %LPi%f1=randn(length(ydim),1);
        f1=randn(1,length(ydim));
        %LPi%f=zeros(length(xdim),1);
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
                %LPi%f=zeros(length(xdim),1);
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


function P = sub_blockelimination(P,dim,Options)

% Block Elimination

xdim = 1:dimension(P);              % Dimension of polytope
ydim = xdim;                        % Dimension of the projection
ydim(dim) = [];

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
