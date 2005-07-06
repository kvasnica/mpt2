function [P,Vconv]=hull(V,Options)
%HULL Converts vertices into an H-representation polytope
%
% [P,Vconv]=hull(V,Options)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Creates convex hull of vertices and returns H-representation of the hull
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% V                      - matrix containing vertices of the polytope
% Options.extreme_solver - which method to use for convex hull computation
%                          (see help mpt_init for details)
% Options.abs_tol        - absolute tolerance
%
% Note: Initial values of the Options variable are given by mptOptions 
%       (see help mpt_init)
%
% ---------------------------------------------------------------------------
% OUTPUT
% ---------------------------------------------------------------------------
% P      - an H-representation polytope P={x | H x <= K}
% Vconv  - extreme points (i.e. points forming the convex hull)
%
% see also POLYTOPE/HULL, EXTREME, UNION, ENVELOPE

% $Id: hull.m,v 1.5 2005/07/06 08:30:05 kvasnica Exp $
%
% (C) 2003 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich
%     kvasnica@control.ee.ethz.ch

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

error(nargchk(1,2,nargin));

global mptOptions;

if ~isstruct(mptOptions),
    mpt_error;
end

if nargin<2,
    Options=[];
end

% if no options given, use default values
if ~isfield(Options,'extreme_solver')
    Options.extreme_solver=mptOptions.extreme_solver;
end
if ~isfield(Options,'abs_tol')
    Options.abs_tol=mptOptions.abs_tol;
end
if ~isfield(Options,'infbox')
    Options.infbox=mptOptions.infbox;
end
if ~isfield(Options,'verbose')
    Options.verbose=mptOptions.verbose;
end
if ~isfield(Options,'lpsolver')
    Options.lpsolver=mptOptions.lpsolver;
end
if ~isfield(Options,'debug_level')
    Options.debug_level=mptOptions.debug_level;
end
if ~isfield(Options,'noReduce')
    Options.noReduce = 0;
end
if ~isfield(Options,'novertred')
    % if set to 1, vertices will not be reduced to kick out non-extreme points
    % before calling cddmex('hull', V) - this may help in certain tricky cases
    Options.novertred = 0;
end

P = mptOptions.emptypoly;

if isempty(V)
    Vconv=[];
    return
end

Vorig = V;

% initiliaze variables ([issue91])
Vconv = []; H = []; K = [];

if Options.extreme_solver==3,
    % cddmex
    [P, Vconv, H, K, lowdim] = hull_cddmex(V, Options);
    if lowdim,
        return
    end
    
elseif Options.extreme_solver==1,
    % LRS (matlab implementation)
    [P, Vconv, H, K] = hull_lrs(V, Options);

elseif Options.extreme_solver==2,
    % analytic solution, alternative method
    [P, Vconv, H, K] = hull_matlab_alt(V, Options);
    
else
    % analytical solution
    [P, Vconv, H, K] = hull_matlab(V, Options);
end

if Options.debug_level>0
    if ~isempty(H),
        % do not check hull if H is empty, otherwise an error about
        % incompatible dimensions
        % [issue91]
        [result,i] = checkhull(P,H,K,Vconv,Options);
    else
        result = 1;
        i = 1;
    end
    if result~=0,
        % something went wrong, try another solver
        cursol = mptOptions.extreme_solver;
        
        % introduce new solver (4) - hull_cddmex called with Options.novertred=1
        % we only include it if extreme_solver=3 is available
        solvers = mptOptions.solvers.extreme;
        solvers(find(solvers==Options.extreme_solver)) = [];
        
        for isol = 1:length(solvers),
            nextsolver = solvers(isol);
            if nextsolver==cursol,
                % don't try current solver
                continue
            end
            cur_str = mpt_solverInfo('extreme', cursol);
            next_str = mpt_solverInfo('extreme', nextsolver);
            if Options.verbose>1,
                fprintf('HULL: "%s" failed, trying "%s"...\n', cur_str, next_str);
            end
            
            switch nextsolver
                case 4
                    % cddmex - hull based on full vertices
                    cddopt = Options;
                    cddopt.novertred = 1;
                    [P, Vconv, H, K] = hull_cddmex(Vorig, cddopt);
                case 3
                    % cddmex
                    cddopt = Options;
                    cddopt.novertred = 0;
                    [P, Vconv, H, K] = hull_cddmex(Vorig, cddopt);
                case 1
                    % LRS
                    [P, Vconv, H, K] = hull_lrs(Vorig, Options);
                case 0
                    % matlab
                    [P, Vconv, H, K] = hull_matlab(Vorig, Options);
                case 2
                    % matlab alternative
                    [P, Vconv, H, K] = hull_matlab_alt(Vorig, Options);
                otherwise
                    error(sprintf('HULL: unknown solver %d', nextsolver));
            end
            cursol = nextsolver;
            if ~isempty(H),
                [result,i] = checkhull(P,H,K,Vconv,Options);
            else
                result = 1;
                i = 1;
            end
            if result==0,
                break
            end
        end
    end
    
    if result~=0,
        % still something wrong, try another method...
        
        % use approach based on extreme points enumeration
        % as described in:
        %
        % A Problem in Enumerating Extreme Points
        % Katta G. Murty
        % Dep. of Industrial and Operations Engineering
        % University of Michigan
        % http://www-personal.engin.umich.edu/~murty/segments-10.pdf

        disp('HULL: problem detected, using alternative method...');
        V = unique(Vorig, 'rows');     % first kick out identical points
        Vconv=convhulln(V);            % identify points forming the convex hull
        Vconv=V(unique(Vconv),:);      % store extreme points
        [nvert,ndim] = size(Vconv);
        xmid = (sum(Vconv)/nvert)';    % compute interior point
        Yt = zeros(nvert,ndim);
        for iv = 1:nvert,              % make transformation y = x - xmid
            for id = 1:ndim,
                Yt(iv,id) = Vconv(iv,id) - xmid(id);
            end
        end
        PP = polytope(Yt,ones(nvert,1),1); % form polytope in q-space {q : q y <= 1}
        V = extreme(PP,Options);                 % enumerate it's extreme points
        nvert = size(V,1);
        K = zeros(nvert,1);
        H = V;
        for iv = 1:nvert,
            K(iv) = 1 + V(iv,:)*xmid;
        end
        P = polytope(H,K);               % convex hull is given by {x : q_i x <= 1 + q_i xmid}

        [result,i] = checkhull(P,H,K,Vconv,Options);
        if result~=0,
            error(['HULL: Point ' num2str(i) ' is not a vertex!'])
            return
        end
    end
end


%--------------------------------------------------------------------------
function [P,Vconv,H,K,lowdim]=hull_cddmex(V, Options)

Vorig = V;
vert.V=V;

% initiliaze variables ([issue91])
P=[]; Vconv = []; H = []; K = [];

if Options.novertred==0,
    try
        vert=cddmex('reduce_v',vert);  % kick out points which are not extreme
        %vert=cddmex('v_hull_extreme',vert);  % kick out points which are not extreme
    catch
        % return on error, other solver will be tried ([issue91])
        return
    end
    if isempty(vert.V),
        Vconv=[];
        P = polytope;
        return
    end
    if size(vert.R,1)>0,
        % rays detected, hull of points is not bounded
        % in that case we make an intersection with the infinity box to guarantee boundedness
        if Options.verbose>0,
            disp('HULL(V): polytope is unbounded! making an intersection with infbox...')
        end
        Vinf=extreme(unitbox(size(V,2),Options.infbox),Options);   % extreme points of the unit hypercube
        vert_inf.V=[V; Vinf];                                      % make the intersection
        
        try
            vert=cddmex('v_hull_extreme',vert_inf);                    % kick out points which are not extreme
        catch
            % return on error, other solver will be tried ([issue91])
            P=[]; Vconv = []; H = []; K = [];
            return
        end
        if size(vert.R,1)>0,
            error('HULL(V): polytope still unbounded after correction!');
        end
    end
    Vconv = vert.V;
end

wstatus = warning;
warning off
lowdim = 0;

if Options.novertred,
    vert2.V = Vorig;
else
    vert2.V = Vconv;
end
try
    HK=cddmex('hull',vert2);        % compute H-representation convex hull
    if ~isempty(HK.lin),
        lowdim = 1;
        HK = [];
    end
catch
    HK = [];
end
warning(wstatus);

if isempty(HK),
    % return on error, other solver will be tried ([issue91])
    P=polytope; Vconv = []; H = []; K = [];
    return
end

H=HK.A; %needed for comparison/debug later
K=HK.B; %needed for comparison/debug later
if Options.noReduce,
    P=polytope(H,K,2,2);
else
    P=polytope(H,K);
end
P = set(P,'vertices',Vconv);

return


%--------------------------------------------------------------------------
function [P,Vconv,H,K]=hull_matlab(V, Options)

V = unique(V, 'rows');             % first kick out identical points
if size(V,2)==3,                   % points in 3D space
    Vconv=convhulln(V);            % identify points forming the convex hull
    H=[];                          % initialize the matrices
    K=[];
    for ii=1:size(Vconv,1),
        r=Vconv(ii,:);             % pick up points which define extreme hyperplane
        p1=V(r(1),:)';
        p2=V(r(2),:)';
        p3=V(r(3),:)';
        h=-cross(p1-p2,p1-p3)';    % do the cross-product to get hyperplane description
        k=h*p1;
        H=[H; h];
        K=[K; k];
    end
    if Options.noReduce,
        P=polytope(H,K,0,2);
    else
        P=polytope(H,K);               % create the polytope Hx<=K
    end
    Vconv=V(unique(Vconv),:);      % store extreme points
    P=set(P,'vertices',Vconv);

elseif size(V,2)==2,               % points in 2D space
    Vconv=convhulln(V);            % identify points forming the convex hull
    puni=V(unique(Vconv),:);       % pick up extreme points

    % create a point in the middle by taking a convex combination of all extreme points
    pp=[1/length(puni)*sum(puni(:,1)); 1/length(puni)*sum(puni(:,2))];
    H=[];
    K=[];
    for ii=1:size(Vconv,1),
        r=Vconv(ii,:);
        p1=V(r(1),:);              % pick up 1st point
        p2=V(r(2),:);              % pick up 2nd point
        if (p1(1)==p2(1)),         % special case, first coordinates are identical
            h=[1 0];
            k=p1(1);
        elseif (p1(2)==p2(2)),     % special case, second coordinates are identical
            h=[0 1];
            k=p1(2);
        else
            m=(p2(2)-p1(2))/(p2(1)-p1(1));  % general case, compute the slope
            b=p1(2)-m*p1(1);                % and an offset
            h=[m -1];
            k=-b;
        end
        if h*pp-k>Options.abs_tol,   % identify proper sign of the hyperplane
            h=-h;
            k=-k;
        end
        H=[H; h];
        K=[K; k];
    end
    if Options.noReduce,
        P = polytope(H,K,0,2);
    else
        P=polytope(H,K);                % create the polytope Hx<=K
    end
    Vconv=puni;                     % store extreme points
    P=set(P,'vertices',Vconv);
    return
else
    % general n-D case:
    %
    % based on
    % Linear Programming Approaches to the Convex Hull Problem in R^n
    % P.M.Pardalos, Y.Li, W.W.Hager
    % Computers Math. Applic.
    % Vol. 29, No. 7, pp. 23-29, 1995
    %
    % min sigma
    % s.t.
    % x'(V_i - V_j) <= sigma       \forall i \in I - J, j \in J
    % x'(V_j_k - V_j_(k+1)) = 0    \forall k = 1,...,q-1,    q = |J|
    % sigma >= -1
    %
    % I is a set of all points
    % J is a set of points lying on an extreme hyperplane
    %
    % x is an extreme face iff sigma < 0

    Vconv = convhulln(V);         % pick up only extreme points
    nx = size(Vconv,2);           % number of extreme points
    nv = size(V,1);               % dimension
    I=1:nv;                       % I is a set of all extreme points
    H=[];
    K=[];
    for q=1:size(Vconv,1),
        J=Vconv(q,:);             % J is a set of points lying on the same extreme hyperplane (face)
        ImJ = setdiff(I,J);       % get indices of the set I-J
        A=[-1 zeros(1,nx)];       % inequality constraints for the LP Ax<=B, sigma >= -1
        B=1;                    % inequality constraints for the LP Ax<=B
        Aeq=[];
        Beq=[];
        f=[1 zeros(1,nx)];
        %             for j=1:length(J),
        %                 A=[A; -ones(length(ImJ),1) V(ImJ,:)-repmat(V(J(j),:),length(ImJ),1)];    % x'(V_i - V_j) <= sigma, \forall i \in I-J, j \in J
        %                 B=[B; zeros(length(ImJ),1)];                                             % x'(V_i - V_j) <= sigma, \forall i \in I-J, j \in J
        %             end
        for ii=1:length(ImJ),
            for j=1:length(J),
                A=[A; -1 (V(ImJ(ii),:) - V(J(j),:))];
                B=[B; 0];
            end
        end
        for k=1:length(J)-1,
            Aeq=[Aeq; 0 V(J(k),:)-V(J(k+1),:)];  % x'(V_j_k - V_j_(k+1)) = 0    \forall k = 1,...,q-1,    q = length(J)
            Beq=[Beq; 0];                        % x'(V_j_k - V_j_(k+1)) = 0    \forall k = 1,...,q-1,    q = length(J)
        end
        %[xopt,fval,lambda,exitflag,how]=mpt_solveLPi(f,A,B,Aeq,Beq,[],Options.lpsolver);   % solve the LP
        xopt=mpt_solveLPi(f,A,B,Aeq,Beq,[],Options.lpsolver);   % solve the LP
        if xopt(1)<0,
            % if sigma < 0, 'x' is an extreme face
            H=[H; xopt(2:end)'];
            K=[K; xopt(2:end)'*V(J(1),:)'];
        end
    end
    if(~isempty(H))
        if Options.noReduce,
            P = polytope(H,K,2,2);
        else
            P=polytope(H,K);
        end
        Vconv=V(unique(Vconv),:);
        P=set(P,'vertices',Vconv);
    else
        P=polytope;
        Vconv=[];
    end
end

return


%--------------------------------------------------------------------------
function [P,Vconv,H,K]=hull_matlab_alt(V, Options)

w = warning;
warning off
V = unique(V, 'rows');
k = convhulln(V);
Vconv = V(unique(k),:);  % store extreme points
c = mean(Vconv);
V = V - repmat(c,[size(V,1) 1]);
H = zeros(size(k, 1), size(V, 2));
ind_remove = [];
for ix = 1:size(k,1)
    F = V(k(ix,:),:);
    aa = (F\ones(size(F,1),1))';
    if any(isinf(aa)),
        ind_remove = [ind_remove ix];
        continue
    end
    H(ix,:) = aa;
end
ind_keep = setdiff(1:size(k, 1), ind_remove);
H = H(ind_keep, :);
K = ones(size(H, 1), 1);
K = K + H*c';
P = polytope(H, K);
warning(w);



%--------------------------------------------------------------------------
function [P,Vconv,H,K]=hull_lrs(V, Options)

V = unique(V, 'rows');     % kick out identical points
Vconv=convhulln(V);        % identify points forming the convex hull
Vconv=V(unique(Vconv),:);  % store extreme points
[P,dummy,H,K] = mpt_lrs('hull',Vconv,Options);


%--------------------------------------------------------------------------
function [result,i]=checkhull(P,H,K,Vconv,Options)
% result = 0 - all ok
% result = 1 - point 'i' is not a vertex
% result = 2 - point 'i' is not inside of convex hull

result = 0;
i = 0;
nx = size(Vconv,2);
for i=1:size(Vconv,1)
    tmp=find(abs(H*Vconv(i,:)'-K)<=Options.abs_tol); %find intersections with hyperplanes
    if length(tmp)<nx
        %check if each point is really a vertex
        result = 1;
        return
    elseif(~isinside(P,Vconv(i,:)'))
        %is point inside polytope, i.e. on facet            
        result = 2;
        return
    end
end
