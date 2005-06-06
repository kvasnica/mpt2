function [Pn,Fi,Gi,activeConstraints, Phard,details]=mpt_mplp_ver6(Matrices,Options)
%MPT_MPLP Explicitly solves the given linear program (LP)
%
% [Pn,Fi,Gi,Phard,details]=mpt_mplp(Matrices,Options)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Multiparametric linear programming
%
% Solves the problem
%   V(x) = min (H + Dx)' U
%           U
%   s.t.   G U <= W + E x
%          bndA*x <= bndb
%
% As a solution we get 'nR' regions
%   Pn(i)={x : H x <= K}
% 
% with the optimal control law
%   U = Fi{i} x + Gi{i}
%
% and the corresponding cost function expression
%   V(x) = Bi{i} x + Ci{i}
%  
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% Matrices - a struct with all the parameters which are needed.
%            See description above for explanation.
%   Matrices.H=H;
%   Matrices.G=G;   
%   Matrices.W=W;
%   Matrices.E=E;
%   Matrices.D=D;    
%   Matrices.bndA=bndA;     Limits on exploration space, i.e. bndA*x<=bndb
%   Matrices.bndb
%   Matrices.constraintInfo;    
%
% Options.verbose      - level of verbosity
% Options.lpsolver     - which LP solver to use (help mpt_solveLP)
% Options.max_iter     - maximum number of iterations of the algorithm
% Options.step_size    - length of step over a facet (default: 1e-?)
% Options.f_perturb    - Perturbation of the optimization direction
%
% Note: If Options is missing or some of the fields are not defined, the default
%       values from mptOptions will be used
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% Pn,Fi,Gi           - for region Pn(i).H*x <= Pn(i).K computed input is
%                      U=Fi{i}*x+Gi{i}   
% activeConstraints  - Cell Array which stores the active constraints 
%                      of the optimizer in each region.
% Phard              - Defines the feasible state space partition (i.e. union of
%                      all regions) as Phard.H*x<=Phard.K
% details            - a structure with the following fields:
%     nRegions  number of regions
%     Pn        polyhedral partition
%     Fi,Gi     optimizer matrices: u(x) = Fi * x + Gi
%     Bi,Ci     value function matrices: J(x) = Bi * x + Ci
%     nHard     number of hard constraints
%     Phard     polytope given by hard constraints
%     nb        number of constraints for each region
%     LISTa     list of active constraints
%     adjacencyInfo adjacency information of the polytopic partition
%     adjacencyInfo.adjacencyList Adjacency list for each region of the
%                                 parition indexed by the border numbers 
%     adjacencyInfo.tSetList      List of infeasible boundaries of each region.  

% see also MPT_CONSTRUCTMATRICES, MPT_MPQP, MPT_OPTCONTROL, MPT_OPTCONTROLPWA

% $Revision: 1.3 $ $Date: 2005/06/06 15:21:06 $
%    
% (C) 2004 Miroslav Baric, Automatic Control Laboratory, ETH Zurich,
%     baric@control.ee.ethz.ch    
% (C) 2003 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%     kvasnica@control.ee.ethz.ch
% (C) 2003 Mato Baotic, Automatic Control Laboratory, ETH Zurich,
%     baotic@control.ee.ethz.ch
% (C) 2002 Francesco Borrelli, Automatic Control Laboratory, ETH Zurich

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

if ~isfield(Options,'verbose'),
    Options.verbose=mptOptions.verbose;
end
if ~isfield(Options,'abs_tol'),
    Options.abs_tol=mptOptions.abs_tol;
end
if ~isfield(Options,'rel_tol'),
    Options.rel_tol=mptOptions.rel_tol;
end
if ~isfield(Options,'lpsolver'),
    Options.lpsolver = mptOptions.lpsolver;
end

if ~isfield(Options,'qpsolver'),
    Options.qpsolver=mptOptions.qpsolver;
end
if ~isfield(Options,'max_regions'),
    Options.max_regions = Inf;      % maximum number of regions
end
if ~isfield(Options,'step_size'),
    Options.step_size=mptOptions.step_size;
end
if ~isfield(Options,'skipDualDegenerate'),
    % skip the calculation of the optimiziers within dual
    % degenerate regions
    Options.skipDualDegenerate = 0;
end
if ~isfield(Options,'smoothOptimizer'),
    % handles dual degeneracy in a way that ensures smooth
    % optimizer
    Options.smoothOptimizer = 1;
end
if ~isfield(Options,'projection'),
    % selects projection method used
    Options.projection = [5 1 3 4 2 0];
end
if ~isfield(Options, 'statusbar'),
    Options.statusbar = 0;
end

if ~isfield(Options,'plotRegions'),
    Options.plotRegions = 0;
elseif ( size(Matrices.E,2) > 3 ),
    Options.plotRegions = 0;
end

if isfield(Matrices,'D'),
    %
    % parametrized cost function
    %
    isCostParametrized = 1;
else
    isCostParametrized = 0;
end

%----------- TOLERANCES
%-----------------------------------------------------
EMPTY_ROW_TOL    = Options.abs_tol;         % Tolerance for declaring
% that row is empty
CONSTR_TOL       = Options.rel_tol;         % Tolerance for declaring that
% constr. is redundant
ZERO_TOL         = Options.abs_tol;         % Tolerance for
% considering something
% equal to 0
RANK_TOL         = 10*sqrt(eps);         % Tolerance for the
% rank test
FACET_TOL        = 0.1 * Options.step_size; % if the point is on
% the facet or not
CHEBY_TOL        = Options.step_size;       % tolerance for
% chebyshev radius
%----------- TOLERANCES
%-----------------------------------------------------

MAXREGIONS    = Options.max_regions;
ALPHA         = min(Options.step_size,1e-5);
ALPHAmax      = max(Options.step_size,1e-6);  % maximum step
ALPHAit       = 50;                           % number of iterations
ALPHAinc      = ALPHAmax/(ALPHA*ALPHAit);
%
Matrices.H = Matrices.H(:)';

L1   = Matrices.H;
G    = Matrices.G;
S    = Matrices.E;
W    = Matrices.W;
if isfield(Matrices, 'bndA'),
    bndA = Matrices.bndA;
    bndb = Matrices.bndb;
else
    bndA = [];
    bndb = [];
end

%
nx = size(S,2); % dimension of parameter vector
nu = size(G,2); % dimension of optimizer
nC = size(G,1); % number of constraints
%
emptypoly=polytope;
%
nRegions         = 0;       % number of regions
nHard            = 0;       % number of hard constraints
hardA            = [];      % hard constraints
hardb            = [];      % hard constraints
no_of_constr     = [];      % number of constraints for each region
list_active      = {};      % list of active constraints
degenerate       = [];
%
Pn = [emptypoly];
Hn = {};      % normalized polyhedron description
Kn = {};      % normalized polyhedron description
Fi = {};      % control law
Gi = {};      % control law
Ai = {};      % value function
Bi = {};      % value function
Ci = {};      % value function
adjacent={};  % adjacency list

BC = zeros(1,nx+1);   % matrix for storing value function 

%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                                       %%
%%       FIND THE STARTING POINT         %%
%% (IF THE ORIGINAL PROBLEM IS FEASIBLE) %%
%%                                       %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% add bounding constraints
%
if ~isempty(bndA),
    crA = [zeros(size(bndA,1),nu) bndA; G -S];
    crb = [bndb; W];
else
    crA = [G -S];
    crb = W;
end
%
% reduce the number of constraints
%
if ( Options.verbose > 1 ),
    fprintf(1,'Removing redundant constraints ... ');
end
rpOps.abs_tol = ZERO_TOL     ;
ZXpoly = polytope(crA,crb,0,2);
%
% check if the problem is feasible
%
[xCheby, rCheby] = chebyball(ZXpoly);

if length(xCheby)==1 & rCheby==-Inf
 %   disp('mpt_mplp: mpLP problem is infeasible.');
    Phard = emptypoly;
    details.feasible = 0;
    activeConstraints = {};
    return;
end

[ZXpoly,keptRows] = reduce(ZXpoly,rpOps);
if ( Options.verbose > 1 ),
    fprintf('%d constraints removed.\n\n',size(crA,1)-length(keptRows));
end
G     = crA(keptRows,1:nu);
S     = -crA(keptRows,nu+1:nu+nx);
W     = crb(keptRows);

%
% check if we are going to use adjacency information
%
if ( isfield(Matrices,'constraintInfo') ),
    useAdjacency = 1;
    constrInfo = Matrices.constraintInfo;
    % get indices of the constraints and increase them for the
    % number of added bounding constraints (bndA x <= bndb)
    %
    nAdded = length(bndb);
    idxValueFConstr = constrInfo.idxValueFConstr + nAdded;
    idxTSetConstr   = constrInfo.idxTSetConstr + nAdded;
    idxSysConstr    = constrInfo.idxSysConstr;
    if ( nAdded > 0 ),
        % keep system constraints at the beginning
        endSysConstr    = length(idxSysConstr);
        expSysConstr    = [1:nAdded] + endSysConstr;
        idxSysConstr    = [idxSysConstr, expSysConstr];
        constrInfo.idxValueFConstr = idxValueFConstr;
        constrInfo.idxTSetConstr   = idxTSetConstr;
        constrInfo.idxSysConstr    = idxSysConstr;
    end
    constrInfo.keptVFConstr   = intersect(keptRows, idxValueFConstr);
    constrInfo.keptTSetConstr = intersect(keptRows, idxTSetConstr);
    constrInfo.keptSysConstr  = intersect(keptRows, idxSysConstr);
    %
    % we will use full set of constraints (important for the
    % correct usage of adjacency information in the case of dual
    % degeneracy)
    %
    fullMatrices = struct('f',  Matrices.H, ...
                          'G',  crA(:,1:nu), ...
                          'S', -crA(:,nu+1:nu+nx), ...
                          'W',  crb);
else
    useAdjacency = 0;
end
%
% set the center of coordinate system to the projection of
% the Chebyshev center to the parameter space
%
if ~isfield(Options,'center'),
    center = xCheby(nu+1:nu+nx);
end

% the cost is also parametrized
%
if ( isCostParametrized ),
    D = Matrices.D;
else
    D = [];
end

% if we're going to solve dual problem, we need these matrices
%
% we assume that (z,x) polytope is bounded, i.e. number of rows of G is
% at least (nx+nz+1). Furthermore, for a well defined problem, G is full
% column rank.
%
nullG  = [];
dualB  = [];
rangeG = [];

probMatrices = struct('f',     L1, ...
                      'G',     G, ...
                      'S',     S, ...
                      'W',     W, ...
                      'D',     D, ...
                      'nullG', nullG, ...
                      'dualB', dualB, ...
                      'rangeG',rangeG);

crOptions    = struct('zeroTol',            ZERO_TOL, ...
                      'rankTol',            RANK_TOL, ...
                      'qpSolver',           Options.qpsolver, ...
                      'lpSolver',           Options.lpsolver, ...
                      'checkIsLowerDim',    1, ...
                      'skipDualDegenerate', Options.skipDualDegenerate, ...
                      'smoothOptimizer',    Options.smoothOptimizer, ...
                      'projection',         Options.projection, ...
                      'verbose',            Options.verbose, ...
                      'emptypoly',          emptypoly);

asOptions    = struct('zeroTol',   ZERO_TOL, ...
                      'rankTol',   RANK_TOL, ...
                      'lpSolver',  Options.lpsolver);

%================================================%
% Get the initial critical region
%================================================%
%
xInit = xCheby(nu+1:nu+nx);
%
% search for an initial point which lies inside the full
% dimensional critical region
%
initPointFound = 0;

xPert=ones(nx,1)*min(0.1,rCheby/10);
for i = 1:ALPHAit,
    xFeasible = xInit + xPert;
    [aSetFound,activeSet] = findActiveSet(probMatrices,xFeasible,asOptions);
    if aSetFound,
        cr = getCriticalRegion(probMatrices, activeSet, crOptions);
        if cr.type ~= 0, % full dimensional CR
            initPointFound = 1;
            break;
        end

        % otherwise, perturb the initial point within the
        % borders of the Chebyshev ball
        %
        xPert = 2 * rand(nx,1)-1;
        xPert = rand(1) * rCheby * xPert / sqrt(nx);
    end
end

if ~initPointFound,
    details={};
    nRegions=1;
    nHard=nC;
    hardA=bndA;
    hardb=bndb(:);
    disp('MPLP: No feasible starting point!');

    % This is if you whant to store the initial region where either the
    % problem is infeasible or there are only flat CR
    %
    details.list_active{nRegions} = list_active;
    details.no_of_constr = no_of_constr;
    details.Pn = polytope;
    details.Bi{nRegions}=repmat(inf,nx,1);
    details.Ci{nRegions}=Inf;
    details.Fi = {};
    details.Gi = {};
    details.nRegions=0;
    details.nHard=nHard;
    details.Phard = polytope(hardA, hardb);
    details.adjacencyInfo=struct('adjacencyList', {[]}, ...
        'tSetList', {[]});
    details.feasible = 0;
    activeConstraints=details.list_active;
    Phard=details.Phard;
    Pn=details.Pn;
    Fi=details.Fi;
    Gi=details.Gi;
    return;
end

nRegions = 1; % new region

Pquadrant    = {};
Pquadrant    = cell(1,2^nx); % number of quadrant is 2^nx
q = sub_whichquadrant(cr.P, center);
for qqq=1:length(q),
    Pquadrant{q(qqq)}(end+1) = nRegions;
end
%
% store the initial region and continue exploration of the
% parameter space
%
list_active{end+1}     = activeSet.idxActive(activeSet.idxCompl);
Fi{end+1}              = cr.Fi;
Gi{end+1}              = cr.Gi;
BC(end,:)              = [cr.Bi cr.Ci];
Pn(nRegions)           = cr.P;
no_of_constr(1)        = nconstr(Pn(1));
adjacent{1}            = zeros(no_of_constr(1),1);  % adjacency list
tsetcon{1}             = [];

if ( isCostParametrized ),
    Ai{end+1} = cr.Ai;
end
%
%%%%%%%%%%%%%%%%%%%%%%%%
%%                    %%
%% DO THE EXPLORATION %%
%%                    %%
%%%%%%%%%%%%%%%%%%%%%%%%
%
if Options.statusbar,
    mpt_statusbar;
    statbar.handle = mpt_statusbar('Computing...');
    statbar.progress = 0;
    Options.verbose = -1;
end

region = 1;
while region <= nRegions & nRegions <= MAXREGIONS,

    if Options.statusbar & mod(region, 10)==0,
        statbar.progress = statbar.progress + region / 10000;
        if isempty(mpt_statusbar(statbar.handle, statbar.progress, 0, 0.9)),
            mpt_statusbar;
            error('Break...');
        end
    end

    if Options.verbose > 1,
        if mod(region,50) == 0,
            disp(sprintf('Region: %d/%d, %d borders, %d hard', ...
                region,nRegions,no_of_constr(region),nHard));
        end
    elseif Options.verbose == 1,
        if mod(region,50)==0,
            disp(sprintf('Region: %d/%d', region,nRegions));
        end
    end

    [borderDirections, Knr] = double( Pn(region) );
    %
    % borderDirections are normalized
    %
    unexploredBorders = find ( adjacent{region}(:,1) == 0 );
    [facetPoints,facetR] = getFacetPoints(Pn(region),unexploredBorders,Options);
    for border = unexploredBorders',
        if ( facetR(border) < 0 ),
            %
            % we didn't find a point on the facet
            %
            warnMsg = sprintf(['Region %d, facet %d: cannot find point ' ...
                               'on the facet.\n'],region,border);
            warning(warnMsg);
            continue;
        end
        xBorder = facetPoints(:,border);
        xBeyond = xBorder + borderDirections(border,:)' * ALPHA;
        %
        if ( Options.plotRegions & nx < 4 ),
            if nRegions > 1,
                reg_idx = [1:nRegions]; reg_idx(region) = [];
                plot(Pn(reg_idx));
            end
            hold on;
            plot(Pn(region),'k');
            if ( nx == 1 ),
                plot(xBeyond(1),'r*');
            elseif ( nx == 2),
                plot(xBeyond(1),xBeyond(2),'r*');
            else
                plot(xBeyond(1),xBeyond(2),xBeyond(3),'r*');
            end
            hold off;
        end
        %
        % check if the point is outside the bounds of the feasible
        % region or prescribed bounds
        %
        if ( nHard > 0 ),
            idxViolatedHard = find ((hardA * xBeyond - hardb) > 0 );
            if ( ~isempty(idxViolatedHard) ),
                %
                % we're outside the feasible region of parameters, check
                % the point on the facet
                %
                borderSlacks = hardA * xBorder - hardb;
                idxBorder = find ( borderSlacks > ZERO_TOL );
                if ( isempty(idxBorder) ),
                    % check if there's something beyond
                    %
                    bordVect = borderDirections(border,:)';
                    [isBord,xOut] = checkIfBorder(bordVect, ...
                                                  Knr(border), ...
                                                  ZXpoly, ...
                                                  asOptions);
                    if  ( isBord ),
                        adjacent{region}(border,1) = -Inf;
                        [minSlack,idxMin] = min( abs(borderSlacks) );
                        tsetcon{region}(end+1) = idxMin;
                        continue;
                    else
                        outDir = xOut - xBorder;
                        xBeyond = xBorder + ALPHA * outDir / norm(outDir);
                    end
                else
                    % we have violated more than one hard
                    % constraint: this can happen if the chosen
                    % step size is too big and we have the tiny
                    % region somewhere in the corner OR SOMETHING'S WRONG
                    %
                    % check if we're REALLY outside the feasible set
                    %
                    [auxH,auxK] = double(Pn(region));
                    outPn = polytope([auxH;-hardA],[auxK;-hardb]);
                    if ( isfulldim(outPn) ),
                        % yes, it seems we are
                        %
                        errorStr = sprintf ('Region %d is outside feasible area!',region);
                        error(['MPLP ERROR:' errorStr]);
                    end
                    % otherwise, check the violated constraints
                    %
                    falseBorders = [];
                    for idxSusp = idxBorder(:)'
                        [isBord,xOut] = checkIfBorder(hardA(idxSusp,:)', ...
                                                      hardb(idxSusp), ...
                                                      ZXpoly, ...
                                                      asOptions);
                        if ( ~isBord ),
                            % oops, false border, kick it out
                            %
                            falseBorders(end+1) = idxSusp;
                        end
                    end
                    if ( ~isempty(falseBorders) ),
                        hardA(falseBorders,:) = [];
                        hardb(falseBorders) = [];
                        nHard = length(hardb);
                    else
                        % well, it seems everything's ok, we can't do
                        % much about numerical errors
                        %
                        adjacent{region}(border,1) = -Inf;
                        [minSlack,idxMin] = min( abs(borderSlacks) );
                        tsetcon{region}(end+1) = idxMin;
                        continue;
                    end
                end
            end
        end
        if ( ~isempty(bndA) ),
            %
            % check if the violation of the user defined bounds
            % occurs - if so, add the bound to the set of
            % constraints which define the feasible region
            %
            idxViolatedBounds = find( (bndA * xBeyond - bndb) > 0 );
            if ( ~isempty(idxViolatedBounds) ),
                if ( length(idxViolatedBounds) > 1 ),
                    error(['MPLP ERROR: Point xBeyond violates more than' ...
                           ' one bound.']);
                end
                hardA(end+1,:) = bndA(idxViolatedBounds,:);
                hardb          = [hardb;bndb(idxViolatedBounds)];
                nHard = nHard + 1;
                adjacent{region}(border,1) = -Inf;
                tsetcon{region}(end+1) = nHard;
                continue;
            end
        end
        %
        % Check if the point is inside of any existing polyhedra
        %
        solveNewLP = 1;
        isinOpt.abs_tol   = 0;
        isinOpt.fastbreak = 0;
        vi = (xBeyond - center) > 0;
        q = 1 + 2.^[0:nx-1] * vi;
        % polytopes in the same quadrant excluding the present one
        %
        searchIndex = Pquadrant{q}(find(Pquadrant{q}~=region));  
        valueSlacks = BC(searchIndex,:) * [xBeyond(:);1];
        [maxValue,firstNeighbor] = max(valueSlacks);
        neighborsIdx = find(abs(valueSlacks-maxValue) < 10*ZERO_TOL);  
        searchIndex = searchIndex(neighborsIdx);
        [isin,neighbor] = isinside(Pn(searchIndex), xBeyond, isinOpt);
        nMatch = length(neighbor);
        if nMatch > 0,
            solveNewLP = 0;
            neighbor = searchIndex(neighbor);    % actual region index Pquadrant{q}(neighbor)
            if nMatch > 1,
                if Options.verbose > 0,
                    disp('Polyhedra should not overlap');
                end
            end
            %
            % all polyhedra sharing the facet with the current one
            % will be stored in the adjacency list
            %
            % Flag 'solveNewLP' is set if we want to solve an LP
            % for the point xBeyond. New LP is not solved only if
            % there's a neighboring region which shares a facet
            % with the current one. In the case of overlapping
            % regions it can happen that the point 'xBeyond' is
            % inside some other region which overlaps with the
            % current one. In that case, we'll solve LP and obtain
            % a new region for 'xBeyond'.
            %
            for ii = 1:nMatch,
                [Hneigh,Kneigh] = double(Pn(neighbor(ii)));
                idxOposite = find(Hneigh*borderDirections(border,:)' < 0);
                facetSlacks = abs(Hneigh(idxOposite,:) * xBorder - ...
                    Kneigh(idxOposite));
                commonFacet = find( facetSlacks <= FACET_TOL );
                if ( length(commonFacet) == 1 ),
                    commonFacet = idxOposite(commonFacet);
                    freeColReg = find ( adjacent{region}(border,:)==0 );
                    freeColNeigh = find ( adjacent{neighbor(ii)}(commonFacet,:)==0 );
                    if ( isempty(freeColReg) ),
                        freeColReg = size(adjacent{region},2) + 1;
                    end
                    if ( isempty(freeColNeigh) ),
                        freeColNeigh = size(adjacent{neighbor(ii)},2) + 1;
                    end
                    adjacent{region}(border,freeColReg) = neighbor(ii);
                    adjacent{neighbor(ii)}(commonFacet,freeColNeigh) = region;
                    solveNewLP = 0; % regions share the facet -
                    % don't solve LP for 'xBeyond'
                elseif ( length(commonFacet) > 1 ),
                    if ( Options.verbose > 1 ),
                        disp ('Multiple adjacency detected.');
                    end
                else
                    if ( Options.verbose > 1 ),
                        disp (['Overlaping regions or multiple' ...
                            ' adjacency detected']);
                    end
                end
            end
        end
        if ( solveNewLP ),
            %==============================================%
            % Solve LP for the point 'xBeyond'             %
            %==============================================%
            bordvect = borderDirections(border,:)';
            crOptions.checkIsLowerDim = 1;
            crOptions.zeroTol = ZERO_TOL;
            %
            for niter = 1:ALPHAit + 1,
                [aSetFound,activeSet] = findActiveSet(probMatrices, xBeyond, asOptions);
                if aSetFound,
                    %
                    % check if the region with same active
                    % constraints already exists
                    %
                    idxActive = activeSet.idxActive;
                    nActive   = length(idxActive);
                    sameASFound = 0;
                    if ( length(list_active{region}) == nActive ),
                        if ( all(list_active{region} == idxActive) & ~any( degenerate == region )),
                            sameASFound = 1;
                        end
                    end
                    if ( niter == (ALPHAit+1) ),
                        % It is highly improbable that the region is of lower
                        % dimension after series of perturbation. We'll just
                        % skip the check of dimensionality and see what we'll get.
                        %
                        crOptions.checkIsLowerDim = 0;
                    end
                    if ( ~sameASFound ),
                        if ( useAdjacency & niter<=ALPHAit ),
                            [reducedMatrices,newActiveSet] = reduceConstraints ...
                                ( probMatrices, fullMatrices, activeSet, ...
                                constrInfo, asOptions);
                            cr = getCriticalRegion (reducedMatrices, newActiveSet, crOptions);
                        else
                            cr = getCriticalRegion (probMatrices, activeSet, crOptions);
                        end

                        if ( cr.type == 1 | cr.type == -1 )
                            %
                            % full-dimensional critical region has been
                            % found
                            %
                            break;
                        end
                    end
                else  % active set not found
                    % check if we're relly at the border of feasible
                    % region
                    %
                    [isBorder,xEdge] = checkIfBorder (bordvect,Knr(border),ZXpoly, ...
                                                      asOptions);
                    if ( isBorder ),
%                        disp('AS not found.');
                        break;
                    end
%                   disp('tin region at the border');
                    dv = xEdge - xBorder;
                    xBeyond = xBorder + ALPHA * dv/norm(dv);
                    continue;
                end
                %
                % perturb the initial point a little bit
                %
                if ( sameASFound & Options.verbose > 1),
                    disp('Same AS found, perturbing ...');
                    warning(['The same active set has been found for the ' ...
                             'neighboring regions. It seems that your ' ...
                             'problem is badly scaled. Try to reformulate ' ...
                             'it or use another LP solver.']);
                end
                rvect = randn(size(bordvect));
                dotpr = rvect'*bordvect;
                while ( (norm(rvect) < 10*Options.abs_tol) | ...
                        (abs(dotpr)  < 10*Options.abs_tol) ),
                    rvect = randn(size(bordvect));
                    dotpr = rvect'*bordvect;
                end
                rvect = rvect / norm(rvect);
                pertdir = rvect - dot(rvect,bordvect) * bordvect;
                RBorder = facetR(border);
                pertdir = rand(1) * RBorder * pertdir / norm(pertdir);
                pertvect = pertdir + bordvect;
                xBeyond = xBorder + niter * ALPHA * pertvect/norm(pertvect);
            end
            %
            if ~aSetFound,
                %
                % infeasible LP subproblem
                %
                nHard = nHard + 1;
                hardA(nHard,:) = borderDirections(border,:);
                hardb(nHard,:) = Knr(border,:);
                adjacent{region}(border,1) = -Inf;
                tsetcon{region}(end+1) = nHard;
                %
            else
                if ( sameASFound ),
                    %
                    % duplicate active set (?!?) - this shouldn't happen
                    %
                    if ( Options.verbose > 1 ),
                        disp ('Skipping duplicate region.');
                    end
                    continue;
                elseif ( cr.type == 0 )
                    %
                    % flat region
                    %
                    if ( Options.verbose > 1 ),
                        disp ('Skipping a flat region.');
                    end
                    continue;
                elseif ( cr.type == -1 )
                    %
                    % dual degenerate
                    %
                    if ( Options.verbose > 1 ),
                        disp (['Storing dual degenerate', ...
                            ' subregion.']);
                    end
                end
                %
                % store a new region
                %
                nRegions = nRegions + 1;
                list_active{end+1} = activeSet.idxActive;
                Pn(end+1) = cr.P;
                BC(end+1,:) = [cr.Bi cr.Ci];
                Fi{end+1} = cr.Fi;
                Gi{end+1} = cr.Gi;
                if ( isCostParametrized ),
                    Ai{end+1} = cr.Ai;
                end
                if ( cr.type == -1 ),
                    degenerate(end+1) = nRegions;
                end
                q = sub_whichquadrant(cr.P,center);
                for qqq = 1:length(q),
                    Pquadrant{q(qqq)}(end+1) = nRegions;
                end
                nFacets = nconstr(cr.P);
                no_of_constr(end+1) = nFacets;
                adjacent{nRegions}  = zeros(nFacets,1);   % creating adj matrix for the newly discovered region
                tsetcon{nRegions} = [];
                %
                % update adjacency info for the new region
                %
                [Hneigh,Kneigh] = double( Pn(nRegions) );
                idxOposite = find(Hneigh*borderDirections(border,:)' < 0);
                facetSlacks = abs(Hneigh(idxOposite,:) * xBorder - ...
                    Kneigh(idxOposite));
                commonFacet = find( facetSlacks <= FACET_TOL );
                if ( length(commonFacet) == 1 ),
                    commonFacet = idxOposite(commonFacet);
                    adjacent{region}(border,1) = nRegions;
                    adjacent{nRegions}(commonFacet,1) = region;
                elseif ( length(commonFacet) > 1 ),
                    if ( Options.verbose > 1 ),
                        disp ('Multiple adjacency detected.');
                    end
                else
                    if ( Options.verbose > 1 ),
                        disp (['Overlaping regions or multiple' ...
                            ' adjacency detected']);
                    end
                end
            end % if ~aSetFound
        end % solveNewLP
    end % Selective for loop for unexplored borders
    % check consistecy of the adjacency list
    %
    unexplored = find(adjacent{region}(1,:) == 0);
    if ( ~isempty(unexplored) ),
        if ( Options.verbose > 1 ),
            fprintf('Inconsistent adjacency information for region %d\n',region);
        end
    end
    region = region + 1;
end % END EXPLORATION

if Options.statusbar,
    mpt_statusbar(statbar.handle, 1);
end

if ( nRegions > MAXREGIONS ),
    error('MPLP ERROR: Maximum number of regions reached!!!');
end

activeConstraints = list_active;
details.feasible = 1;
details.no_of_constr=no_of_constr;
details.Pn = Pn;
details.Bi = mat2cell(BC(:,1:nx),ones(nRegions,1),nx);
details.Ci = mat2cell(BC(:,end),ones(nRegions,1),1);
details.Ai = Ai;
details.nRegions = nRegions;
details.degenerate = degenerate;
adjacencyInfo = struct('adjacencyList', adjacent, ...
                       'tSetList',      tsetcon);
details.adjacencyInfo = adjacencyInfo;

if nHard == 0,
    details.nHard = length(bndb);
    if details.nHard ~= 0,
        details.Phard = polytope(bndA,bndb,0,1);
    else
        details.Phard = emptypoly;
    end
else
    details.nHard = nHard;
    details.Phard = polytope(hardA, hardb);
end
Phard = details.Phard;
if Options.verbose>0,
    disp(sprintf('mpt_mplp: %d regions', nRegions));
end

if Options.statusbar,
    mpt_statusbar;
end

return;
%
%%%%%%%%%%% End of main routine %%%%%%%%%%%%%%%%%%%


%=============================================
%
function [newSearchIdx] = reduceSearchIdx(bordVect,region,searchIdx,BC)
%
%=============================================
%
% reduce the search for possible neighbors (use value function)
%
    nSearch   = length(searchIdx);
    
    searchB = BC(searchIdx,1:end-1);
    regionB = repmat(BC(region,1:end-1),nSearch,1);
    
    dirProd = (regionB - searchB) * bordVect(:);
    newSearchIdx = searchIdx(find(dirProd <= 0));

%=============================================
%
function [facetPoints,facetR] = getFacetPoints(P, idxUnexplored,Options)
%
%=============================================
%
% generates points on the facets by projections of an interior point to
% each of the facets. The interior point is obtained by improving
% "centrality" of the Chebyshev center
%
    
    [xCheby,rCheby] = chebyball(P);
    [H,K] = double(P);
    [nFacets,dim] = size(H);
    faceDist = K - H * xCheby;
    facetPoints = (repmat(xCheby',nFacets,1)+repmat(faceDist,1,dim).*H)';
    theOnes = ones(nFacets-1,1);
    facetR = -ones(nFacets,1);
    acOpts = struct('x0',[],...
                    'tol',1e-4,...
                    'maxiter',10);
    
    for ii = idxUnexplored(:)',
        Hi0 = null(H(ii,:));
        x0  = facetPoints(:,ii);
        idxOther = 1:nFacets;
        idxOther(ii) = [];
        Hother = H(idxOther,:);
        Kother = K(idxOther);
        facetH = Hother * Hi0;
        facetK = Kother - Hother * x0;
        
        [xopt,fval,lambda,exitflag,how]= ...
            mpt_solveLPi([zeros(1,dim-1) -1],[facetH theOnes],facetK,[],[],[], ...
                         Options.lpsolver,1);

        rFacet = xopt(end);
        cFacet = x0 + Hi0 * xopt(1:end-1);
        if ( strcmp(how,'ok') ),
            facetPoints(:,ii) = cFacet;
            facetR(ii) = rFacet;
        else % LP not feasible
             % check if the projected Chebyshev center is on the facet and
             % return that point and corresponding slacks
             %
            if ( all(facetK > Options.abs_tol) ),
                cFacet = x0;
                rFacet = min(facetK);
            end
            
        end
    end
%
%============= END getFacetPoints ===========================   



%=============================================================================
%
%function [out,ii,indexmaxdiff]=findas(f,A,b,F,indeq,tq, ...
%                                     indexmaxdiff,Options)
function [found,activeSet] = findActiveSet(matrices,x0,Options)
%
%=============================================================================
%
% Find the set of active constraints.
%
%-----------------------------------------------------------------------------

    activeSet = struct('isStrictlyCompl',  1, ...
                       'idxActive',        [], ...   % indices of active constraints
                       'lambda',           [], ...   % dual variables
                       'idxCompl',         [], ...   % strictly complementary active constraints
                       'xoptLP',           [], ...   % primal solution
                       'x0',              x0);       % actual parameter value
    found = 0;

    G            = matrices.G;
    S            = matrices.S;
    W            = matrices.W;
    [nConstr,nu] = size(G);
    
    Aineq = G;
    Bineq = W + S * x0;
    
    %
    % solve the primal and get indices of active constraints
    %
    grad = matrices.f;
    if ( ~isempty(matrices.D) ),
        % cost is parametrized
        %
        grad = grad + x0' * matrices.D';
    end
    [xoptLP,fval,lambda,exitflag,how] = ...
        mpt_solveLPi(grad, Aineq,Bineq, [],[],[],Options.lpSolver);
    if ~strcmp(how,'ok'),
        %
        % primal problem is infeasible or unbounded for some
        % reason
        %
        return;
    end
    slacks = abs(Bineq - Aineq * xoptLP);
    idxActive   = find( slacks < Options.zeroTol );
    idxCompl    = find(abs(lambda(idxActive)) > Options.zeroTol);
    activeSet.isStrictlyCompl  = ( length(activeSet.idxCompl)== ...
                                   length(activeSet.idxActive) );
%
    found               = 1;
    activeSet.idxActive = idxActive;
    activeSet.idxCompl  = idxCompl;
    activeSet.lambda    = lambda;
    activeSet.xoptLP    = xoptLP;
%
%============= END findActiveSet ===========================


%============= getCriticalRegion ====================
%
% get H-description of a critical region
function cr  = getCriticalRegion (matrices, activeSet, crOptions)
%
%====================================================
%
% Compute matrices the solution X and the polyhedra crA crb in this way:
% apply Gauss to the following system
% Gtilde   -St     |z  =  Wt
%                  |x
% to obtain
%
%  Iz       0    L     |z    =  |Wt
%  0        Ix   N     |x
%
if ( isempty(matrices.D) )
    cr = struct('P',crOptions.emptypoly,'Fi',[],'Gi',[],'Bi',[],'Ci',[], ...
        'type',0);
else
    cr = struct('P',crOptions.emptypoly,'Fi',[],'Gi',[],'Bi',[],'Ci',[], ...
        'Ai',[],'type',0);
end
nu = size(matrices.G,2);
nx = size(matrices.S,2);
idxActive   = activeSet.idxActive;
%
Ga  = matrices.G(idxActive,:);
Sa  = matrices.S(idxActive,:);
Wa  = matrices.W(idxActive,:);
%
if ( length(idxActive) < nu ),
    cr.type = -1;  % dual degeneracy
    %
else
    %
    [Q,R] = qr([Ga, -Sa]);
    U = R(1:nu,1:nu);
    P = R(1:nu,nu+1:nu+nx);
    if ( (length(idxActive) > nu) & crOptions.checkIsLowerDim ),
        %
        % PRIMAL DEGENERACY
        %
        D = R(nu+1:size(R,1),nu+1:nu+nx);
        %
        if ( rank(D, crOptions.rankTol) > 0 ),
            %
            % critical region is lower dimensional facet of another region
            %
            cr.type = 0;
            return;
        end
    end
    %
    if ( crOptions.smoothOptimizer & ~activeSet.isStrictlyCompl ),
        altBaseFound = 0;
        idxCompl = activeSet.idxActive(activeSet.idxCompl);
        Ga = matrices.G(idxCompl,:);
        xiSpace = null( Ga * null(matrices.f) );
        if ( ~isempty(xiSpace) ),
            cr.type = -1;
            if crOptions.verbose > 1,
                disp (['Alternative base found.']);
            end
        else
            if crOptions.verbose > 1,
                disp (['Alternative base NOT found.']);
            end
            if rank(U, crOptions.rankTol) < nu,
                cr.type = -1;
            else
                cr.type = 1;
            end
        end
    elseif ( rank(U, crOptions.rankTol) < nu ),
        %
        %  no unique optimizer => DUAL DEGENERACY
        %
        cr.type = -1;
    else
        %
        % we have full-dimensional primal degenerate critical
        % region
        cr.type = 1;
    end
end

if ( cr.type == -1 ),
    %
    %-----------------
    % DUAL DEGENERACY
    %-----------------
    % solution proposed in:
    %
    % J. Spjotvold, P.Tondel, T.A.Johansen
    % "A Method for Obtaining Continuous Solutions to
    % Multiparametric Linear Programs", submitted to CDC04
    %
    % formulate QP which minimizes optimizer norm along the
    % active constraints
    %-------------------------------------------------------
    cr  = getDualDegenerateCR (matrices, activeSet, crOptions);
    if cr.type == -1 | cr.type == 0,
        return;
    end
end

if ( cr.type == 1 ),
    %
    % We have a unique optimizer.
    %
    if ( length(idxActive) > nu ),
        idxActive = idxActive(activeSet.idxCompl);
        Ga  = matrices.G(idxActive,:);
        if rank(Ga, crOptions.rankTol) == nu,
            Sa  = matrices.S(idxActive,:);
            Wa  = matrices.W(idxActive,:);
            [Q,R] = qr([Ga, -Sa]);
            U = R(1:nu,1:nu);
            P = R(1:nu,nu+1:nu+nx);
        else
            idxActive = activeSet.idxActive;
        end
    end
    idxInactive = [1:size(matrices.G,1)];
    idxInactive(idxActive) = [];
    Gna = matrices.G(idxInactive,:);
    Wna = matrices.W(idxInactive);
    Sna = matrices.S(idxInactive,:);
    %
    BigB0 = Q \ Wa;
    Afbf  = U \ [P BigB0(1:nu)];
    Fi    = -Afbf(:,1:nx);
    Gi    =  Afbf(:,nx+1);
    H     = [Gna * Fi - Sna];
    K     = [Wna - Gna * Gi];
    cr.P  = polytope(H,K);

    if ( ~isempty(matrices.D) ),
        %
        % parametrized cost, take into account that dual
        % variables are parameter dependent
        %
        if ( length(idxActive) > nu ),
            %
            % primal degeneracy, requires a special treatment
            %
            Ga  = matrices.G(idxActive,:);
            [qq,rr] = qr(Ga);
            M1 = qq(:,1:nu);
            M2 = qq(:,nu+1:end);
            invGaM1 = inv(Ga' * M1);
            dualH = [M1 * invGaM1 * matrices.D, M2];
            dualK = -M1 * invGaM1 * matrices.f';
            dualPolyFull = polytope(dualH,dualK);
            dualPoly = projection(dualPolyFull,[1:nx]);
        else
            invGa = inv(Ga');
            dualH = invGa * matrices.D;
            dualK = -invGa * matrices.f';
            dualPoly = polytope(dualH, dualK);
        end
        if ( ~isfulldim(dualPoly) | ~isfulldim(cr.P) ),
            cr.type = 0;
            return;
        end
        cr.P  = intersect(cr.P,dualPoly);
    end

    if ~isfulldim(cr.P),
        cr.type = 0;
        return;
    end
end
%
cr.Fi   = Fi;
cr.Gi   = Gi;
cr.Bi   = matrices.f * Fi;
cr.Ci   = matrices.f * Gi;
if ( ~isempty(matrices.D) ),
    %
    % parametrized cost, value function is quadratic in parameters
    %
    cr.Bi = cr.Bi + Gi' * matrices.D;
    cr.Ai = matrices.D' * cr.Fi;
end

return;
%
%============= END getCriticalRegion ===========================


%============= getDualDegenerateCR====================
%
% get dual degenerate critical region and corresponding
% optimizer
%
function cr  = getDualDegenerateCR (matrices, activeSet, crOptions)
%
%====================================================
%
cr = struct('P',crOptions.emptypoly,'Fi',[],'Gi',[],'Bi',[],'Ci',[],'type',-1);

% take only the constraints satisfying the strict
% complementarity
%
nu = size(matrices.G,2);
nx = size(matrices.S,2);
idxActive = activeSet.idxActive(activeSet.idxCompl);

if ( crOptions.skipDualDegenerate ),
    %
    % we'll obtain dual degenerate region by projection and
    % skip the calculation of the optimizer
    %
    nconstr = size(matrices.G,1);
    cr.P  = projectDualDegenerateCR (matrices, idxActive, ...
        crOptions);
    cr.Bi = -activeSet.lambda(1:nconstr)' * matrices.S;
    cr.Ci = -matrices.W' * activeSet.lambda(1:nconstr);
    cr.Fi = zeros(nu,nx);
    cr.Gi = zeros(nu,1);
    return;
end

Ga = matrices.G(idxActive,:);
%
idxInactive = [1:size(matrices.G,1)];
idxInactive(idxActive) = [];
%
bQP = matrices.W + matrices.S * activeSet.x0;
Aqp_ineq = matrices.G(idxInactive,:);
Bqp_ineq = bQP(idxInactive,:);
Aqp_eq   = matrices.G(idxActive,:);
Bqp_eq   = bQP(idxActive,:);
%
%------------
% SOLVE QP
%------------
Hess = eye(nu); f = zeros(nu,1);
[zoptQP,lambda,how,eflag,objqp] = ...
    mpt_solveQP (Hess, f, Aqp_ineq, Bqp_ineq, ...
    Aqp_eq, Bqp_eq, activeSet.xoptLP, ...
    crOptions.qpSolver,[],1); %userescue
slacksQP = abs(bQP - matrices.G*zoptQP);
idxActiveQP = find ( slacksQP < crOptions.zeroTol );
%
Ga = matrices.G(idxActiveQP,:);
rankGa = rank(Ga, crOptions.rankTol);
%
if ( rankGa < length(idxActiveQP) ),
    %
    % primal degenerate QP: we'll use the projection
    % here
    %
    if ( crOptions.verbose > 1 ),
        disp('===> Primal degenerate QP');
    end
    %
    cr  = getPrimalDegenerateCR (matrices, idxActiveQP, ...
          idxActive, crOptions);
    %
    % this should happen only due to the bug in projection
    % algorithm
    %
    if ~isfulldim(cr.P),
        cr.type = 0;
        if ( crOptions.verbose > 1 )
            disp('lower dimensional crtical region');
        end
    end
else
    %
    % nondegenerate QP
    %
    idxInactiveQP = [1:size(matrices.G,1)];
    idxInactiveQP(idxActiveQP) = [];
    %
    % compute the boundaries of the sub- region and the
    % optimizer
    %
    Sa  = matrices.S(idxActiveQP,:);
    Wa  = matrices.W(idxActiveQP,:);
    Gna = matrices.G(idxInactiveQP,:);
    Wna = matrices.W(idxInactiveQP,:);
    Sna = matrices.S(idxInactiveQP,:);
    lambdaHK = -(Ga*Ga') \ [Sa Wa];
    cr.Fi = -Ga' * lambdaHK(:,1:nx);
    cr.Gi = -Ga' * lambdaHK(:,nx+1);
    cr.Bi = matrices.f * cr.Fi;
    cr.Ci = matrices.f * cr.Gi;
    [dummy,idxActivediffQP] = setdiff(idxActiveQP,idxActive);
    H  = [ Gna * cr.Fi - Sna; -lambdaHK(idxActivediffQP,1:nx)];
    K  = [ Wna - Gna * cr.Gi;  lambdaHK(idxActivediffQP,nx+1)];
    cr.P = polytope(H,K);
    if ~isfulldim(cr.P),
        cr.type = 0;
        if ( crOptions.verbose > 1 ),
            disp('lower dimensional dual degenerate CR');
        end
        return;
    end
end

if ( ~isempty(matrices.D) ),
    %
    % parametrized cost
    %
    Ga = matrices.G(idxActive,:);
    nActive = length(idxActive);
    [Q,R] = qr([Ga' -matrices.D]);
    R1 = R(1:nActive,1:nActive);
    R2 = R(1:nActive,nActive+1:nActive+nx);
    R3 = R(nActive+1:nu,nActive+1:nActive+nx);
    if ( rank(R3,crOptions.rankTol) >  0 ),
        cr.type = 0;
        cr.P = crOptions.emptypoly;
        return;
    else
        invR1 = inv(R1);
        QC = Q \ matrices.f';
        dualH = -invR1 * R2;
        dualK = -invR1 * QC(1:nActive);
        dualPoly = polytope(dualH,dualK);
    end
    cr.P = intersect(cr.P,dualPoly);
    if ( ~isfulldim(cr.P) ),
        cr.type = 0;
    end
end
%
%============= END getDualDegenerateCR ==============


%============= projectDualDegenerateCR ====================
%
% obtains dual degenerate critical region for the mpLP using
% the projection
%
function P  = projectDualDegenerateCR (matrices, idxActive, Options)
%
%====================================================
%
idxInactive = [1:size(matrices.G,1)];
idxInactive(idxActive) = [];
Ga  = matrices.G(idxActive,:);
Sa  = matrices.S(idxActive,:);
Wa  = matrices.W(idxActive);
Gna = matrices.G(idxInactive,:);
Sna = matrices.S(idxInactive,:);
Wna = matrices.W(idxInactive);
nx  = size(matrices.S,2);
%
[Qt,Rt] = qr(Ga');
P1      = Qt(:,1:size(Rt,2)) / (Rt(1:size(Rt,2),1:size(Rt,2))');
P2      = null(Ga);
HaKa    = Gna * P1 * ((Ga*P1) \ ([Sa Wa]));
H       = [HaKa(:,1:nx)-Sna, Gna*P2];
K       = Wna - HaKa(:,nx+1);
XZpoly  = polytope(H,K);  
%
if isfulldim(XZpoly),
    P = projection(XZpoly,[1:nx],Options);
else
    P = Options.emptypoly;
end

return
%
%============= END getDualDegenerateCR ==========


%============= getPrimalDegenerateCR ====================
%
% obtains primal degenerate critical region for the mpQP using
% the projection
%
function cr  = getPrimalDegenerateCR (matrices, idxActive, idxEquality, Options)
%
%====================================================
%
cr = struct('P',Options.emptypoly,'Fi',[],'Gi',[],'type',-1);
%
% determine the critical region: the solution is
% obtained using null-space method. Matrix P1 is chosen
% such that matrix (Ga'*P1) is well conditioned.
%
idxInactive = [1:size(matrices.G,1)];
idxInactive(idxActive) = [];
[dummy,idxDiffEq] = setdiff(idxActive, idxEquality);
nActive = length(idxActive);
nu = size(matrices.G,2);
nx = size(matrices.S,2);
%
Ga  = matrices.G(idxActive,:);
Gna = matrices.G(idxInactive,:);
Sna = matrices.S(idxInactive,:);
Wna = matrices.W(idxInactive,:);
%
% obtain a reduced set of equations satisfying LICQ
%
rankGa = rank(Ga, Options.rankTol);
[Qt,Rt,permT] = qr(Ga');
[Q,R,perm]    = qr(Ga);
reduxGa = permT' * Ga;
reduxGa = reduxGa(1:rankGa,:);
idxActiveRedux = permT' * idxActive;
idxActiveRedux = idxActiveRedux(1:rankGa);
%
% calculate the optimizer from the reduced set of active constraints
%
SWredux = [matrices.S(idxActiveRedux,:) matrices.W(idxActiveRedux)];
FiGi = reduxGa' * ((reduxGa * reduxGa') \ SWredux);
BiCi = matrices.f * FiGi;
cr.Fi = FiGi(:,1:nx);
cr.Gi = FiGi(:,nx+1);
cr.Bi = BiCi(1:nx);
cr.Ci = BiCi(nx+1);
%
% get the critical region
%
% $$$     dimP1 = min(size(R,1),nu);
% $$$     P1  = Q(:,1:dimP1) / (R(1:dimP1,:)');
P1  = Q(:,1:rankGa);
PP1 = Qt(:,1:rankGa);
P2  = null(Ga');
L   = P1 * ((PP1' * Ga' * P1) \ (PP1' * FiGi)); % this defines P1*lambda1
H1  = []; K1 = [];
if ~isempty(idxDiffEq),
    H1  = [L(idxDiffEq,1:nx), -P2(idxDiffEq,:)];
    K1  = -L(idxDiffEq,nx+1);
end
H2  = Gna * cr.Fi - Sna;
K2  = Wna - Gna * cr.Gi;
H   = [H1; H2,zeros(size(H2,1),size(P2,2))];
K   = [K1;K2];
LXpoly = polytope(H,K);
if ~isfulldim(LXpoly),
    % can this really happen?
    %
    cr.type = 0;
    return;
end
%
cr.P = projection(LXpoly,[1:nx],Options);
%
%============= END getPrimalDegenerateCR ==============


%============= reduceConstraints  ====================
%
% reduces the set of constraints (target set and inactive
% constraints) by exploiting the adjacency information
%
function [reducedMatrices, newActiveSet] = reduceConstraints ...
    (reduxMatrices, fullMatrices, activeSet, constrInfo, Options)
%
%====================================================
%
newActiveSet    = activeSet;
reducedMatrices = reduxMatrices;
%
fullG  = fullMatrices.G;
fullS  = fullMatrices.S;
fullW  = fullMatrices.W;
fullGS = [fullG, -fullS];
%
% determine full set of active value function constraints and
% the corresponding 'adjacent' constraints
%
zeroTol = Options.zeroTol;
zxOpt   = [activeSet.xoptLP;activeSet.x0];
adjacencyInfo = constrInfo.adjacencyInfo;
idxActive = activeSet.idxActive;
idxVFConstr = constrInfo.idxValueFConstr;
fullVFSlacks = abs( fullGS(idxVFConstr,:) * zxOpt - fullW(idxVFConstr));
idxVFActive  = find ( fullVFSlacks <= zeroTol );
%
idxTset = [];
idxAdjacent = [];
useFull = 0; % weather to use full set of constraints
for i = idxVFActive',
    adjacent       = adjacencyInfo(i).adjacencyList;
    firstColZeros   = find( adjacent(:,1) == 0 );
    if ( ~isempty(firstColZeros) ),
        % for some reason adjacency information for one of the
        % active regions is not complete (due to the region
        % overlapping, for example) - in this case we'll use
        % full set of constraints
        %
        useFull = 1;
        break;
    end
    idxAddAdjacent = setdiff(adjacent(:),idxAdjacent);
    tset           = adjacencyInfo(i).tSetList;
    idxAddTset     = setdiff(tset(:),idxTset);
    if ( ~isempty(idxAddAdjacent) ),
        idxAdjacent    = [idxAdjacent;idxAddAdjacent];
    end
    if ( ~isempty(idxAddTset) )
        idxTset        = [idxTset;idxAddTset];
    end
end
%
if ( useFull ),
%    disp ('Using full set of constraints.');
    return;
end
% remove 0s, -Infs and active value function indices
%
idxRegular  = find ( idxAdjacent > 0 );
idxAdjacent = idxAdjacent(idxRegular);
idxAdjacent = setdiff (idxAdjacent,idxVFActive);
idxAdjacent = constrInfo.idxValueFConstr(idxAdjacent);
%
% target set constraints
%
if ( ~isempty(constrInfo.idxTSetConstr) & ~isempty(idxTset) ),
    %
    % here we have to take all Tset constraints, even if they
    % are removed as redundant
    idxTset = constrInfo.idxTSetConstr(idxTset);
    %
    % remove active Tset constraints
    %
    GSTset = fullGS(idxTset,:); 
   idxActiveTset = find(abs(GSTset*zxOpt - fullW(idxTset)) <= zeroTol);
    idxTset(idxActiveTset) = [];
else
    idxTset = [];
end
%
% indices of inactive constraints
%
%
idxSysConstr = constrInfo.idxSysConstr;
GSSys = fullGS(idxSysConstr,:);
idxInactiveSysConstr = find(abs(GSSys*zxOpt - fullW(idxSysConstr)) > zeroTol);

idxInactiveReduced = [idxAdjacent(:); idxTset(:); ...
    idxInactiveSysConstr(:)];
%
% recompute the slacks of the inactive constraints
%
inactiveSlacks = abs (fullGS(idxInactiveReduced,:) * zxOpt - ...
    fullW(idxInactiveReduced,:));

%
%----------------------
% build the matrices
%----------------------
%
reducedMatrices.G = [reduxMatrices.G(idxActive,:); ...
    fullG(idxInactiveReduced,:)];
reducedMatrices.S = [reduxMatrices.S(idxActive,:); ...
    fullS(idxInactiveReduced,:)];
reducedMatrices.W = [reduxMatrices.W(idxActive,:); ...
    fullW(idxInactiveReduced,:)];
%
newActiveSet.idxActive = [1:length(idxActive)];
newActiveSet.lambda    = [activeSet.lambda(idxActive); ...
    zeros(length(idxInactiveReduced),1)];
%
%============= END reduceConstraints ==============


function [isBorder,xBorder] = checkIfBorder(borderVec,kBorder,ZXpoly,Options)
    
    isBorder = 0;
    [H,K] = double(ZXpoly);
    nz = size(H,2) - length(borderVec);
    f = [zeros(nz,1);-borderVec]';
    [xoptLP,fval,lambda,exitflag,how] = mpt_solveLPi(f, ...
                                                     H,...
                                                     K,... 
                                                     [],[],[], ...
                                                     Options.lpSolver);
    xBorder = xoptLP(nz+1:length(xoptLP));
    isBorder = ( abs(borderVec'*xBorder-kBorder)<Options.zeroTol );
%


%--------------------Raphael-------------------------
% change the function to:
%  1. compute all remaining extreme points of the bounding box
%  2. change the procedure to handle arbitrary dimensions
%========================================
function q = sub_whichquadrant(P,center)
%========================================
%
Options.noPolyOutput = 1;
[R,l,u] = bounding_box(P,Options);

nx = dimension(P);
vert = ones(nx,2^nx);
binOne=dec2bin(1);
for i=1:2^nx
    decVec=dec2bin(i-1,nx);
    for j=1:nx
        if(decVec(j)==binOne)
            vert(j,i)=l(j);
        else
            vert(j,i)=u(j);
        end
    end
end

%-------------------My-------------------
q1 = zeros(1,2^nx);
for i=1:2^nx
    q2 = sub_whichquadrant_point(vert(:,i),center);
    q1(q2) = 1;
end
q = find(q1==1);

return
%
%========= END sub_whichquadrant ===============


%===================================================
%
function q = sub_whichquadrant_point(xBeyond,center)
%
%===================================================
q  = [];
q1 = [];

nx = length(xBeyond);
%center = zeros(nx,1);       % center per default
twoQuadrant = find(xBeyond==0);

if length(twoQuadrant)==0
    %One = eye(nx);
    vi = (xBeyond - center) > 0;
    q1 = 1;
    for i=0:nx-1
        q1 = q1 + (vi(i+1))*2^i;
    end
else
    for j=1:length(twoQuadrant)
        xBeyond(twoQuadrant(j))=1;
        %One = eye(nx);
        vi = (xBeyond - center) > 0;
        q = 1;
        for i=0:nx-1
            q = q + (vi(i+1))*2^i;
        end

        q1 = [q1 q];
        xBeyond(twoQuadrant(j))=-1;
        %One = eye(nx);
        vi = (xBeyond - center) > 0;
        q = 1;
        for i=0:nx-1
            q = q + (vi(i+1))*2^i;
        end
        xBeyond(twoQuadrant(j))=0;
        q1 = [q1 q];
    end
end
q = q1;

return
%
% ============ END sub_whichquadrant_point =========
