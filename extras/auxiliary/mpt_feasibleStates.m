function X0 = mpt_feasibleStates(ctrl,gridpoints,Options)
%MPT_FEASIBLESTATES returns equidistantly spaced data points in feasible set
%
% X0 = mpt_feasibleStates(ctrl,gridpoints,Options)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Grids the state-space into given number of points and returns those which
% lie in the feasible set of a given explicit controller
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% ctrl            - Explicit controller (MPTCTRL object)
% gridpoints      - number of grid points (if not provided, 30 is default)
% Options.Pfinal  - polytope defining part of the state-space which should
%                   be considered for cost computation. (only reasonable
%                   if ctrl.Pfinal is an empty polytope)
%
% Note: If Options is missing or some of the fields are not defined, the default
%       values from mptOptions will be used
%
% ---------------------------------------------------------------------------
% OUTPUT
% ---------------------------------------------------------------------------
% X0     - set of feasible initial states (states which lie in
%          ctrl.Pn)
%
% see also MPT_PERFORMANCE
%

% Copyright is with the following author(s):
%
%(C) 2004-2005 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%              kvasnica@control.ee.ethz.ch

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

error(nargchk(1,3,nargin));

global mptOptions;

if ~isstruct(mptOptions),
    mpt_error;
end

if nargin<1,
    error('mpt_feasibleStates: Wrong number of input arguments!');
end

if nargin<3,
    Options = [];
end

if isa(ctrl, 'mptctrl')
    if ~isexplicit(ctrl)
        error('This function supports only explicit controllers!');
    end
    ctrlStruct = struct(ctrl);
else
    ctrlStruct = ctrl;
end

if ~isfield(Options,'verbose')
    Options.verbose=mptOptions.verbose;
end

if nargin>1 & ~isa(gridpoints,'double')
    error('Second input argument must be number of grid points!');
end

if ~mpt_isValidCS(ctrlStruct)
    error('mpt_feasibleStates: First argument has to be a valid controller structure! See mpt_control for details.');
end

if nargin<2
    gridpoints = 30;
    if Options.verbose>0,
        disp('mpt_feasibleStates: Number of grid points not given, assuming 30');
    end
end

if isfield(Options,'Pfinal') %& ~isfulldim(ctrlStruct.Pfinal),
    if ~isa(Options.Pfinal,'polytope'),
        error('mpt_performance: Options.Pfinal must be a polytope object!');
    end
    ctrlStruct.Pfinal = Options.Pfinal;
end

% first compute bounds on feasible state-space
if ~isfulldim(ctrlStruct.Pfinal),
    error('mpt_feasibleStates: Please limit the state-space of interest by Options.Pfinal !');
end
bbOptions = Options;
bbOptions.noPolyOutput = 1; % we don't need the bounding box as a polytope object

if ctrlStruct.probStruct.tracking,
    [nx,nu]=mpt_sysStructInfo(ctrlStruct.sysStruct);
    if isa(ctrl, 'mptctrl')
        nx = ctrl.sysStruct.dims.nx;
    else
        nx=(nx-nu)/2;
    end
    Pfinal = projection(ctrlStruct.Pfinal,1:nx);
else
    Pfinal = ctrlStruct.Pfinal;
end

if length(Pfinal)>1,
    % if Pfinal is a polyarray, first compute the hull
    hullPf = hull(Pfinal);
    % and then calculate the bounding box to obtain bounds on feasible state-space
    [B, lb, ub] = bounding_box(hullPf,bbOptions);
else
    [B, lb, ub] = bounding_box(ctrlStruct.Pfinal,bbOptions);
end

if ctrlStruct.probStruct.tracking,
    lb = lb(1:nx);
    ub = ub(1:nx);
end

% grid the state-space into equidistantly placed points
%dimB = dimension(B);
dimB = size(lb(:),1);
Xpoints = zeros(gridpoints,dimB);
for ii=1:dimB
    Xpoints(:,ii) = linspace(lb(ii),ub(ii),gridpoints)';
end
    

% generate all possible combinations of states
% one could use kron() here, but that one fails for high number of elements
n_states=dimB;
ZZ=[];
ZZ{n_states}=Xpoints(:,n_states);
for ii=n_states-1:-1:1,
    Zd=[];
    for jj=1:size(Xpoints,1),
        Zd=[Zd; repmat(Xpoints(jj,ii),length(ZZ{ii+1}),1)];
    end
    ZZ{ii}=Zd;
end
for ii=2:n_states,
    ZZ{ii}=repmat(ZZ{ii},length(ZZ{ii-1})/length(ZZ{ii}),1);
end
datapoints=[];
for ii=1:n_states,
    datapoints(:,ii)=ZZ{ii};
end

% datapoints now contains all possible states, but be careful, some of them do
% not belong to feasible area! 

npoints = size(datapoints,1);
locOpt = Options;
locOpt.verbose = -1;
locOpt.openloop = 0;

cost = zeros(npoints,1);

% break after first region which is being found
locOpt.fastbreak = 1;

if iscell(ctrlStruct.sysStruct.A),
    nx = size(ctrlStruct.sysStruct.A{1},2);
else
    nx = size(ctrlStruct.sysStruct.A,2);
end

X0 = [];
for ii=1:npoints,
    x0 = datapoints(ii,:)';     % one state vector
    %if isinside(ctrlStruct.Pn,x0),
    if isinside(Pfinal,x0),
        X0 = [X0; x0'];
    end
end
