function [Pn,dynamics,invCtrl]=mpt_infsetPWA(Pn,A,f,Wnoise,Options)
%MPT_INFSETPWA Computes (robust) positive invariant subset for PWA systems
%
% [Pn,dynamics] = mpt_infsetPWA(ctrl)
% [Pn,dynamics] = mpt_infsetPWA(ctrl,Options)
% [Pn,dynamics] = mpt_infsetPWA(Pn,A,f,Wnoise,Options)
% [Pn,dynamics,invCtrlStruct] = mpt_infsetPWA(ctrlStruct,Options)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Computes the maximal (robust) positive invariant set of a PWA System
% x(k+1)=A{i}x(k)+f{i}+w    for x(k) \in Pn(i), w \in Wnoise
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% Pn       - Polytope array defining the area where the PWA system is defined
% A        - Cell array containing dynamic matrices A_i
% f        - Cell array containing dynamic matrices f_i
% Wnoise   - Polytope which bounds additive uncertainty w \in Wnoise (can be empty)
% ctrl     - Explicit controller (MPTCTRL object)
% Options.verbose   - Level of verbosity 0,1 or 2
% Options.nohull    - If set to 1, do not compute convex unions
% Options.maxIter   - maximum number of iterations. Set is not invariant if
%                     iteration is aborted prior to convergence  (default is 200)
% Options.useTmap   - If set to true (default is false), transition map will be
%                     computed to rule out certain transitions
% Options.maxsplanes - maximum number of generated separating hyperplanes when
%                      computing transition map (default is 1000)
%
% NOTE: Length of Pn, A, f must be identical
%
% Note: If Options is missing or some of the fiels are not defined, the default
%       values from mptOptions will be used
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% Pn        - Polytope array defining the (robust) positive invariant set
% dynamics  - Integer array defining the active dynamics A_i,f_i for each
%             polytope Pn(i)
% invCtrl   - optional; returns a controller which contains control
%                 laws associated to the positive invariant set
%
% ---------------------------------------------------------------------------
% LITERATURE
% ---------------------------------------------------------------------------
% "Computation of Invariant Sets for Piecewise Affine Discrete Time Systems 
%  subject to Bounded Disturbances"
% S. Rakovic, P. Grieder, M. Kvasnica, D. Q. Mayne and M. Morari, 2003, submitted
% check http://control.ee.ethz.ch for latest info
%
% see also MPT_INFSET
%

% $Id: mpt_infsetPWA.m,v 1.11 2005/06/28 10:46:33 kvasnica Exp $
%
% (C) 2005 Pascal Grieder, Automatic Control Laboratory, ETH Zurich,
%          grieder@control.ee.ethz.ch
% (C) 2005 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%          kvasnica@control.ee.ethz.ch
% (C) 2003 Pascal Grieder, Automatic Control Laboratory, ETH Zurich,
%          grieder@control.ee.ethz.ch

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

error(nargchk(1,5,nargin));

if nargout>2 & nargin>=3,
    error('Please provide MPTCTRL object as an input!');
end

global mptOptions;
if ~isstruct(mptOptions),
    mpt_error;
end

if nargin<5,
    Options.verbose=1;
end
if(nargin<4 | isempty(Wnoise))
    Wnoise=polytope;
end

nargs = nargin;
if nargin==2
    if isstruct(A),
        Options = A;
    end
    nargs = 1;
end
if ~isfield(Options,'verbose'),
    Options.verbose=mptOptions.verbose;
end
if ~isfield(Options,'abs_tol'),
    Options.abs_tol = mptOptions.abs_tol;
end
if ~isfield(Options,'nounion')
    % if regions should be merged or not
    Options.nounion=0;
end
if ~isfield(Options, 'useTmap'),
    % if set to 1, transition map will be computed to rule out certain
    % transitions
    Options.useTmap = 0;
end
if ~isfield(Options, 'maxIter'),
    % Set is not invariant if iteration is aborted prior to convergence
    Options.maxIter = 200;
end

maxIter = Options.maxIter;

if nargs==1,
    ctrl = Pn;
    if isa(ctrl, 'mptctrl')
        if ~isexplicit(ctrl),
            error('This function supports only explicit controllers!');
        end
        ctrlStruct = struct(ctrl);
    else
        ctrlStruct = ctrl;
    end

    if ~mpt_isValidCS(ctrlStruct)
        error('mpt_infsetPWA: First argument has to be a valid controller structure! See mpt_control for details.');
    end
    sysStruct = ctrlStruct.sysStruct;
    if isfulldim(sysStruct.noise)
        Wnoise = sysStruct.noise;
    end
    Pn = ctrlStruct.Pn;
    Fi = ctrlStruct.Fi;
    Gi = ctrlStruct.Gi;
    if (iscell(sysStruct.A))
        pwasystem=1;
        if length(sysStruct.A)~=length(Pn)
            disp('In PWA case, each dynamics has to be associated with exactly one region!');
            disp('Linking dynamics to regions...');
            Acell={};
            Fcell={};
            for ii=1:length(Pn)
                [x,R] = chebyball(Pn(ii));            % compute center of the chebyshev's ball
                for jj=1:length(sysStruct.A)          % go through all dynamics description
                    if max(sysStruct.guardX{jj}*x+sysStruct.guardU{jj}*(Fi{ii}*x+Gi{ii})-sysStruct.guardC{jj})<Options.abs_tol,    % check which dynamics is active in the region
                        Acell{ii}=sysStruct.A{jj}+sysStruct.B{jj}*ctrlStruct.Fi{ii};
                        Fcell{ii}=sysStruct.B{jj}*ctrlStruct.Gi{ii}+sysStruct.f{jj};
                    end
                end
            end
        end
        A = Acell;
        f = Fcell;
    else
        error('system is not PWA!');
    end
else
    if(nargin<3 | isempty(f))
        nx=size(A{1},1);
        for i=1:length(A)
            f{i}=zeros(nx,1);
        end     
    end
end

nx=size(A{1},1);
origin = zeros(nx,1);
emptypoly=polytope;


%%%NOW COMPUTE INVARIANT SUBSET
iter=1;
dynamics=1:length(Pn);
notConverged=1;

if(isfulldim(Wnoise))
    targetPn=Pn-Wnoise;
else
    targetPn=Pn;
end

if ~isfield(Options, 'statusbar')
    Options.statusbar = 0;
end
if ~isfield(Options, 'status_min')
    Options.status_min = 0;
end
if ~isfield(Options, 'status_max')
    Options.status_max = 1;
end
if ~isfield(Options, 'closestatbar'),
    Options.closestatbar = 1;
end
statusbar = Options.statusbar;
closestatbar = Options.closestatbar;
Options.closestatbar = 0;
Options.statusbar = 0;
if statusbar,
    Options.verbose = -1;
end

progress = 0;
if statusbar,
    if ~isfield(Options, 'status_handle')
        Options.status_handle = mpt_statusbar('Computing invariant set...');
        closestatbar = 1;
    end
end

if statusbar,
    if isempty(mpt_statusbar(Options.status_handle, progress, Options.status_min, Options.status_max)),
        mpt_statusbar;
        error('Break...');
    end     
end

mergeOpt = Options;
mergeOpt.statusbar = 0;
mergeOpt.verbose = -1;
while(notConverged>0 & iter<maxIter)
    
    if statusbar,
        prog_min = mod((iter-1), 10)/10;
        prog_max = mod(iter, 10)/10;
        if isempty(mpt_statusbar(Options.status_handle, prog_min, prog_min, prog_max)),
            mpt_statusbar;
            error('Break...');
        end     
    end
    
    
    if Options.verbose>-1,
        disp(['Iteration Number ' num2str(iter)])
    end
    
    transP=polytope;    %initialize to empty polytope
    tdyn=[];
    notConverged=0;

    if Options.useTmap,
        % compute transition map
        
        % prepare Acell and Fcell
        lenPn = length(Pn);
        Acell = cell(1, lenPn);
        Fcell = cell(1, lenPn);
        for ii = 1:lenPn,
            Acell{ii} = A{dynamics(ii)};
            Fcell{ii} = f{dynamics(ii)};
        end
        % compute the transition map
        if Options.verbose > -1,
            fprintf('Computing transition map...\n');
        end
        [tmap, Pn] = mpt_transmap(Pn, Acell, Fcell, Options);
        
        if Options.verbose > -1,
            fprintf('Transition map discarded %.2f%% of possible transitions.\n', 100*(1 - nnz(tmap)/numel(tmap)));
        end
    end
    
    for i=1:length(Pn)
        
        if statusbar,
            if isempty(mpt_statusbar(Options.status_handle, (i-1)/length(Pn), prog_min, prog_max)),
                mpt_statusbar;
                error('Break...');
            end     
        end
        
        trans=0;        %initialize transition counter
        tP=emptypoly;   %initialize transition polyarray
        convCtr=0;      %this extra counter is needed for error checks
        for j=1:length(targetPn)
            if Options.useTmap,
                possible_transition = (tmap(i, j) == 1);
            else
                possible_transition = 1;
            end
            if (possible_transition), 
                % possible transition exists
                [Px,dummy,feasible]=domain(targetPn(j),A{dynamics(i)},f{dynamics(i)},Pn(i)); %compute set of states Pn(i)->targetPn(j)
                if(feasible) 
                    %transition exists
                    if Px~=Pn(i),
                        convCtr=convCtr+1;
                        notConverged=notConverged+1;
                    end
                    trans=trans+1;          %one more transition found
                    tP=[tP Px];             %store transition polytope
                end
            end
        end
        
        if statusbar,
            if isempty(mpt_statusbar(Options.status_handle, (i-1)/length(Pn), prog_min, prog_max)),
                mpt_statusbar;
                error('Break...');
            end     
        end
        
        %try to merge regions
        if(trans>0)
            if Options.verbose>1,
                fprintf('iteration=%d region=%d transitions=%d\n', iter, i, trans);
            end
            
            if Options.nounion==1,
                how=0;
            else
                % Options.hullunion is handled directly in polytope/union.m
                if nx<=3,
                    % compute the convex union if dimension is below or equal to 3
                    [Pu,how]=union(tP,Options);
                    if how,
                        % union is convex
                        PuPn_equal = (Pu == Pn(i));
                    end
                else
                    % it is also possible to simplify Pu using greedy merging
                    Pu = merge(tP, mergeOpt);
                    how = length(Pu) < trans;
                    if how,
                        % the merged object consists of less polytopes than the
                        % original, this is good for us
                        PuPn_equal = (Pu == Pn(i));
                    end
                end
            end
            if(how)
                %union is convex (or simplified using greedy merging) => overwrite old set
                if trans>1 & PuPn_equal,
                    notConverged=notConverged-convCtr;%union is identical to original set; reduce transition counter again
                end
                transP = [transP Pu];
                for kk=1:length(Pu),
                    tdyn(end+1)=dynamics(i);
                end
            else
                %union is not convex
                transP = [transP tP];
                for kk=1:trans
                    tdyn(end+1)=dynamics(i);             %dynamics of new polytope are same as original polytope at t-1
                end
            end
        end
    end
    
    Pn=transP;      %write results for next iteration
    dynamics=tdyn;
    if(isfulldim(Wnoise))
        targetPn=Pn-Wnoise;
    else
        targetPn=Pn;
    end

    iter=iter+1;
end


if(iter>=maxIter)
    if closestatbar,
        mpt_statusbar;
    end
    error('mpt_infsetPWA: Computation aborted because maximum number of iterations was reached');
end


if statusbar,
    if isempty(mpt_statusbar(Options.status_handle, 1)),
        mpt_statusbar;
        error('Break...');
    end     
end

%%TRY TO MERGE THE FINAL RESULT
for dyn=1:max(dynamics)
    ctr=0;
    Pt=emptypoly;
    
    dynSet=find(dynamics==dyn); 
    Pt=Pn(dynSet);   %set of polytopes with dynamic dyn

    %try to merge polytopes
    if nx<=3,
        % use union for dimensions smaller than 4
        [Pu,how]=union(Pt, Options); 
    else
        % use greedy merging for higher dimensions (computation of union becomes
        % prohibitive for dimensions above 3 and more than 2 input polytopes)
        Pu = merge(Pt, mergeOpt);
        how = length(Pu) <= length(Pt);
    end
    
    if(how==1)
        %union is convex
        Pn = [Pn Pu];               %add new set 
        for kk=1:length(Pu)
            dynamics(end+1)=dyn;    %add new set
        end
        
        Pn(dynSet)=[];          %remove old sets 
        dynamics(dynSet)=[];    %remove old sets
    end
end

if statusbar,
    if isempty(mpt_statusbar(Options.status_handle, (i-1)/length(Pn), prog_min, prog_max)),
        mpt_statusbar;
        error('Break...');
    end     
end

dynamics = dynamics(:)';

if nargout>2,
    invCtrlStruct = ctrlStruct;
    invCtrlStruct.Pn = Pn;
    invCtrlStruct.Pfinal = Pn;
    invCtrlStruct.Fi = {ctrlStruct.Fi{dynamics}};
    invCtrlStruct.Gi = {ctrlStruct.Gi{dynamics}};
    invCtrlStruct.Ai = {ctrlStruct.Ai{dynamics}};
    invCtrlStruct.Bi = {ctrlStruct.Bi{dynamics}};
    invCtrlStruct.Ci = {ctrlStruct.Ci{dynamics}};
    invCtrlStruct.dynamics = ctrlStruct.dynamics(dynamics);
    invCtrlStruct.details.isinvariant = 1;
    invCtrl = mptctrl(invCtrlStruct);
end

if closestatbar,
    mpt_statusbar;
end

return
