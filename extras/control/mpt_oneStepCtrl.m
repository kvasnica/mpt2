function [ctrlStruct,feasibleN,loopCtr] = mpt_oneStepCtrl(sysStruct,probStruct,Options)
%MPT_ONESTEPCTRL Computes low complexity controller for LTI systems
%
% ctrlStruct = mpt_oneStepCtrl(sysStruct,probStruct)
% ctrlStruct = mpt_oneStepCtrl(sysStruct,probStruct,Options)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% This function computes a low complexity controller for the system defined in 
% "sysStruct".
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------  
% sysStruct        - System structure in the sysStruct format
% probStruct       - Problem structure in the probStruct format
%
%	See the MPT Manual for additional details on the structure format or   
%   consult one of the example systems (e.g. Double_Integator) which were  
%	provided with this package.                                            
%
% Options.maxCtr   - Maximum number of iterations (default is 1000)
% Options.scaling  - Sclaing the set at each iteration with a parameter 0 < lambda < 1
%                    guarantees finite time convergence to a robust invariant subset
%                    of the maximal control invariant set. (Default: Options.scaling = 1)
% Options.PWQlyap  - if set to 1, compute PWQ Lyapunov function (default)
% Options.verbose  - Optional: level of verbosity
% Options.set_limit - If the invariant set has a chebychev redius which is smaller than this value
%                    the iteration is aborted. (Default is 1e-3)
%
% Note: If Options is missing or some of the fields are not defined, the default
%       values from mptOptions will be used (see help mpt_init)
%
% ---------------------------------------------------------------------------
% OUTPUT
% ---------------------------------------------------------------------------
% ctrlStruct     - Controller structure with following fields:
%   Pn,Fi,Gi     - for region Pn(i).H*x <= Pn.K(i) computed input is U=Fi{i}*x+Gi{i}   
%   Ai,Bi,Ci     - cost associated to each region (x'Aix + Bi*x + Ci)
%   Pfinal       - The maximum control invariant set as a polytope object
%   dynamics     - Dynamics active in region Pn(i)
%   details      - Structure with more details about the solution:
%     lyapunovQ  - Output of the function mpt_getPWQLyapFct
%     lyapunovL  - Output of the function mpt_getPWQLyapFct
%     lyapunovC  - Output of the function mpt_getPWQLyapFct
%     feasible   - Output of the function mpt_getPWQLyapFct
%     loopCtr    - Number of iterations needed to converge
%
%   feasibleN       Set to 1, if LMI analysis was successfull
%   loopCtr         Total number of iterations in computation
%     
% ---------------------------------------------------------------------------
% LITERATURE
% ---------------------------------------------------------------------------
% "Complexity Reduction of Receding Horizon Control", P. Grieder and M. Morari;
% In the Proceedings of the IEEE Conference on Decision and Control 2003, Maui, Hawaii
%
%
% see also MPT_ITERATIVEPWA
%

% Copyright is with the following author(s):
%
% (C) 2003-2005 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%               kvasnica@control.ee.ethz.ch
% (C) 2004 Raphael Suard, Automatic Control Laboratory, ETH Zurich,
%          suardr@ee.ethz.ch
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

error(nargchk(2,3,nargin));

global mptOptions;

if ~isstruct(mptOptions),
    mpt_error;
end

if ~isfield(sysStruct,'verified'),
    verOpt.verbose=1;
    sysStruct=mpt_verifySysStruct(sysStruct,verOpt);
end

if ~isfield(probStruct,'verified'),
    verOpt.verbose=1;
    probStruct=mpt_verifyProbStruct(probStruct,verOpt);
end

if nargin<3,
    Options = [];
end

origSysStruct = sysStruct;
origProbStruct = probStruct;

if(~isfield(Options,'maxCtr'))
    Options.maxCtr=1000;
end
if ~isfield(Options,'verbose'),
    Options.verbose=mptOptions.verbose;
end
if ~isfield(Options,'feasset'),
    % if set to 1, function returns maximum controllable set instead of a
    % controller
    Options.feasset=0;
end
if ~isfield(Options,'abs_tol'),
    Options.abs_tol=mptOptions.abs_tol;
end
if ~isfield(Options,'set_limit'),
    Options.set_limit=1e-3;
end
if ~isfield(Options,'scaling'),
    Options.scaling=1;
elseif(Options.scaling>1)
    error('Scaling parameter ''Options.scaling'' must be smaller than 1')
elseif(Options.scaling<=0)
    error('Scaling parameter ''Options.scaling'' must be larger than 0')
end

if ~isfield(Options, 'statusbar'),
    Options.statusbar = 0;
end
if ~isfield(Options, 'closestatbar')
    Options.closestatbar = 1;
end
closestatbar = Options.closestatbar;
Options.closestatbar = 0;
statusbar = Options.statusbar;
if statusbar,
    Options.statusbar = 0;
end


if ~isfield(Options,'ispwa'),
    % even though this function does not provide a full support for PWA systems,
    % it can be used to compute iterative solution for one fiex dynamics. Since
    % sysStruct will be in PWA format, we need this flag to suppress error
    % messages and to enforce slightly different convergence checks
    Options.ispwa=0;                   
end
if ~isfield(Options,'PWQlyap'),
    % if set to 1, PWQ Lyapunov function will be computed to testify stability
    % (default)
    Options.PWQlyap = 1;
end

if Options.ispwa==0,
    % this routine does not work for "true" PWA systems and for 1-norm case (see the note above)
    if strcmp(sysStruct.type,'PWA') | sysStruct.type==1,
        error('Sorry, this function handles LTI systems only!');
    end
    %     if probStruct.norm~=2,
    %         error('Sorry, this function supports only 2-norm problems!');
    %     end
end

starttime = cputime;

if statusbar,
    statbar.handle = mpt_statusbar('Computing...');
    statbar.progress = 0;
    Options.verbose = -1;
end

%initialize
loopCtr=0;
PfinalOld = probStruct.Tset;
notconverged=1;

if ~isfield(Options, 'Pfinal'),
    while notconverged 
        
        progress = mod(loopCtr, 20) / 20;
        if statusbar,
            if isempty(mpt_statusbar(statbar.handle, progress, 0, 0.5)),
                mpt_statusbar;
                error('Break...');
            end
        end
        
        loopCtr=loopCtr+1;
        if Options.verbose>1,
            fprintf('Iteration %d      \r',loopCtr);
        else
            if mod(loopCtr,20)==0 | loopCtr==1,
                if Options.verbose > -1,
                    fprintf('Iteration %d       \r',loopCtr);
                end
            end
        end
        
        % construct the problem - one step solution to previously computed target
        % set
        Options.includeLQRset = 0;
        tmpProbStruct = probStruct;
        tmpProbStruct.Tset = PfinalOld;
        tmpProbStruct.Tconstraint = 2;
        tmpProbStruct.subopt_lev = 0;
        tmpProbStruct.N = 1;
        
        % get matrices of the problem
        [G1,W1,E1]=mpt_constructMatrices(sysStruct,tmpProbStruct,Options); 
        
        % compute feasible set via projection
        P=polytope([-E1 G1],W1);
        Pfinal=projection(P,(1:size(E1,2)),Options);
        
        % algorithm converges when two consequent feasible sets are equal
        if(isfulldim(PfinalOld) & Options.scaling<1)
            %set is being "shrunk" from the outside in; only applies for computation of Cinf
            notconverged=~ge(Pfinal,PfinalOld,Options);       % Pfinal => PfinalOld
        elseif(isfulldim(PfinalOld) & Options.scaling==1)
            %no scaling, hence convergence is reached when sets are eual
            %this option can apply to the computation of Kinf and Cinf
            notconverged=~eq(Pfinal,PfinalOld,Options);       % Pfinal == PfinalOld
        else
            notconverged=1;
        end
        %notconverged = ~eq(Pfinal,PfinalOld,Options);
        %plot(PfinalOld,Pfinal);
        
        if Options.scaling<1,
            PfinalOld = Pfinal*Options.scaling;
        else
            PfinalOld = Pfinal;
        end
        if loopCtr > Options.maxCtr,
            disp('Maximum number of iterations reached without convergence! Increase value of Options.maxCtr');
            break
        end
        
        [x,R]=chebyball(PfinalOld);
        if(~isfulldim(PfinalOld) | R<=Options.set_limit)
            disp(['The invariant set for this system (if it exists) has a chebychev radius smaller than Options.set_limit (=' num2str(Options.set_limit) ') !!'])
            disp('Aborting iteration...')
            if Options.feasset,
                ctrlStruct = polytope;
                feasibleN = [];
            else
                ctrlStruct = [];
                feasibleN = 0; 
            end
            return
        end
    end
else
    Pfinal = Options.Pfinal;
end

if Options.feasset,
    ctrlStruct = Pfinal;
    feasibleN = [];
    return
end

% generate regions by enforcing x(1) \in Pfinal
Options.includeLQRset = 0;
Options.setHorizon = 1;
probStruct.Tset = Pfinal;
probStruct.Tconstraint = 2;
probStruct.subopt_lev = 0;
ctrlStruct = mpt_optControl(sysStruct,probStruct,Options);

if statusbar,
    if isempty(mpt_statusbar(statbar.handle, 1, 0, 0.5)),
        mpt_statusbar;
        error('Break...');
    end
end

lyapOptions = Options;
if statusbar,
    lyapOptions.statusbar = 1;
    lyapOptions.status_min = 0.5;
    lyapOptions.status_max = 1;
    lyapOptions.status_handle = statbar.handle;
    lyapOptions.closestatbar = 0;
end

if isfulldim(sysStruct.noise)
    disp('System is subject to additive noise, calculating Quadratic Lyapunov function...');
    [lyapunovP, drho, feasibleN] = mpt_getQuadLyapFct(ctrlStruct, lyapOptions);
    if feasibleN==0
        disp('WARNING: System may be unstable!');
    else
        ctrlStruct.details.lyapunovP = lyapunovP;
    end
    ctrlStruct.details.feasible = feasibleN;
    
elseif Options.PWQlyap,
    % PWQ Lyapunov function is computed to verify stability
    if Options.verbose>-1,
        disp('Calculating PWQ Lyapunov function...');
    end
    [lyapunovQ,lyapunovL,lyapunovC,feasibleN,drho]=mpt_getPWQLyapFct(ctrlStruct, lyapOptions);
    
    if feasibleN==0
        disp('WARNING: System may be unstable!');
    end
    
    % store PWQ Lyapunov function in the details fields
    %     ctrlStruct.details.lyapunovQ = lyapunovQ;
    %     ctrlStruct.details.lyapunovL = lyapunovL;
    %     ctrlStruct.details.lyapunovC = lyapunovC;
    ctrlStruct.details.feasible = feasibleN;
    if feasibleN,
        ctrlStruct.details.lyapunov.Q = lyapunovQ;
        ctrlStruct.details.lyapunov.L = lyapunovL;
        ctrlStruct.details.lyapunov.C = lyapunovC;
    end
    ctrlStruct.details.lyapunov.feasible = feasibleN;
end

if statusbar,
    if isempty(mpt_statusbar(statbar.handle, 1)),
        mpt_statusbar;
        error('Break...');
    end
end

ctrlStruct.details.loopCtr = loopCtr;

endtime = cputime;
ctrlStruct.details.runTime = endtime-starttime;
ctrlStruct.sysStruct = origSysStruct;
ctrlStruct.probStruct = origProbStruct;

if closestatbar,
    mpt_statusbar;
end