function [ctrl, F, obj, variables] = mpt_yalmipcftoc(sysStruct, probStruct, Options)
% MPT_YALMIPCFTOC CFTOC of PWA systems
%
% ctrl = mpt_yalmipcftoc(sysStruct, probStruct)
% ctrl = mpt_yalmipcftoc(sysStruct, probStruct, Options)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Computes the explicit solution of a given CFTOC problem with either linear
% or quadratic cost function. Supports PWA and LTI systems with
% boolean/integer/alphabet inputs (sysStruct.Uset), time-varying penalties, soft
% constraints (probStruct.{S|Sx|Su|Sy}), move blocking (probStruct.Nc), terminal
% state constraints (probStruct.xN), multiple target sets (probStruct.Tset),
% computes stabilizing sets for 2-norm problems.
%
% There a two approaches:
%  1. one-shot formulation
%  2. dynamic programming (DP) formulation
%
% You can choose the DP formulation by setting Options.dp=1
%
% By default we use the DP formulation if the cost function is linear, otherwise
% we use the one-shot formulation.
%
% Following problem formulations are supported:
%  * terminal state constraints - when probStruct.xN is given, terminal state
%                                 constraint is added (x(N)==probStruct.xN)
%  * time-varying penalties     - when probStruct.Q, probStruct.R, probStruct.Qy
%                                 are given as cell arrays
%  * multi-model dynamics       - allows to specify one model per prediction
%                                 step. to use this feature, specify sysStruct
%                                 as a cell array of system structures.
%  * move blocking              - if probStruct.Nc is given, first Nc control
%                                 moves are considered free (i.e.
%                                 u_0...u_(Nc-1)), while all  subsequent control
%                                 moves (u_Nc...u_(N-1)) are equal to u_(Nc-1).
%                                 works also for PWA and MLD systems!
%  * soft constraints - if "probStruct.Sx" is specified, uses soft state
%                       constraints.
%                     - probStruct.sxmax - maximum allowed violation of each
%                       state constraint. e.g. probStruct.sxmax=[10;0] allows to
%                       violate 1st state constraint by 10, while keeping the
%                       second constraint hard (i.e. no violation are allowed)
%                     - probStruct.Sy, probStruct.symax - softening of output
%                       constraints
%                     - probStruct.Su, probStruct.sumax - softening of input
%                       constraints
%  * penalize switching of PWA dynamics - probStruct.Qdyn, probStruct.Qswitch
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% sysStruct         - System structure in the sysStruct format
% probStruct        - Problem structure in the probStruct format
% Options           - Additional options
%    .verbose       - level of verbosity
%    .mp.algorithm  - which enumeration algorithm to use
%                     (Options.mp_algorithm=3 uses different enumeration)
%    .mp.presolve   - perform pre-solving if this option is true (default)
%    .dp            - if set to 1, use the DP formulation
%
% ---------------------------------------------------------------------------
% OUTPUT
% ---------------------------------------------------------------------------
% ctrl              - MPTCTRL object 
%
% see also MPT_CONTROL, MPT_OPTCONTROLPWA
%

% Copyright is with the following author(s):
%
% (C) 2006 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%          kvasnica@control.ee.ethz.ch
% (C) 2006 Johan Loefberg, Automatic Control Laboratory, ETH Zurich,
%          loefberg@control.ee.ethz.ch

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

% This function is the most general control function in the whole history of
% MPT. All the credit goes to Johan and his YALMIP, which makes it very easy to
% solve the CFTOC problem for LTI and PWA systems with both continuous and
% discrete inputs. The problem can be solved either in the dynamic programming
% fashion (default for linear cost functions), or in the one-shot formulation
% (default for quadratic cost functions). The user always force one of the
% formulation by setting "Options.dp" to a proper value.
%
% NOTE! if we have probStruct.xref/probStruct.uref, mpt_control() already shifts
% the dynamics such that the reference becomes the new origin. Therefore we
% always set the state/input reference to zero in the sequel. However, if you
% intend to call this function not via mpt_control, look for appropriate lines
% in this file and follow the instructions.

global mptOptions;
if ~isstruct(mptOptions),
    mpt_error;
end

if nargin<3,
    Options = [];
end
Options = mpt_defaultOptions(Options, ...
    'verbose', mptOptions.verbose, ...
    'includeLQRset', 1, ...
    'pwa_index', [], ...
    'dont_solve', 0 );


%===============================================================================
% set options which we pass to solvemp()
yalmipOptions = mptOptions.sdpsettings;
yalmipOptions.verbose = 0;  % to keep intemediate mpLPs silent
f = fields(Options);
for ii = 1:length(f),
    yalmipOptions = setfield(yalmipOptions, f{ii}, getfield(Options, f{ii}));
end


%===============================================================================
% handle multi-model systems (one sysStruct per one prediction step)
% part 1.
if iscell(sysStruct),
    multi_model = 1;
    if (length(sysStruct) ~= probStruct.N),
        error('Number of models must be equal to the prediction horizon.');
    else
        SST = sysStruct;        
    end
    SST{end+1} = SST{end};
    origSysStruct = sysStruct{1};
else
    multi_model = 0;
    origSysStruct = sysStruct;
    SST = sub_sysStruct2cell(sysStruct, probStruct.N+1);
end
verOpt.verbose = -1;
verOpt.ybounds_optional = 1;
verOpt.useyalmip = 1; % to tell mpt_verifyProbStruct we can deal with move blocking
[origSysStruct, origProbStruct] = mpt_verifySysProb(origSysStruct, ...
    probStruct, verOpt);


%===============================================================================
% verify sysStruct and probStruct structures
verOpt.verbose = 1;
verOpt.ybounds_optional = 1;
verOpt.useyalmip = 1;  % to tell mpt_verifyProbStruct we can deal with move blocking
pst = probStruct;
for ii = 1:length(SST),
    if ~isfield(SST{ii}, 'verified') | ~isfield(probStruct, 'verified'),
        [SST{ii}, pst] = mpt_verifySysProb(SST{ii}, probStruct, verOpt);
    end
    if pst.tracking > 0 & ~isfield(pst, 'tracking_augmented')
        [SST{ii}, pst] = mpt_yalmipTracking(SST{ii}, pst, verOpt);
    end
    verOpt.verbose = -1;
end
probStruct = pst;
sysStruct = SST{1};


%===============================================================================
% check whether one can use the DP formulation
if ~isfield(Options, 'dp'),
    % use the DP formulation for linear cost, otherwise use the one-shot
    % formulation
    if probStruct.norm==2,
        Options.dp = 0;
    else
        Options.dp = 1;
    end
end
if Options.dp,
    if Options.dont_solve & probStruct.N>1,
        fprintf('WARNING: switching to one-shot formulation because Options.dont_solve is 1.\n');
        Options.dp = 0;
    elseif isfield(probStruct, 'Qswitch'),
        fprintf('WARNING: switching to one-shot formulation because probStruct.Qswitch is given.\n');
        Options.dp = 0;
    end
end


%===============================================================================
% check system dimensions
[nx,nu,ny,nPWA,nbool,ubool] = mpt_sysStructInfo(sysStruct);


%===============================================================================
% do we have an LTI, PWA or an MLD dynamics?
[dynamics_type, nPWA, MLD, SST, sysStruct] = sub_checkmodels(SST, sysStruct);

% do we have at least one MLD/PWA/LTI system?
haveMLD = ~isempty(findstr([dynamics_type{:}], 'mld'));
havePWA = ~isempty(findstr([dynamics_type{:}], 'pwa'));
haveLTI = ~isempty(findstr([dynamics_type{:}], 'lti'));

% set pwa_index
if ~isempty(Options.pwa_index),
    if haveMLD,
        error('Cannot use Options.pwa_index for MLD systems.');
        
    else
        pwa_index = cell(1, length(SST));
        [pwa_index{:}] = deal(Options.pwa_index);
    end
    
else
    pwa_index = cell(1, length(nPWA));
    for im = 1:length(nPWA),
        pwa_index{im} = 1:nPWA(im);
    end
end


%===============================================================================
% reject certain problem descriptions
if isinf(probStruct.N),
    error('mpt_yalmipcftoc: Prediction horizon must be finite!');
end
if mpt_isnoise(sysStruct.noise),
    error('mpt_yalmipcftoc: additive noise not supported.');
end
if nbool>0 & probStruct.tracking==1,
    error('mpt_yalmipcftoc: tracking cannot be used for systems with integer inputs.');
end
if isfield(probStruct, 'Aunc') | isfield(probStruct, 'Bunc'),
    error('mpt_yalmipcftoc: parametric uncertainties are not supported.');
end


%===============================================================================
% handle control horizons
if isfield(probStruct, 'Nc'),
    Nc = probStruct.Nc;
    if Nc<probStruct.N & Options.dp==1,
        fprintf('Switching to one-shot formulation because control horizon is given.\n');
        Options.dp = 0;
    end
else
    Nc = probStruct.N;
end


%===============================================================================
% compute stabilizing target set and associated LQR cost-to-go for 2-norm
% problems

% we can't compute the stabilizing set if we have time-varying penalties,
% becuase mpt_computePWATset does not support them.
timevar_penalties = iscell(probStruct.Q) | iscell(probStruct.R);

% now try to compute the stabilizing target set. we can't do that if we have
% tracking, discrete inputs, penalties on otputs and/or own target set is
% already specified
if probStruct.norm==2 & probStruct.Tconstraint==1 & ...
        probStruct.tracking==0 & nbool == 0 & ~isfield(probStruct, 'Qy') & ...
        ~isfulldim(probStruct.Tset) & ~haveMLD & multi_model==0,

    if timevar_penalties,
        fprintf('WARNING: No stabilizing target can be computed with time-varying penalties.\n');
        
    else
        [Tset, P_N, origin_in_dyn] = sub_stabilizingset(SST{1}, pst, nx, nPWA(1));
        if isempty(origin_in_dyn),
            fprintf(['WARNING: No dynamics contains the origin in it''s interior, '...
                    'cannot compute a stabilizing target set.\n']);
            
        elseif isfulldim(Tset),
            probStruct.Tset = Tset;
            probStruct.P_N = P_N;
            
        else
            fprintf('WARNING: No stabilizing target set found!\n');
            
        end
    end
end


%===============================================================================
% create a cell array of probStruct.[R,Q,Qy] if they are not already provided as
% a cell array. this will be used later for simpler implementation of
% time-varying penalties
%
% Note that we must do this only after the stabilizing target set is computed,
% because mpt_computePWATset does not support time-varying penalties
%
probStruct = sub_timevarpenalties(probStruct);


%===============================================================================
% find out whether sysStruct.C, sysStruct.D and sysStruct.g all contain the same
% elements for all dynamics. if so, we can simplify things a bit
if multi_model | haveMLD,
    % always consider outputs as separate variables for the multi-model
    % approach, it makes things less messy coding-wise. do the same if we have
    % at least one MLD model.
    YeqSame = 0;
else
    YeqSame = sub_allYeqsEqual(SST);
end


%===============================================================================
% reject certain objectives if we have MLD/LTI systems
if haveMLD | haveLTI,
    if isfield(probStruct, 'Qswitch'),
        error('probStruct.Qswitch can only be used for PWA systems.');
    elseif isfield(probStruct, 'Qdyn'),
        error('probStruct.Qdyn can only be used for PWA systems.');
    end
end


%===============================================================================
% check constraints
haveXbounds = zeros(1, length(SST));
haveYbounds = zeros(1, length(SST));
dUconstraints = zeros(1, length(SST));
for im = 1:length(SST),
    haveXbounds(im) = isfield(SST{im}, 'xmax') & isfield(SST{im}, 'xmin');
    haveYbounds(im) = isfield(SST{im}, 'ymax') & isfield(SST{im}, 'ymin');
    dUconstraints(im) = any(~isinf(SST{im}.dumin)) | any(~isinf(SST{im}.dumax));
end


%===============================================================================
% when deltaU formulation is introduced, input constraints are set to +/- Inf.
% make them tighther.
for im = 1:length(SST),
    SST{im}.umax(isinf(SST{im}.umax)) = 1e6;
    SST{im}.umin(isinf(SST{im}.umin)) = -1e6;
end


%===============================================================================
% prepare references
yref = sub_defaultfield(probStruct, 'yref', zeros(ny, 1));
xref = sub_defaultfield(probStruct, 'xref', zeros(nx, 1));
uref = sub_defaultfield(probStruct, 'uref', zeros(nu, 1));
if haveMLD,
    % references for "d" and "z" variables of an MLD model
    nd = MLD{m}.nd; nz = MLD{1}.nz;
    dref = sub_defaultfield(probStruct, 'dref', zeros(nd, 1));
    zref = sub_defaultfield(probStruct, 'zref', zeros(nz, 1));
end
if isfield(probStruct, 'xref_augmented') | probStruct.tracking>0,
    % this flag is set in mpt_prepareTracking and indicates that the system was
    % already augmented such that xref/uref becomes the new origin. in this case
    % we MUST set xref/uref to zero
    xref = 0*xref;
    uref = 0*uref;
end


%===============================================================================
% prepare necessary variables
N = probStruct.N + 1;

% States x(k), ..., x(k+N)
x = sdpvar(repmat(nx,1,N), repmat(1,1,N));

% Inputs u(k), ..., u(k+N) (last one not used)
u = sdpvar(repmat(nu,1,N), repmat(1,1,N));

% Outputs y(k), ..., y(k+N) (last one not used)
y = sdpvar(repmat(ny,1,N), repmat(1,1,N));

d = cell(1, N);
z = cell(1, N);
for im = 1:length(SST),
    if isequal(dynamics_type{im}, 'mld'),
        % prepare "d" and "z" variables for MLD models
        nd = MLD{m}.nd; nz = MLD{1}.nz;
        d{im} = binvar(nd, 1);
        z{im} = sdpvar(nz, 1);
        
    else
        % we just need binary variables for PWA selection
        d{im} = binvar(nPWA(im), 1);
        
    end
end


%===============================================================================
% introduce slack variables for soft constraints if necessary
dims = struct('nx', nx, 'nu', nu, 'ny', ny);
[soften, slacks, smax, sweights] = sub_prepareslacks(probStruct, N, haveXbounds, haveYbounds, dims);


%===============================================================================
% initialize the solution
F = set([]);
J{N} = 0;  % initial cost
obj = 0;
starttime = cputime;


%===============================================================================
% now formulate constraints and objective of the CFTOC problem. use either the
% DP approach or one-shot formulation (depending on "Options.dp")
for k = N-1:-1:1    

    
    %===============================================================================
    if Options.dp & Options.verbose > -1,
        fprintf('--- Step %d (out of %d) ---\n', N-k, probStruct.N);
    end
    if k > Nc,
        % block u_k, k=Nc+1..N to be equal to u_Nc
        ku = Nc;
    else
        % otherwise u_k is a free variable
        ku = k;
    end
    iN = k - 1; iNu = ku - 1;

    
    %===============================================================================
    % set bounds on states, inputs and outputs.
    if haveXbounds(k) & ~(soften.all | soften.x),
        % we can't set hard bounds if we have soft constraints
        bounds(x{k}, SST{k}.xmin, SST{k}.xmax);
        bounds(x{k+1}, SST{k+1}.xmin, SST{k+1}.xmax);
    end
    if ~(soften.all | soften.u),
        % we can't set hard bounds if we have soft constraints
        bounds(u{k}, SST{k}.umin, SST{k}.umax);
    end
    if ~(k==1 & probStruct.y0bounds==0)
        % do not impose constraints on y0 if user does not want to
        if ~YeqSame & ~(soften.all | soften.y),
            % we can't set hard bounds if we have soft constraints
            bounds(y{k}, SST{k}.ymin, SST{k}.ymax);
        end
    end
    
    
    %===============================================================================
    if Options.dp,
        % in the DP formulation we formulate constraints for each step
        % separatelly. In the one-shot formulation, we add constraints for all
        % steps.
        F = set([]);
        
        % in the DP formulation we start a new objective for each step, in the
        % one-shot formulation we sum up cost for each horizon.
        obj = 0;
    end

    
    %===============================================================================
    % impose bounds on x_0 if necessary
    if isfulldim(SST{k}.Pbnd) & k==1,
        tag = sprintf('x_%d in Pbnd', iN);        
        F = F + set(ismember(x{k}, SST{k}.Pbnd), tag);
    end
    
    
    %===============================================================================
    % input constraints
    if k <= Nc,
        umin = SST{k}.umin - slacks.all{k} - slacks.u{k};
        umax = SST{k}.umax + slacks.all{k} + slacks.u{k};
        tag = sprintf('umin < u_%d < umax', iNu);
        F = F + set(umin < u{ku} < umax, tag);
    end
   
    % input slew rate constraints
    if dUconstraints(k) & k <= Nc - 1,
        dumin = SST{k}.dumin - slacks.all{k} - slacks.u{k};
        dumax = SST{k}.dumax + slacks.all{k} + slacks.u{k};
        tag = sprintf('dumin < u_%d - u_%d < umax', iNu+1, iNu);
        F = F + set(dumin < u{ku+1} - u{ku} < dumax, tag);
    end
    
    % some inputs can be boolean or from finite alphabet
    for iu = 1:nu,
        [t, inputset] = sub_inputtype(SST{k}, iu);
        tag = sprintf('u_%d(%d) in %s', iNu, iu, mat2str(inputset));
        if t == 'B',
            % this input is boolean
            F = F + set(binary(u{ku}(iu)), tag);
        elseif t == 'A',
            % this input is from finite alphabet
            F = F + set(ismember(u{ku}(iu), inputset), tag);
        end
    end

    
    %===============================================================================
    % state constraints
    if haveXbounds(k),
        xmin = SST{k}.xmin - slacks.all{k} - slacks.x{k};
        xmax = SST{k}.xmax + slacks.all{k} + slacks.x{k};
        tag = sprintf('xmin < x_%d < xmax', iN);
        F = F + set(xmin < x{k} < xmax, tag);
    end

    
    %===============================================================================
    % output constraints
    if haveYbounds(k),
        % soften constraints if necessary
        ymin = SST{k}.ymin - slacks.all{k} - slacks.y{k};
        ymax = SST{k}.ymax + slacks.all{k} + slacks.y{k};
        
        if YeqSame,
            % C,D,g are identical, it's enough to consider one dynamics
            tag = sprintf('ymin < y_%d < ymax', iN);
            if k > 1 | probStruct.y0bounds==1,
                % do not impose constraints on y0 if user does not want to
                F = F + set(ymin < SST{k}.C{1}*x{k} + ...
                    SST{k}.D{1}*u{ku} + SST{k}.g{1} < ymax, tag);
            end
            
        else
            tag = sprintf('ymin < y_%d < ymax', iN);
            if k > 1 | probStruct.y0bounds==1,
                % do not impose constraints on y0 if user does not want to
                F = F + set(ymin < y{k} < ymax, tag);
            end        
            
        end
    end
    
    
    %===============================================================================
    % add constraints on slacks (slacks have to be positive)
    if soften.all,
        tag = sprintf('0 < sa_%d < smax', iN);
        F = F + set(0 <= slacks.all{k} <= smax.all, tag);
    end
    if soften.x,
        tag = sprintf('0 < sx_%d < sxmax', iN);
        F = F + set(0 <= slacks.x{k} <= smax.x, tag);
    end
    if soften.u,
        tag = sprintf('0 < su_%d < sumax', iN);
        F = F + set(0 <= slacks.u{k} <= smax.u, tag);
    end
    if soften.y,
        tag = sprintf('0 < sy_%d < symax', iN);
        F = F + set(0 <= slacks.y{k} <= smax.y, tag);
    end
    
        
    %===============================================================================
    % Dynamics
    switch lower(dynamics_type{k}),
        case 'lti',
            % LTI dynamics. note that we have converted LTI systems into PWA form
            % with one dynamics
            tag = sprintf('x_%d == A*x_%d + B*u_%d', iN+1, iN, iNu);
            F = F + set(x{k+1} == SST{k}.A{1}*x{k} + SST{k}.B{1}*u{ku}, tag);
            if ~YeqSame,
                tag = sprintf('y_%d == C*x_%d + D*u_%d', iN+1, iN, iNu);
                F = F + set(y{k} == SST{k}.C{1}*x{k} + SST{k}.D{1}*u{ku}, tag);
            end
            
        case 'pwa',
            % PWA dynamics
            for i = pwa_index{k}
                tag = sprintf('d_%d => x_%d == A{%d}*x_%d + B{%d}*u_%d + f{%d}', ...
                    iN, iN+1, i, iN, i, iNu, i);
                F = F + set(implies(d{k}(i), x{k+1} == SST{k}.A{i}*x{k} + ...
                    SST{k}.B{i}*u{ku} + SST{k}.f{i}), tag);
                
                if ~YeqSame,
                    % we need to have the output as a variable
                    tag = sprintf('d_%d => y_%d == C{%d}*x_%d + D{%d}*u_%d + g{%d}', ...
                        iN, iN, i, iN, i, iNu, i);
                    F = F + set(implies(d{k}(i), y{k} == SST{k}.C{i}*x{k} + ...
                        SST{k}.D{i}*u{ku} + SST{k}.g{i}), tag);
                end
                
                tag = sprintf('d_%d => guardX{%d}*x_%d + guardU{%d}*u_%d <= guardC{%d}', iN, i, iN, i, iNu, i);
                F = F + set(implies(d{k}(i),SST{k}.guardX{i}*x{k} + ...
                    SST{k}.guardU{i}*u{ku} <= SST{k}.guardC{i}), tag);
            end
            
            tag = sprintf('sum d_%d = 1', iN);
            F = F + set(sum(d{k}(pwa_index{k})) == 1, tag);
            
        case 'mld',
            % MLD dynamics
            %        x(k+1) = A x + B1 u + B2 d + B3 z
            %         y(k)  = C x + D1 u + D2 d + D3 z
            %  E2 d + E3 z <= E1 u + E4 x + E5
            tag = sprintf('x_%d == A*x_%d + B1*u_%d + B2*d_%d + B3*z_%d + B5', ...
                iN+1, iN, iNu, iN, iN);
            F = F + set(x{k+1} == MLD{k}.A*x{k} + MLD{k}.B1*u{ku} + ...
                MLD{k}.B2*d{k} + MLD{k}.B3*z{k} + MLD{k}.B5, tag);
            
            tag = sprintf('y_%d == C*x_%d + D1*u_%d + D2*d_%d + D3*z_%d + D5', ...
                iN, iN, iNu, iN, iN);
            F = F + set(  y{k} == MLD{k}.C*x{k} + MLD{k}.D1*u{ku} + ...
                MLD{k}.D2*d{k} + MLD{k}.D3*z{k} + MLD{k}.D5);
            
            tag = sprintf('E2*d_%d + E3*z_%d <= E1*u_%d + E4*z_%d + E_5', ...
                iN, iN, iNu, iN);
            F = F + set(MLD{k}.E2*d{k} + MLD{k}.E3*z{k} <= MLD{k}.E1*u{ku} + ...
                MLD{k}.E4*x{k} + MLD{k}.E5, tag);
            
            % some states, inputs and/or outputs can be boolean:
            % states x(MLD.nx-MLD.nxb+1:MLD.nx) are boolean,
            % inputs u(MLD.nu-MLD.nub+1:MLD.nu) are boolean,
            % outputs y(MLD.ny-MLD.nyb+1:MLD.ny) are boolean
            ib1 = MLD{k}.nx-MLD{k}.nxb+1; ib2 = MLD{k}.nx; ib = ib1:ib2;
            if ~isempty(ib),
                tag = sprintf('x_%d(%d:%d) binary', iN, ib1, ib2);
                F = F + set(binary(x{k}(ib)), tag);
            end
            ib1 = MLD{k}.nu-MLD{k}.nub+1; ib2 = MLD{k}.nu; ib = ib1:ib2;
            if ~isempty(ib),
                tag = sprintf('u_%d(%d:%d) binary', iN, ib1, ib2);
                F = F + set(binary(u{k}(ib)), tag);
            end
            ib1 = MLD{k}.ny-MLD{k}.nyb+1; ib2 = MLD{k}.ny; ib = ib1:ib2;
            if ~isempty(ib),
                tag = sprintf('y_%d(%d:%d) binary', iN, ib1, ib2);
                F = F + set(binary(y{k}(ib)), tag);
            end
            
            % add bounds on auxiliary "z" variables
            % note: we could also use bounds() here:
            %   bounds(z{k}, MLD{k}.zl, MLD{k}.zu);
            tag = sprintf('MLD.zl < z_%d < MLD.zu', iN);
            F = F + set(MLD{k}.zl <= z{k} <= MLD{k}.zu, tag);

        otherwise
            error('Unknown type of system dynamics.');
            
    end
    

    %===============================================================================
    % add target set constraint
    if k==N-1 & isfulldim(probStruct.Tset),
        % ismember automatically handles cases where Tset is a polytope array
        tag = sprintf('x_%d in Tset', iN+1);
        F = F + set(ismember(x{k+1}, probStruct.Tset), tag);
    end


    %===============================================================================
    % add terminal state constraint
    if k==N-1 & isfield(probStruct, 'xN'),
        tag = sprintf('x_%d == probStruct.xN', iN+1);
        F = F + set(x{k+1} == probStruct.xN, tag);
    end
    
    
    %===============================================================================
    % objective function
    if isfield(probStruct, 'Qy'),
        % add penalty on outputs
        
        if YeqSame,
            % it's enough to consider just one dynamics because C,D,g are
            % identical for all dynamics
            obj = obj + sub_norm(SST{k}.C{1}*x{k} + SST{k}.D{1}*u{ku} + ...
                SST{k}.g{1} - yref, probStruct.Qy{k}, probStruct.norm);
   
        else
            obj = obj + sub_norm((y{k} - yref), probStruct.Qy{k}, probStruct.norm);
            
        end

    else
        % add penalty on states
        obj = obj + sub_norm((x{k} - xref), probStruct.Q{k}, probStruct.norm);
        
        % add terminal weight if specified
        if k==N-1 & isfield(probStruct, 'P_N'),
            obj = obj + sub_norm((x{k+1} - xref), probStruct.P_N, probStruct.norm);
        end
        
    end
    
    
    %===============================================================================
    % add penalty on inputs
    if 1 | (k <= Nc),
        % we could penalize just the inputs from k=0..Nc-1, i.e.
        %
        % min (\sum_{k=0}^{Nc-1} ||Q*x_k||_p + ||R*u_k||) + (\sum_{k=Nc}^{N-1} ||Q*x_k||_p)
        %
        % but currently we penalize
        % min \sum_{k=0}^{N-1} ||Q*x_k||_p + ||R*u_k||_p
        obj = obj + sub_norm((u{ku} - uref), probStruct.R{k}, probStruct.norm);
    end

    
    %===============================================================================
    % add penalty on deltaU
    if isfield(probStruct, 'Rdu'),
        obj = obj + sub_norm(u{ku+1} - u{ku}, probStruct.Rdu{ku}, probStruct.norm);
    end


    %===============================================================================
    % penalize delta and z variables from the MLD model
    if isequal(dynamics_type{k}, 'mld'),
        if isfield(probStruct, 'Qz'),
            obj = obj + sub_norm((z{k} - zref), probStruct.Qz{k}, probStruct.norm);
        end
        if isfield(probStruct, 'Qd'),
            obj = obj + sub_norm((d{k} - dref), probStruct.Qd{k}, probStruct.norm);
        end
    end

    
    %===============================================================================
    % penalize switching between dynamics of a PWA system    
    if isequal(dynamics_type{k}, 'pwa') & isfield(probStruct, 'Qswitch'),
        obj = obj + sub_norm(d{k+1} - d{k}, probStruct.Qswitch{k}, probStruct.norm);
    end

    
    %===============================================================================
    % penalize "d" variables which denote to which dynamics does a state belong
    % to (only for PWA systems!)
    if isequal(dynamics_type{k}, 'pwa') & isfield(probStruct, 'Qdyn'),
        obj = obj + sub_norm(d{k}, probStruct.Qdyn{k}, probStruct.norm);
    end
    
    
    %===============================================================================
    % penalize slacks
    if soften.all,
        obj = obj + sub_norm(slacks.all{k}, sweights.all, probStruct.norm);
    end    
    if soften.x,
        obj = obj + sub_norm(slacks.x{k}, sweights.x, probStruct.norm);
    end    
    if soften.y,
        obj = obj + sub_norm(slacks.y{k}, sweights.y, probStruct.norm);
    end
    if soften.u,
        obj = obj + sub_norm(slacks.u{k}, sweights.u, probStruct.norm);
    end
        

    %===============================================================================
    if Options.dp,
        % Solve one-step problem, add cost-to-go J{k+1}
        
        % decrease verbosity value. by default we use Options.verbose=1, but
        % this makes mpt_mplp/mpt_mpqp display number of regions obtained in
        % each call, which can be rather disturbing.
        %yalmipOptions.verbose = Options.verbose - 1;
        [sol{k}, diagnost{k}, Uz{k}, J{k}] = solvemp(F, obj + J{k+1}, ...
            yalmipOptions, x{k}, u{k});
        if Options.verbose > -1 & ~isempty(sol{k}),
            nr = 0;
            for ii = 1:length(sol{k}),
                if ~isempty(sol{k}{ii}),
                    nr = nr+length(sol{k}{ii}.Pn);
                end
            end
            fprintf('Generated %d regions\n', nr);
        end
    end
    
end

variables.x = x;
variables.u = u;

if Options.dont_solve,
    % just return constraints, objectives and variables
    ctrl = [];
    return
end

if Options.dp & Options.verbose > -1 & probStruct.norm~=2,
    % make an empty line to separate output from mpt_removeOverlaps
    fprintf('\n');
end

%===============================================================================
% solve the one-shot problem as mpMIQP/mpMILP if necessary
if Options.dp==0,
    %[sol{k}, diagnost{k}, Uz{k}] = solvemp(F, obj, yalmipOptions, x{k}, u{k});
    
    % obtain the open-loop solution:
    [sol{k}, diagnost{k}, Uz{k}] = solvemp(F, obj, yalmipOptions, x{k}, [u{1:end-1}]);
end


%===============================================================================
% collect overlapping partitions together, remove overlaps if cost is linear
overlaps = 1;
if length(sol{k})==1,
    ctrl = sol{k}{1};
    overlaps = 0;
    
else
    if isempty(sol{k}),
        fprintf('\n\n');
        fprintf('========================================================================\n');
        fprintf('Please send your system and problem definition to mpt@control.ee.ethz.ch\n');
        fprintf('========================================================================\n\n');
        error('mpt_yalmipcftoc: an error has occurred, see message above.');
    end
        
    if probStruct.norm==2,
        ctrl = mpt_mergeCS(sol{k});
    else
        ctrl = mpt_removeOverlaps(sol{k});
        overlaps = 0;
    end
    
end


%===============================================================================
if isempty(ctrl),
    % problem either infeasible or some problem occured
    ctrl = mptctrl;
    return
end

if probStruct.norm~=2,
    for ii = 1:length(ctrl.Ai),
        ctrl.Ai{ii} = zeros(nx);
    end
end

% add necessary fields and create an MPTCTRL object
ctrl.overlaps = overlaps;
% if probStruct.norm==2,
%     % overlaps are possible with quadratic cost
%     ctrl.overlaps = 1;
% else
%     % overlaps have been removed for linear cost
%     ctrl.overlaps = 0;
% end

ctrl.sysStruct = SST{1};
ctrl.probStruct = probStruct;
ctrl.details.origSysStruct = origSysStruct;
ctrl.details.origProbStruct = origProbStruct;
ctrl.details.runTime = cputime - starttime;
try
    ctrl = mptctrl(ctrl);
end



%-----------------------------------------------------------------
function [t, s] = sub_inputtype(sysStruct, ind)
% t='C', s=[]                     if input 'ind' is continuous
% t='B', s=[0 1]                  if input 'ind' is boolean
% t='A', s=sysStruct.Uset{ind}    if input 'ind' is from an alphabet

if isfield(sysStruct, 'Uset'),
    if isequal(sort(sysStruct.Uset{ind}), [0 1]),
        % input is boolean
        t = 'B';
        s = [0 1];
        
    elseif all(isinf(sysStruct.Uset{ind})),
        % input is continuous
        t = 'C';
        s = [];
        
    else
        % input is from finite alphabet
        t = 'A';
        s = sysStruct.Uset{ind};
    end
    
else
    % all inputs are continuous
    t = 'C';
    s = [];
end


%-----------------------------------------------------------------
function yesno = sub_allYeqsEqual(sysStruct)
% returns true if sysStruct.C, sysStruct.D and sysStruct.g are identical for all
% dynamics

yesno = 1;
if ~iscell(sysStruct),
    sysStruct = {sysStruct};
end
C = sysStruct{1}.C{1};
D = sysStruct{1}.D{1};
g = sysStruct{1}.g{1};
for ii = 1:length(sysStruct)
    for jj = 1:length(sysStruct{ii}.C),
        if ~isequal(C, sysStruct{ii}.C{jj}) | ...
                ~isequal(D, sysStruct{ii}.D{jj}) | ~isequal(g, sysStruct{ii}.g{jj}),
            yesno = 0;
            return
        end
    end
end


%-----------------------------------------------------------------
function probStruct = sub_timevarpenalties(probStruct)
% if probStruct.[R,Q,Qy] are single matrices, we convert them to a cell array of
% length probStruct.N

penalties = {'R', 'Q', 'Rdu', 'Qy', 'Qd', 'Qz', 'Qdyn', 'Qswitch'};

for ii = 1:length(penalties),
    if isfield(probStruct, penalties{ii}),
        P = getfield(probStruct, penalties{ii});
        if ~iscell(P),
            Pcell = cell(1, probStruct.N);
            [Pcell{:}] = deal(P);
            probStruct = setfield(probStruct, penalties{ii}, Pcell);
        end
    end
end


%-----------------------------------------------------------------
function [Tset, P_N, origin_in_dynamics] = sub_stabilizingset(sysStruct, probStruct, nx, nPWA)
% computes a stabilizing set for a given PWA system (LTI systems are
% converted to PWA systems anyhow in the main function)

global mptOptions

Tset = polytope;
P_N = [];

% first find out which dynamics contain the origin in their interior
%
% here we could take xref and uref from probStruct.xref and probStruct.uref,
% respectively. but mpt_control already takes over this and performes change of
% coordinates by calling mpt_prepareTracking, therefore we must use zero
% reference in the sequel 
%
origin = zeros(nx, 1);

origin_in_dynamics = [];

for ii=1:nPWA
    if(all(sysStruct.f{ii}==0)) | isfield(probStruct,'xref') | isfield(probStruct,'uref')
        %.... othewise the origin would not be an equilibrium point
        if all(sysStruct.guardU{ii}==0) & ...
                max(sysStruct.guardX{ii}*origin - sysStruct.guardC{ii}) <= mptOptions.abs_tol,
            origin_in_dynamics = [origin_in_dynamics ii];
            
        elseif(any(sysStruct.guardU{ii}~=0))
            tempP=polytope(sysStruct.guardU{ii}, ...
                -sysStruct.guardX{ii}*origin + sysStruct.guardC{ii});
            
            if(isfulldim(tempP))
                origin_in_dynamics = [origin_in_dynamics ii];
            end
            
        end
    end
end
if isempty(origin_in_dynamics),
    return
end
Options.verbose = -1;
[Tset,Fi,Gi,dynamics,pst] = mpt_computePWATset(sysStruct, probStruct, origin_in_dynamics, Options);
P_N = pst.P_N;


%-----------------------------------------------------------------
function o = sub_norm(x, P, n)

if n == 2,
    o = x'*P*x;
else
    o = norm(P*x, n);
end


%-----------------------------------------------------------------
function SST = sub_sysStruct2cell(sysStruct, N)
% convert single sysStruct into a cell array

SST = cell(1, N);
[SST{:}] = deal(sysStruct);



%-----------------------------------------------------------------
function [dynamics_type, nPWA, MLD, SST, sysStruct] = sub_checkmodels(SST, sysStruct)
% checks whether we have MLD, PWA or LTI systems

dynamics_type = cell(1, length(SST));
nPWA = zeros(1, length(SST));
MLD = cell(1, length(SST));

for im = 1:length(SST),
    if isfield(SST{im}, 'data'),
        % we do have an MLD representation
        if isfield(SST{im}.data, 'onlymld'),
            % there is no equivalent PWA representation for this system available
            MLD{im} = SST{im}.data.MLD
            % initialize B5 and D5 matrices to zero if they are not given
            if ~isfield(MLD{im}, 'B5'),
                MLD{im}.B5 = zeros(size(MLD{im}.A, 1), 1);
            end
            if ~isfield(MLD{im}, 'D5'),
                MLD{im}.D5 = zeros(size(MLD{im}.C, 1), 1);
            end
            dynamics_type{im} = 'mld';
            
        else
            % we also have the PWA representation available for this system, use it
            dynamics_type{im} = 'pwa';
            
        end
        
    elseif iscell(SST{im}.A),
        dynamics_type{im} = 'pwa';
        nPWA(im) = length(SST{im}.A);
        
    else
        % convert the LTI system into a PWA form, it will allow simplier code later
        dynamics_type{im} = 'lti';
        SST{im} = mpt_lti2pwa(SST{im});
        sysStruct = mpt_lti2pwa(sysStruct);
        nPWA(im) = 1;
        
    end
end


%-----------------------------------------------------------------
function val = sub_defaultfield(S, fname, default)
% returns S.fname if 'fname' is a valid field of the structure "S". Otherwise
% returns "default".

if isfield(S, fname),
    val = getfield(S, fname);
elseif nargin==3,
    val = default;
else
    val = [];
end



%-----------------------------------------------------------------
function [soften, slacks, smax, sweights] = sub_prepareslacks(probStruct, N, haveXbounds, haveYbounds, dims)
% introduce slack variables for soft constraints if necessary

soften = struct('all', 0, 'x', 0, 'u', 0, 'y', 0);
smax = struct('all', 0, 'x', 0, 'u', 0, 'y', 0);
sweights = struct('all', 0, 'x', 0, 'u', 0, 'y', 0);
slacks.all = cell(1, N); [slacks.all{:}] = deal(0);
slacks.x = cell(1, N); [slacks.x{:}] = deal(zeros(dims.nx, 1));
slacks.y = cell(1, N); [slacks.y{:}] = deal(zeros(dims.ny, 1));
slacks.u = cell(1, N); [slacks.u{:}] = deal(zeros(dims.nu, 1));

if isfield(probStruct, 'S') | isfield(probStruct, 'smax'),
    % only one slack which softens state, input and output constraints
    % simultaneously
    slacks.all = sdpvar(repmat(1, 1, N), repmat(1, 1, N));
    soften.all = 1;
    % upper bound on this slack
    smax.all = sub_defaultfield(probStruct, 'smax', Inf);
    % penalty on slacks
    sweights.all = sub_defaultfield(probStruct, 'S', 1e3);
    
else
    if isfield(probStruct, 'Sx') | isfield(probStruct, 'sxmax'),
        % softening of state constraints
        if ~haveXbounds,
            fprintf('WARNING: no state constraints given, cannot soften them.\n');
        else
            slacks.x = sdpvar(repmat(dims.nx, 1, N), repmat(1, 1, N));
            soften.x = 1;
            % upper bound on this slack
            smax.x = sub_defaultfield(probStruct, 'sxmax', Inf);
            % penalty on slacks
            sweights.x = sub_defaultfield(probStruct, 'Sx', 1e3);
        end
    end
    if isfield(probStruct, 'Sy') | isfield(probStruct, 'symax'),
        % softening of output constraints
        if ~haveYbounds,
            fprintf('WARNING: no output constraints given, cannot soften them.\n');
        else
            slacks.y = sdpvar(repmat(dims.ny, 1, N), repmat(1, 1, N));
            soften.y = 1;
            % upper bound on this slack
            smax.y = sub_defaultfield(probStruct, 'symax', Inf);
            % penalty on slacks
            sweights.y = sub_defaultfield(probStruct, 'Sy', 1e3);
        end
    end
    if isfield(probStruct, 'Su')  | isfield(probStruct, 'sumax'),
        % softening of input constraints
        slacks.u = sdpvar(repmat(dims.nu, 1, N), repmat(1, 1, N));
        soften.u = 1;
        % upper bound on this slack
        smax.u = sub_defaultfield(probStruct, 'sumax', Inf);
        % penalty on slacks
        sweights.u = sub_defaultfield(probStruct, 'Su', 1e3);
    end
end

smax.all = smax.all(:); smax.x = smax.x(:); smax.y = smax.y(:); smax.u = smax.u(:);