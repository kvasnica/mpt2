function [newCtrlStruct]=mpt_removeOverlaps(Partition,Options)
%MPT_REMOVEOVERLAPS Removes overlaps from (a set of) polyhedral partitions with associated linear cost
%
% [newCtrlStruct]=mpt_removeOverlaps(ctrl,Options)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Given 'n' (possibly overlapping) polyhedral partitions with associated linear cost,
% this function detect such overlaps and removes them by picking up regions which
% have the least cost associated to them. If intersection of the overlapping regions
% is non-empty, slicing is introduced to create one non-overlapping polyhedral
% partition.
% 
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% ctrl        - Explicit controller a cell aray thereof; with fields:
%      Pfinal - maximum feasible set (i.e. the outer hull of U Pn)
%      Pn     - polytopic regions
%      Fi,Gi  - cells containing control law (u = Fi{j} x + Gi{j})
%      Bi,Ci  - cells containing the linear cost (i.e. J = Bi{j} x + Ci{j})
% Options.verbose   - level of verbosity
% Options.lpsolver  - default solver for LP's
% Options.abs_tol   - absolute tolerance
% Options.rel_tol   - relative tolerance
% Options.lowmem    - defines memory saving mode
%                       0 - no memory saving - fast computation (default)
%                       1 - slight memory saving
%                       2 - heavy memory saving (slow computation)
%
% Note: If Options is missing or some of the fiels are not defined, the default
%       values from mptOptions will be used
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% newCtrl    - controller which consists of purely non-overlapping regions
%
% see also MPT_ITERATIVEPWA, MPT_ITERATIVE, MPT_OPTCONTROLPWA, MPT_PLOTU
%

% $Id: mpt_removeOverlaps.m,v 1.4 2005/06/25 15:02:49 kvasnica Exp $
%
% (C) 2003-2005 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%               kvasnica@control.ee.ethz.ch
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

error(nargchk(1,2,nargin));

global mptOptions;

if ~isstruct(mptOptions),
    mpt_error;
end

if nargin<2,
    Options=[];
end

if ~isfield(Options,'abs_tol')
    Options.abs_tol=mptOptions.abs_tol;     % default absolute tolerance
end
if ~isfield(Options,'rel_tol')
    Options.rel_tol=mptOptions.rel_tol;     % default relative tolerance
end
if ~isfield(Options,'verbose')
    Options.verbose=mptOptions.verbose;
end
if ~isfield(Options,'lpsolver')
    Options.lpsolver = mptOptions.lpsolver;
end
if ~isfield(Options,'lowmem')
    Options.lowmem=0;
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

lowmem0 = (Options.lowmem==0);
lowmem1 = (Options.lowmem==1);
lowmem2 = (Options.lowmem==2);

rel_tol = Options.rel_tol;
abs_tol = Options.abs_tol;
lpsolver = Options.lpsolver;

if isa(Partition, 'polytope'),
    % input is a polytope array, convert it to a dummy controller structure
    input_is_polytope = 1;
    Partition = mpt_dummyCS(Partition);
else
    input_is_polytope = 0;
end

mptctrl_input = 0;
progress = 0;

if ~iscell(Partition)
    if isa(Partition, 'mptctrl')
        if ~isexplicit(Partition)
            error('This function supports only explicit controllers!');
        end
        Partition = struct(Partition);
        mptctrl_input = 1;
    end
    if ~mpt_isValidCS(Partition),
        error('Input argument must be either a cell array containing polyhedral partitions or a valid ctrlStruct structure!');
    end
    % convert ctrlStruct into "Partition" cells (see help above)
    
    PPfinal = Partition.Pfinal;
    Options.noSectionCheck = 1;
    ctrlStruct = Partition;
    if ~ctrlStruct.overlaps,
        disp('mpt_removeOverlaps: Partitions do not overlap, nothing to do here...');
        newCtrlStruct = ctrlStruct;
        if mptctrl_input
            newCtrlStruct = mptctrl(newCtrlStruct);
        end
        return
    end
    if ctrlStruct.probStruct.subopt_lev == 1,
        newCtrlStruct = sub_removeOverlaps_MinTime(ctrlStruct);
        if mptctrl_input
            newCtrlStruct = mptctrl(newCtrlStruct);
        end
        return
    end
    sysStruct = Partition.sysStruct;
    probStruct = Partition.probStruct;
    details = Partition.details;
    Partition = cell(1,length(ctrlStruct.Pn));
    for ii=1:length(ctrlStruct.Pn),
        Partition{ii}.Pn = ctrlStruct.Pn(ii);
        Partition{ii}.Pfinal = Partition{ii}.Pn;
        Partition{ii}.Fi{1} = ctrlStruct.Fi{ii};
        Partition{ii}.Gi{1} = ctrlStruct.Gi{ii};
        Partition{ii}.Ai{1} = ctrlStruct.Ai{ii};
        Bi = ctrlStruct.Bi{ii};
        Partition{ii}.Bi{1} = (Bi(:))';
        Partition{ii}.Ci{1} = ctrlStruct.Ci{ii};
        Partition{ii}.dynamics = ctrlStruct.dynamics(ii);
    end
else
    Options.noSectionCheck = 0;
    PPfinal = [];
    for ii=1:length(Partition),
        if isa(Partition{ii}, 'mptctrl')
            if ~isexplicit(Partition{ii})
                error('This function supports only explicit controllers!');
            end
            Partition{ii} = struct(Partition{ii});
            mptctrl_input = 1;
        end
            
        if ~mpt_isValidCS(Partition{ii}),
            error('Each element of the cell array has to be a valid ctrlStruct structure!');
        end
        PPfinal = [PPfinal Partition{ii}.Pfinal];
        if ~isfield(Partition{ii},'dynamics')
            Partition{ii}.dynamics = zeros(1,length(Partition{ii}.Pn));
        end
    end
    sysStruct = Partition{1}.sysStruct;
    probStruct = Partition{1}.probStruct;
    details = Partition{1}.details;
end

npart = length(Partition);      % number of polyhedral partitions
emptypoly = polytope;
Pn = emptypoly;
Fi = {};
Gi = {};
Ai = {};
Bi = {};
Ci = {};
dynamics = [];
nR = 0;

intOptions=Options;
intOptions.reduce_intersection=0; % we don't want to construct the intersection, just determine if it exist
Options.reduce_intersection = 1;

% check if cost contains quadratic cost
for ii=1:npart,
    Aai = [Partition{ii}.Ai{:}];
    if ~all(all(Aai==0)),
        error('mpt_removeOverlaps: Quadratic terms in the cost function not allowed!');
    end
end
clear Aai

% look for duplicate regions, i.e. regions which have identical cost, control
% law and regions are the same. if such regions is identified, remove them from
% Partition list (be carefull not to remove both identical regions, just one of
% them!)

if statusbar,
    if ~isfield(Options, 'status_handle')
        Options.status_handle = mpt_statusbar('Removing overlaps...');
        closestatbar = 1;
    end
end

remove_regions = cell(1,npart);
nRemoved = 0;
for ii=1:npart,
    %sbprogress(pbarh,ceil(ii/npart*100));
    part_ii = Partition{ii};
    part_Pn_ii = part_ii.Pn;
    if statusbar,
        if isempty(mpt_statusbar(Options.status_handle, progress, Options.status_min, Options.status_max)),
            mpt_statusbar;
            error('Break...');
        end     
    end
    for jj=ii+1:npart,
        part_jj = Partition{jj};
        part_Pn_jj = part_jj.Pn;
        for kk=1:length(part_ii.Fi),
            part_ii_Fi_kk = part_ii.Fi{kk};
            part_ii_Gi_kk = part_ii.Gi{kk};
            part_ii_Bi_kk = part_ii.Bi{kk};
            part_ii_Ci_kk = part_ii.Ci{kk};
            for mm=1:length(part_jj.Fi),
                %%part_jj_Ci_mm = part_jj.Ci{mm};
                if abs(part_ii_Ci_kk - part_jj.Ci{mm}) <= abs_tol,
                    if all(all(abs(part_ii_Bi_kk - part_jj.Bi{mm}) <= abs_tol)),
                        if all(all(abs(part_ii_Fi_kk - part_jj.Fi{mm}) <= abs_tol)),
                            if all(abs(part_ii_Gi_kk - part_jj.Gi{mm}) <= abs_tol),
                                if part_ii.Pn(kk) == part_jj.Pn(mm),
                                    % cost, control law and regions are identical, remove one of
                                    % them
                                    if isempty(remove_regions{jj}),
                                        remove_regions{jj}=mm;
                                    else
                                        remove_regions{jj} = [remove_regions{jj} mm];
                                    end
                                    nRemoved = nRemoved + 1;
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end
%close(nbarh);
clear part_ii_Fi_kk part_ii_Gi_kk part_ii_Bi_kk part_ii_Ci_kk
clear part_ii part_Pn_ii part_jj part_Pn_jj
if nRemoved>0 & Options.verbose > 0,
    disp(sprintf('Removed %d duplicate regions',nRemoved));
end

% ----------------------------------------------------------------
% this part can be further optimized for speed

for part=1:length(remove_regions),
    if isempty(remove_regions{part}),
        continue
    end
    regions = remove_regions{part};
    onepart = Partition{part};
    Fi = {};
    Gi = {};
    Ai = {};
    Bi = {};
    Ci = {};
    for jj=1:length(Partition{part}.Fi),
        if ~any(jj==regions)
            Fi{end+1} = Partition{part}.Fi{jj};
            Gi{end+1} = Partition{part}.Gi{jj};
            Ai{end+1} = Partition{part}.Ai{jj};
            Bi{end+1} = Partition{part}.Bi{jj};
            Ci{end+1} = Partition{part}.Ci{jj};
        end
    end
    Pn = Partition{part}.Pn;
    Pn(regions) = [];
    dynamics = Partition{part}.dynamics;
    dynamics(regions) = [];
    onepart.Pfinal = Partition{part}.Pfinal;
    onepart.Pn = Pn;
    onepart.Fi = Fi;
    onepart.Gi = Gi;
    onepart.Ai = Ai;
    onepart.Bi = Bi;
    onepart.Ci = Ci;
    onepart.dynamics = dynamics;
    Partition{part} = onepart;
end
newPart = {};
keptParts = [];
for ii=1:npart,
    if length(Partition{ii}.Fi)>0,
        newPart{end+1} = Partition{ii};
        keptParts(end+1) = ii;
    end
end
Partition= newPart;
npart = length(Partition);
clear remove_regions onepart newPart Pn Ai Fi Gi Bi Ci
% ----------------------------------------------------------------

isPartPfinalFullDim = ones(npart,1);
for ii=1:npart,
    if ~isfulldim(Partition{ii}.Pfinal) | isempty(Partition{ii}.Fi),
        isPartPfinalFullDim(ii)=0;
    end
end
nx = dimension(Partition{1}.Pn);
abs_tol = Options.abs_tol;

Pn = emptypoly;
Fi = {};
Gi = {};
Ai = {};
Bi = {};
Ci = {};
dynamics = [];
nR = 0;

% compute bounding box of each polytope in each partition and store it for later
% use
boxOpt = Options;
boxOpt.noPolyOutput = 1;

if lowmem0 | lowmem1,
    boxes = cell(1,npart);
    for ii=1:npart,
        lenPn = length(Partition{ii}.Fi);
        boxes{ii} = cell(1,lenPn);
        for jj=1:lenPn,
            [R, low, up] = bounding_box(Partition{ii}.Pn(jj),boxOpt);
            boxes{ii}{jj}.low = low;
            boxes{ii}{jj}.up = up;
        end
    end
end

if statusbar,
    if isempty(mpt_statusbar(Options.status_handle, progress, Options.status_min, Options.status_max)),
        mpt_statusbar;
        error('Break...');
    end     
end

if Options.verbose>0,
    if lowmem0
        disp('Removing overlaps...');
    elseif lowmem1
        disp('Removing overlaps (memory saving mode)...');
    else
        disp('Removing overlaps (maximum memory saving mode)...');
    end
end

if lowmem0
    Hstore = cell(1,npart);
    Kstore = cell(1,npart);
    for ii = 1:npart,
        lenFi = length(Partition{ii}.Fi);
        Hstore{ii} = cell(1, lenFi);
        Kstore{ii} = cell(1, lenFi);
        for jj = 1:lenFi,
            [Hstore{ii}{jj}, Kstore{ii}{jj}] = double(Partition{ii}.Pn(jj));
        end
    end
end

keptPartsIdx = [];

for ii=1:npart

    if Options.verbose > -1,
        if mod(ii,round(npart/3))==0 | ii==1 | ii==npart,
            %sbprogress(pbarh,ceil(ii/npart*100));
            fprintf('hull %d/%d\t',ii,npart);
        end
    end
    progress = ii/npart;
    PartitionII = Partition{ii};
    if lowmem0 | lowmem1
        boxes_ii = boxes{ii};
    end
    
%     if statusbar,
%         if isempty(mpt_statusbar(Options.status_handle, progress, Options.status_min, Options.status_max)),
%             mpt_statusbar;
%             error('Break...');
%         end     
%     end
    
    for jj=1:length(PartitionII.Fi)
        
        Intersection=[];  % intersection information
        Intersection.Pn = emptypoly;
        Intersection.nR=0;
        
        Part_ii_Pn_jj = PartitionII.Pn(jj);
        
        if lowmem2 | lowmem1,
            [H1,K1] = double(Part_ii_Pn_jj);
        else
            H1 = Hstore{ii}{jj};
            K1 = Kstore{ii}{jj};
        end
        
        if lowmem0 | lowmem1
            low1 = boxes_ii{jj}.low;
            up1 = boxes_ii{jj}.up;
            low1tol = low1 + abs_tol;
            up1tol = up1 + abs_tol;
        end
        
%         if statusbar,
%             if isempty(mpt_statusbar(Options.status_handle, progress, Options.status_min, Options.status_max)),
%                 mpt_statusbar;
%                 error('Break...');
%             end     
%         end
        
        for kk=1:npart
            if kk==ii
                % do not consider overlaps within of the same partition
                %
                % by Options.noSectionCheck we also indicate that the regions are already sorted in
                % descending order by value function and we don't need to compare partitions with greater cost (set distance)
                continue;
            end
            
            if lowmem0 | lowmem1
                boxes_kk = boxes{kk};
            end
            
            PartitionKK = Partition{kk};
            
            if statusbar,
                if isempty(mpt_statusbar(Options.status_handle, progress, Options.status_min, Options.status_max)),
                    mpt_statusbar;
                    error('Break...');
                end     
            end
            
            for mm=1:length(PartitionKK.Fi)

                auxI_mm = 1;
            
                if lowmem0 | lowmem1
                    low2 = boxes_kk{mm}.low;
                    up2 = boxes_kk{mm}.up;
                    low2tol = low2 + abs_tol;
                    up2tol = up2 + abs_tol;
                else
                    [R, low1, up1] = bounding_box(Partition{ii}.Pn(jj),boxOpt);
                    [R, low2, up2] = bounding_box(Partition{kk}.Pn(mm),boxOpt);
                    low1tol = low1+abs_tol;
                    up1tol = up1+abs_tol;
                    low2tol = low2+abs_tol;
                    up2tol = up2+abs_tol;
                end
                auxI_mm = 1;
                for qq=1:nx,
                    if ~( (low2(qq)<=low1tol(qq) & low1(qq)<=up2tol(qq)) | ...
                            (low2(qq)<=up1tol(qq) & up1(qq)<=up2tol(qq)) | ...
                            (low1(qq)<=low2tol(qq) & low2(qq)<=up1tol(qq)) | ...
                            (low1(qq)<=up2tol(qq) & up2(qq)<=up1tol(qq)) )
                        % bounding boxes do not intersect, hence polytopes
                        % do not intersect as well
                        auxI_mm = 0;
                        break
                    else
                        continue
                    end
                end
                
                if auxI_mm == 0,
                    % if polytope Partition{kk}.Pn(mm) is not intersecting the feasible set of Partition{ii}, skip it
                    continue
                end
                
                if lowmem2 | lowmem1,
                    [H2,K2] = double(PartitionKK.Pn(mm));
                else
                    H2 = Hstore{kk}{mm};
                    K2 = Kstore{kk}{mm};
                end
                
                Haux = [H1; H2];
                Kaux = [K1; K2];
                % get radius of chebyshev's ball of the intersection
                if auxI_mm==3,
                    fulldim=1;
                else
                    [xcheb, R] = sub_chebyball(Haux,Kaux,nx,rel_tol,abs_tol,lpsolver);
                    fulldim = (R>=abs_tol);
                end
                if fulldim
                    
                    % intersection is non-empty
                    Baux=PartitionKK.Bi{mm}-PartitionII.Bi{jj};
                    Caux=PartitionKK.Ci{mm}-PartitionII.Ci{jj};
                    % if costs are the same for two regions be careful not to remove both regions
                    % here we keep only the region belonging to the first partition
                    if sum(abs([Baux Caux]),2)<=abs_tol                       % is the cost identical?
                        if ii<kk,                                                     % if so, we just remove the region once
                            Intersection.nR=Intersection.nR+1;
                            Intersection.who(Intersection.nR,:)=[kk mm];
                            Paux = polytope(Haux, Kaux, 0, 2, xcheb, R);
                            Intersection.Pn = [Intersection.Pn Paux];
                        end
                    else
                        % find a part of the region in which cost associated to index 'mm' is lower than the one of index 'jj'
                        H = [Haux; Baux];
                        K = [Kaux; -Caux];
                        % get radius of chebyshev's ball of the intersection
                        [xcheb, R] = sub_chebyball(H,K,nx,rel_tol,abs_tol,lpsolver);
                       
                        if R>=abs_tol,
                            % faster call - do not reduce the polytope yet
                            Paux = polytope(H, K, 0, 2, xcheb, R);
                            Intersection.nR=Intersection.nR+1;
                            Intersection.who(Intersection.nR,:)=[kk mm];
                            Intersection.Pn = [Intersection.Pn Paux];
                        end
                    end
                end
            end % mm
        end % kk
        
        if statusbar,
            if isempty(mpt_statusbar(Options.status_handle, progress, Options.status_min, Options.status_max)),
                mpt_statusbar;
                error('Break...');
            end     
        end
        
        if Intersection.nR>0
            % get all segments which have the same 'minimal' cost
            Ri = regiondiff(PartitionII.Pn(jj), Intersection.Pn, Options);          
            if ~isfulldim(Ri(1)) % union is covering Ri
                if Options.verbose>1,
                    disp(['      reg ' num2str([ii jj]) '     ']);
                end
                continue;
            else
                if Options.verbose>1,
                    disp([' ==== reg ' num2str([ii jj]) ' ==== kept']);
                end
                Pn = [Pn Ri];
                for mm=1:length(Ri)
                    nR = nR+1;
                    Fi{nR} = PartitionII.Fi{jj};
                    Gi{nR} = PartitionII.Gi{jj};
                    Ai{nR} = PartitionII.Ai{jj};
                    Bi{nR} = PartitionII.Bi{jj};
                    Ci{nR} = PartitionII.Ci{jj};
                    dynamics = [dynamics PartitionII.dynamics(jj)];
                    if ( ~any(keptPartsIdx == ii) ),
                        keptPartsIdx(end+1) = ii;
                    end
                end
            end
        else
            if Options.verbose>1,
                disp([' ==== reg ' num2str([ii jj]) ' ==== kept']);
            end
            nR = nR+1;
            Pn = [Pn Part_ii_Pn_jj];
            Fi{nR} = PartitionII.Fi{jj};
            Gi{nR} = PartitionII.Gi{jj};
            Ai{nR} = PartitionII.Ai{jj};
            Bi{nR} = PartitionII.Bi{jj};
            Ci{nR} = PartitionII.Ci{jj};
            dynamics = [dynamics PartitionII.dynamics(jj)];
            if ( ~any(keptPartsIdx == ii) ),
                keptPartsIdx(end+1) = ii;
            end
        end
    end % jj
end % ii 
clear Intersection PartitionII
if Options.verbose > -1,
    fprintf('\n');
end

if statusbar,
    if isempty(mpt_statusbar(Options.status_handle, 1, Options.status_min, Options.status_max)),
        mpt_statusbar;
        error('Break...');
    end     
end

if iscell(Partition)
    newCtrlStruct.sysStruct = sysStruct;
    newCtrlStruct.probStruct = probStruct;
end
if Options.noSectionCheck
    if ~exist('ctrlStruct','var'),
        ctrlStruct = Partition{1};
    else
        newCtrlStruct = ctrlStruct;
    end
end

if closestatbar,
    mpt_statusbar;
end

if input_is_polytope
    % return only Pn if input was not a controller structure
    newCtrlStruct = Pn;
    return
end

%newCtrlStruct.Pfinal = reduceunion(PPfinal);
%newCtrlStruct.Pfinal = Pn;
details.keptParts = keptParts(keptPartsIdx);
newCtrlStruct.Pfinal = PPfinal;
newCtrlStruct.Pn = Pn;
newCtrlStruct.Fi = Fi;
newCtrlStruct.Gi = Gi;
newCtrlStruct.Ai = Ai;
newCtrlStruct.Bi = Bi;
newCtrlStruct.Ci = Ci;
newCtrlStruct.dynamics = dynamics;
newCtrlStruct.details = details;
newCtrlStruct.overlaps = 0;

if mptctrl_input
    % if input was an MPTCTRL object, return MPTCTRL object
    newCtrlStruct = mptctrl(newCtrlStruct);
end
clear RegionIntersect



return


% ---------------------------------------------------------------------------------------
function ctrlStruct = sub_ro_minTime(ctrlStruct)

PA = ctrlStruct.Pn;
PA = fliplr(PA);
ctr = 1;
PPA = PA(end);
FFi{ctr} = ctrlStruct.Fi{1};
GGi{ctr} = ctrlStruct.Gi{1};
AAi{ctr} = ctrlStruct.Ai{1};
BBi{ctr} = ctrlStruct.Bi{1};
CCi{ctr} = ctrlStruct.Ci{1};
DD = ctrlStruct.dynamics(1);

lenPn = length(PA);        
for ii=2:lenPn
    i=lenPn-ii+1;
    if mod(ii,20)==0,
        disp(['Region ' num2str(ii) '/' num2str(lenPn)])
    end
    tmpA=PA((i+1):lenPn);
    tmpA=PA(i)\tmpA;
    if(isfulldim(tmpA))
        for j=1:length(tmpA)
            ctr=ctr+1;
            PPA = [PPA tmpA(j)];
            FFi{ctr}=ctrlStruct.Fi{ii};
            GGi{ctr}=ctrlStruct.Gi{ii};
            AAi{ctr}=ctrlStruct.Ai{ii};
            BBi{ctr}=ctrlStruct.Bi{ii};
            CCi{ctr}=ctrlStruct.Ci{ii};
            DD = [DD ctrlStruct.dynamics(ii)];
        end
    end
end 
ctrlStruct.Pn = PPA;
ctrlStruct.Fi = FFi;
ctrlStruct.Gi = GGi;
ctrlStruct.Ai = AAi;
ctrlStruct.Bi = BBi;
ctrlStruct.Ci = CCi;
ctrlStruct.dynamics = DD;
ctrlStruct.overlaps = 0;


% ---------------------------------------------------------------------------------------
function [xcheb, R]=sub_chebyball(H,K,nx,rel_tol,abs_tol,lpsolver)

if all(K>-1e9),
    % use 'rescue' function - resolve an LP automatically if it is infeasible
    % with default solver
    [xopt,fval,lambda,exitflag,how]=mpt_solveLPi([zeros(1,nx) 1],[H, -sqrt(sum(H.*H,2))],...
        K,[],[],[],lpsolver,1);
else
    how = 'infeasible';
end

if ~strcmp(how,'ok')
    % maybe there is a numerical problem, thus we normalize H and K
    
    [nc,nx]=size(H);
    Anorm=sqrt(sum(H .* H,2));
    ii=find(Anorm<=abs_tol | Anorm<=abs(rel_tol*K));
    nii=length(ii);
    if nii>0
        Anorm(ii)=1;
        H(ii,:)=repmat([1 zeros(1,nx-1)],nii,1);
        % decide if some constraint is always true
        jj=(K(ii)>=-abs_tol);
        K(ii(jj))=Inf;
        % or is it always false
        K(ii(~jj))=-Inf;
    end
    temp=1./Anorm;
    H=H .* temp(:,ones(1,nx));
    K=K .* temp;
    
    % If any boundary is -Inf polytope P is empty
    %--------------------------------------------
    if any(K==-Inf)
        xc=zeros(nx,1);
        R=-Inf;
        lambda=zeros(nc,1);
        return;
    end
    
    % Remove rows with Inf boundaries
    %--------------------------------
    ii=(K==Inf);
    H(ii,:)=[];
    K(ii)=[];
    
    if size(H,1)==0
        xc=zeros(nx,1);
        R=Inf;
        lambda=zeros(nc,1);
        return;
    end
    
    x0 = [zeros(nx,1); 1000];         % hard-coded initial conditions

    % use 'rescue' function - resolve an LP automatically if it is infeasible
    % with default solver
    [xopt,fval,lambda,exitflag,how]=mpt_solveLPi([zeros(1,nx) 1],[H, -sqrt(sum(H.*H,2))],...
        K,[],[],x0,lpsolver,1);
end

xcheb = xopt(1:nx); % center of the ball
R=-xopt(nx+1); % Radius of the ball



% ----------------------------------------------------------------------------
function [ctrlStruct] = sub_removeOverlaps_MinTime(ctrlStruct)

lenPn = length(ctrlStruct.Pn);
Pn = polytope; Fi = {}; Gi = {}; Ai = {}; Bi = {}; Ci = {}; dyn = [];
for ii=lenPn:-1:2,
    count = lenPn-ii+1;
    if count==1 | count==lenPn-1 | mod(count,20)==0,
        if count==lenPn-1,
            count=lenPn;
        end
        disp(['Region ' num2str(count) '/' num2str(lenPn)])
    end
    P = ctrlStruct.Pn(ii) \ ctrlStruct.Pn(1:ii-1);
    if isfulldim(P(1)),
        Pn = [Pn P];
        for jj=1:length(P),
            Fi{end+1} = ctrlStruct.Fi{ii};
            Gi{end+1} = ctrlStruct.Gi{ii};
            Ai{end+1} = ctrlStruct.Ai{ii};
            Bi{end+1} = ctrlStruct.Bi{ii};
            Ci{end+1} = ctrlStruct.Ci{ii};
            dyn = [dyn ctrlStruct.dynamics(ii)];
        end
    end
end
Pn = [Pn ctrlStruct.Pn(1)];
dyn = [dyn ctrlStruct.dynamics(1)];
Pn = fliplr(Pn);
dyn = fliplr(dyn);
Fi{end+1} = ctrlStruct.Fi{1};
Gi{end+1} = ctrlStruct.Gi{1};
Ai{end+1} = ctrlStruct.Ai{1};
Bi{end+1} = ctrlStruct.Bi{1};
Ci{end+1} = ctrlStruct.Ci{1};

ctrlStruct.Pn = Pn;
ctrlStruct.Fi = sub_flip_cell(Fi);
ctrlStruct.Gi = sub_flip_cell(Gi);
ctrlStruct.Ai = sub_flip_cell(Ai);
ctrlStruct.Bi = sub_flip_cell(Bi);
ctrlStruct.Ci = sub_flip_cell(Ci);
ctrlStruct.dynamics = dyn;
ctrlStruct.overlaps = 0;

return


function [fC]=sub_flip_cell(C)

lenC = length(C);
fC=cell(1,lenC);
for ii=1:lenC,
    fC{lenC-ii+1}=C{ii};
end
