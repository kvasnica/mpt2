function handle=mpt_plotPartition(ctrl,Options)
%MPT_PLOTPARTITION Plots a polyhedral partition obtained by mpt_control
%
% handle=mpt_plotPartition(ctrl)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Plots the polyhedral parition obtained as a solution of the 
% optimal control problem obtained by mpt_control
%
% USAGE:
%   mpt_plotPartition(ctrl)
%   mpt_plotPartition(ctrl, Options)
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% ctrl             - Explicit controller (MPTCTRL object)
% Options.noInfCol - Supress green-shade coloring for time-optimal solutions
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% handle       - handle to the graph
%
% see also MPT_PLOTTRAJECTORY, MPT_PLOTPWA, MPT_PLOTPWQ, MPT_CONTROL
%

% Copyright is with the following author(s):
%
% (C) 2003-2005 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%               kvasnica@control.ee.ethz.ch
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

error(nargchk(1,2,nargin));

global mptOptions;

if ~isstruct(mptOptions),
    mpt_error;
end

if nargin<1,
    error('mpt_plotPartition: wrong number of input arguments!');
end

if nargin<2,
    Options = mptOptions;
end

if isa(ctrl, 'mptctrl')
    if ~isexplicit(ctrl)
        error('This function supports only explicit controllers!');
    end
    ctrlStruct = struct(ctrl);
else
    ctrlStruct = ctrl;
end

if ~isfield(Options,'noInfCol'),
    Options.noInfCol=0;
end
if ~isfield(Options,'newfigure')
    Options.newfigure=mptOptions.newfigure;
end


if ~mpt_isValidCS(ctrlStruct)
    error('mpt_plotPartition: First argument has to be a valid controller structure! See mpt_control for details.');
end

% if regions do not overlap, we can enforce break in isinside at least it finds the first region associated
% to a given state (since there cannot be more than one if there are no overlaps)

Options.fastbreak = ~ctrlStruct.overlaps;

titlehandle=[];

details = ctrlStruct.details;
if (iscell(details) & isfield(details{1},'setdist')) | ...
        (iscell(ctrlStruct.sysStruct.A) & ctrlStruct.probStruct.subopt_lev==1),
    % iterative PWA solution
    lC = length(ctrlStruct.Ci);
    details = zeros(lC,1);
    for ii=1:lC
        details(lC-ii+1) = ctrlStruct.Ci{ii};
    end
elseif isfield(details,'regionHorizon'),
    details = details.regionHorizon;
elseif isfield(details,'IterStore') & ctrlStruct.overlaps,
    details = details.IterStore;
else
    details = ctrlStruct.details;
end

if Options.newfigure,
    figure;
    Options.newfigure = 0;  % we need to tell polytope/plot() not to open a yet another window
end


PA = fliplr(ctrlStruct.Pn);
nx = dimension(PA);
dimP = nx;

mergetitles = 1;
if ctrlStruct.probStruct.tracking,
    if isfield(ctrlStruct.sysStruct,'dims'),
        % use stored dimensions if available
        nu = ctrlStruct.sysStruct.dims.nu;
        nx = ctrlStruct.sysStruct.dims.nx;
    else
        if iscell(ctrlStruct.sysStruct.B),
            nu = size(ctrlStruct.sysStruct.B{1},2);
            nx = (size(ctrlStruct.sysStruct.A{1},2)-nu)/2;
        else
            nu = size(ctrlStruct.sysStruct.B,2);
            nx = (size(ctrlStruct.sysStruct.A,2)-nu)/2;
        end
    end
    % tracking partition is in higher dimension, we just plot the section
    % through true system states
    Options.xsection = ((min(nx, 3)+1):dimP);
    Options.verbose = 0;   % to avoid 'section is empty' warnings
    mergetitles = 0;  % do not add 'section through x3=... x4=...' to the figure title
end
if isfield(ctrlStruct.sysStruct,'dumode')
    % if this flag is set, solution has been computed for extended state-space
    % to guarantee fullfilment of deltaU constraints in closed-loop
    if iscell(ctrlStruct.sysStruct.B),
        nu = size(ctrlStruct.sysStruct.B{1}, 2);
        nx = size(ctrlStruct.sysStruct.A{1}, 2) - nu;
    else
        nu = size(ctrlStruct.sysStruct.B, 2);
        nx = size(ctrlStruct.sysStruct.A, 2) - nu;
    end
    Options.xsection = nx+1:nx+nu;
    Options.verbose = 0;
    mergetitles = 0;
end

if isfield(Options,'color'),
    [handle,titlehandle]=plot(PA,Options);
elseif isempty(details),  
    % plot for constrained Finite-time optimal control problem
    [handle,titlehandle]=plot(PA,Options);
elseif isfield(ctrlStruct,'simplified') & ctrlStruct.simplified == 1
    % plot for simplified solutions (mpt_simplify)
    [handle,titlehandle]=plot(PA,Options);
else % for Infinite-time and iterative solution
    if isa(details,'double') & ~Options.noInfCol,
        regionhorizon=fliplr(details);
        auxcolors=hsv;           % take the color map
        maxcol = length(PA)+1;
        for i=1:maxcol,      % prepare enough colors
            jj=mod(i*2,size(auxcolors,1))+1;
            colors(i,:)=auxcolors(jj,:);
        end 
        Options.newfigure=0;     % do not open a new figure  (because a call to hsv opens the windows automatically
        infcolors=[];
        for i=1:length(PA)
            color=colors(regionhorizon(i)+1,:);
            infcolors=[infcolors; color];
        end
        Options.color=infcolors;
        [handle,titlehandle]=plot(PA,Options);
    else
        [handle,titlehandle]=plot(PA,Options);
    end
end

% set figure properties:
if ~isempty(titlehandle) & mergetitles
    oldtitle=get(titlehandle,'String');
else
    oldtitle='';
end
if ctrlStruct.probStruct.tracking,
    oldtitle = '(tracking)';
elseif isfield(ctrlStruct.sysStruct,'dumode'),
    oldtitle = '(deltaU mode)';
end
title(['Controller partition with ' num2str(length(PA)) ' regions. ' oldtitle],'FontSize',14);

if isfield(ctrlStruct.sysStruct,'StateName'),
    xlabel(ctrlStruct.sysStruct.StateName{1},'Fontsize',14); % LaTeX math symbols are directly supported!
    ylabel(ctrlStruct.sysStruct.StateName{2},'Fontsize',14);
else
    xlabel('x_1','Fontsize',14); % LaTeX math symbols are directly supported!
    ylabel('x_2','Fontsize',14);
end
%if dimension(PA(1))>=3,
if nx>=3,
    if isfield(ctrlStruct.sysStruct,'StateName'),
        zlabel(ctrlStruct.sysStruct.StateName{3},'FontSize',14);
    else
        zlabel('x_3','Fontsize',14);
    end
end
h=gcf;
h1 = get(h,'CurrentAxes');
set(h1,'Fontname','times');
set(h1,'Fontsize',14);

grid on;
axis tight

if nargout == 0;
    clear('handle');
end
