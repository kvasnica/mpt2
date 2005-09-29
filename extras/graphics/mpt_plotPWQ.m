function mpt_plotPWQ(Pn,lyapunovQ,lyapunovL,lyapunovC,meshgridpoints,Options);
%MPT_PLOTPWQ Plots a PWQ function defined over polyhedral partition
%
% mpt_plotPWQ(Pn,Q,L,C,meshgridpoints,Options);
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Plots a PWQ function defined over a given polyhedral partition.
%
% PWQ = x'Qx + Lx + C.
%
% This function could be used either to plot PWQ Lyapunov function
% obtained by mpt_getPWQLyapFct or to plot the piece-wise quadratic
% cost function of an explicit solution
%
% USAGE:
%   mpt_plotPWQ(Pn,Ai,Bi,Ci)           to plot PWQ function over Pn
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% Pn                - Polyhedral partition given by the polytope array Pn
% Q,L,C             - Cells containing parameters of the PWQ function x'Qx+x'L+C
% meshgridpoints    - number of grid points in one axis,
%                     (default: 30)
% Options.shade     - Level of transparency (0 = fully transparent, 1 = solid)
% Options.edgecolor - specifies the color of edges. Default: 'k'.
% Options.lpsolver  - Solver for LPs when (and if) computing bounding box of Pn,
%                     (default: mptOptions.lpsolver)
% Options.newfigure - If set to 1, opens a new figure window,
%                     (default: mptOptions.newfigure)
% Options.overlaps  - If regions in partition Pn are overlaping this flag should
%                     be set to 1, in order to find the smallest quadratic value
%                     (default: 0)
% Options.showPn    - If set to 1, plots on polyhedral sets Pn,
%                     (default: 1)
% Options.samecolors - If set to 1, pieces of PWA function will be plotted
%                      in the same colors as the partition below
%                      (default: 0)
% Options.min_x1    - Rectangular space where PWQ function is computed
%                     (default: bounding box on Pn)
% Options.max_x1    - Rectangular space where PWQ function is computed
%                     (default: bounding box on Pn)
% Options.min_x2    - Rectangular space where PWQ function is computed
%                     (default: bounding box on Pn)
% Options.max_x2    - Rectangular space where PWQ function is computed
%                     (default: bounding box on Pn)
%
% Note: If Options is missing or some of the fields are not defined, the default
%       values from mptOptions will be used
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% none
%
% see also MPT_PLOTPWA, MPT_GETPWQLYAPFCT, MPT_GETCOMMONLYAPFCT

% Copyright is with the following author(s):
%
% (c) 2005 Frank J. Christophersen, Automatic Control Laboratory, ETH Zurich,
%          fjc@control.ee.ethz.ch
% (c) 2003 Mato Baotic, Automatic Control Laboratory, ETH Zurich,
%          baotic@control.ee.ethz.ch
% (c) 2003 Pascal Grieder, Automatic Control Laboratory, ETH Zurich,
%          grieder@control.ee.ethz.ch
% (c) 2003 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%          kvasnica@control.ee.ethz.ch
% (c) 2003 Marco Luethi, Automatic Control Laboratory, ETH Zurich,
%          mluethi@ee.ethz.ch

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

error(nargchk(4,6,nargin));

global mptOptions;

if ~isstruct(mptOptions),
    mpt_error;
end

if(nargin<5 | isempty(meshgridpoints))
    meshgridpoints=30;
end

if isstruct(meshgridpoints) & nargin < 6,
    Options = meshgridpoints;
    meshgridpoints = 30;
elseif nargin<6
    Options=[];
end

if ~isfield(Options,'lpsolver'),
    Options.lpsolver=mptOptions.lpsolver;
end
if ~isfield(Options,'newfigure')
    Options.newfigure=mptOptions.newfigure;
end
if ~isfield(Options,'overlaps')
    Options.overlaps=0;
end
if ~isfield(Options,'showPn')
    Options.showPn=1;
end
if ~isfield(Options,'samecolors')
    Options.samecolors = 0;
end

if dimension(Pn)>2,
    error('mpt_plotPWQ: only 2D partitions supported!');
end

oneDimCase = (dimension(Pn)==1);

pwq_or_q=1;

if(isempty(lyapunovL) | isempty(lyapunovC))
     pwq_or_q=0;
     if(~iscell(lyapunovQ))
         Qiscell=0;
     else
         Qiscell=1;
     end
     tQ=lyapunovQ;
       
     clear lyapunovQ
     for i=1:length(Pn)
         if(Qiscell)
             lyapunovQ=tQ;
         else
             lyapunovQ{i}=tQ;    
         end
         lyapunovL{i}=[];
         lyapunovC{i}=[];
     end
end

disp('Creating PWQ plot...');

if Options.samecolors,
    maxlen = length(Pn);
    auxcolors=hsv(maxlen);
    multiplier=7;
    if mod(size(auxcolors,1),multiplier)==0,
        multiplier=multiplier+1;
    end
    for i=1:maxlen,
        jj=mod(i*multiplier,size(auxcolors,1))+1; % prepare enough colors for all polytopes, cycle through a given color map
        colors(i,:)=auxcolors(jj,:);
    end
end

% Use predefined range for x, or find the extremal values of Pn

if oneDimCase,
    V = [];
    for ii=1:length(Pn),
        V = [V; extreme(Pn(ii))];
    end
    min_x1 = min(V);
    max_x1 = max(V);
    %   produce a meshgrid between the extremal values
    x1=linspace(min_x1,max_x1,meshgridpoints);
    x2=linspace(0,0,meshgridpoints);
    [X,Y]=meshgrid(x1,x2);
    Z=repmat(Inf,meshgridpoints,meshgridpoints); %if meshgridpoint is in not in any region, set function value to Inf
    %(no plot for this point)
    C=zeros(meshgridpoints,meshgridpoints);
else
    V = [];
    if isfield(Options,'min_x1')
        min_x1=Options.min_x1;
    else
        min_x1=Inf;
        for i=1:length(Pn)
            f_min_x1=[1,0];
            [H,K]=double(Pn(i));
            [x,fval,lambda,exitflag,how]=mpt_solveLPi(f_min_x1,H,K,[],[],[],Options.lpsolver);
            if(x(1) < min_x1)
                min_x1=x(1);
            end
        end
    end
    if isfield(Options,'max_x1')
        max_x1=Options.max_x1;
    else
        max_x1=-Inf;
        for i=1:length(Pn)
            f_max_x1=[-1,0];
            [H,K]=double(Pn(i));
            [x,fval,lambda,exitflag,how]=mpt_solveLPi(f_max_x1,H,K,[],[],[],Options.lpsolver);
            if(x(1) > max_x1)
                max_x1=x(1);
            end
        end
    end
    if isfield(Options,'min_x2')
        min_x2=Options.min_x2;
    else
        min_x2=Inf;
        for i=1:length(Pn)
            f_min_x2=[0,1];
            [H,K]=double(Pn(i));
            [x,fval,lambda,exitflag,how]=mpt_solveLPi(f_min_x2,H,K,[],[],[],Options.lpsolver);
            if(x(2) < min_x2)
                min_x2=x(2);
            end
        end
    end
    if isfield(Options,'max_x2')
        max_x2=Options.max_x2;
    else
        max_x2=-Inf;
        for i=1:length(Pn)
            f_max_x2=[0,-1];
            [H,K]=double(Pn(i));
            [x,fval,lambda,exitflag,how]=mpt_solveLPi(f_max_x2,H,K,[],[],[],Options.lpsolver);
            if(x(2) > max_x2)
                max_x2=x(2);
            end
        end
    end
    %   produce a meshgrid between the extremal values
    x1=linspace(min_x1,max_x1,meshgridpoints);
    x2=linspace(min_x2,max_x2,meshgridpoints);
    [X,Y]=meshgrid(x1,x2);
    Z=repmat(Inf,meshgridpoints,meshgridpoints); %if meshgridpoint is in not in any region, set function value to Inf
    %(no plot for this point)
    C=zeros(meshgridpoints,meshgridpoints);
end




% check if meshgridpoint is in a region and if it is so, calculate the function value
I_2=eye(2);
f_opt=[1,1];    
for i=1:meshgridpoints
    for j=1:meshgridpoints
        k=1;
        region_found=0;
        while ((k <= length(Pn)) & ((region_found == 0) | Options.overlaps))
            if oneDimCase
                Position = x1(j);
            else
                Position=[x1(j);x2(i)];
            end
            m=1;
            x_in_region=1;
            [Hk,Kk]=double(Pn(k));
            while((m <= length(Hk)) & (x_in_region==1))  % check if position achieves all (m) inequalities of region k
                x_value=Hk(m,:)*Position;
                if(x_value > Kk(m))
                    x_in_region=0;
                end
                m=m+1;    
            end
            if(x_in_region==1)
                x=Position;
                if((pwq_or_q==1) & (~isempty(lyapunovL{k})) & (~isempty(lyapunovC{k})))
                    lL = lyapunovL{k};
                    zaux=x'*lyapunovQ{k}*x+(lL(:))'*x+lyapunovC{k}; % calculate Lyapunov-function value
                else
                    zaux=x'*lyapunovQ{k}*x; % calculate quadratic-function value
                end
                if zaux<Z(i,j)
                    Z(i,j)=zaux;
                    C(i,j)=k;   % C is used for color (surf-plot)
                end
                region_found=1;
            end
            k=k+1;
        end
    end
end

% plot feasible set: can not use ifa_region_plot,
% because then it is not possible that region and corresponding part of PWQ function have same color
legLab = [];  % legend labels
handles = []; % legend handles
xlabel('x_1','Fontsize',16); % LaTeX math symbols are directly supported!
ylabel('x_2','Fontsize',16);
h=gcf;
h1 = get(h,'CurrentAxes');
set(h1,'Fontname','times');
set(h1,'Fontsize',14);
title(sprintf('The PWQ function over %d regions', length(Pn)),'FontSize',18);
grid; 
%end of plot invariant-regions

if oneDimCase,
    hold on
    plot(X(1,:), Z(1,:), 'LineWidth', 3);
    ylabel('f_{PWQ}','Fontsize',16);
else
    h = surf(X,Y,Z,C);
    if isfield(Options, 'shade'),
        set(h, 'FaceAlpha', Options.shade);
    else
        set(gcf, 'Renderer', 'painters');
    end
    if isfield(Options, 'edgecolor')
        set(h, 'EdgeColor', Options.edgecolor);
    end
end
if Options.showPn
    hold on
    if Options.samecolors,
        handle=plot(Pn,struct('color',hsv(length(Pn))));
    else
        handle=plot(Pn);
    end
end

if Options.samecolors,
    colormap(hsv);
end
if ~oneDimCase,
    view(-37.5,20) 
end
hold off;
grid on
drawnow;
