function [tmap,Pn,ex,explus] = mpt_transmap(Pn, Acell, fcell, Options)
%MPT_TRANSMAP Computes transition map
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Prunes infeasible transitions. Specifically:
%   domain(P2(j), A{i}, f{i}, P1(i)) is NOT feasible if tmap(i, j) is 0
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% Pn          - polytope array
% A, f        - cells containing affine map: x(k+1) = A{i}*x + f{i}
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% tmap        - sparse matrix with non-zero elements on indexes which correspond
%               to transitions whiche are POSSIBLY feasible.
% Pn          - input polyarray P1 with extreme points stored inside for each
%               polytope
% ex          - extreme points of Pn as a cell array
% explus      - extreme points of the affine transformation A*x+f
%

% $Id: mpt_transmap.m,v 1.2 2005/06/28 10:46:04 kvasnica Exp $
%
% (C) 2005 Johan Loefberg, Automatic Control Laboratory, ETH Zurich,
%          joloef@control.ee.ethz.ch
% (C) 2005 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%          kvasnica@control.ee.ethz.ch

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

global mptOptions
if ~isstruct(mptOptions),
    mpt_error;
end
if ~isfield(Options, 'lpsolver'),
    Options.lpsolver = mptOptions.lpsolver;
end
if ~isfield(Options, 'abs_tol'),
    Options.abs_tol = mptOptions.abs_tol;
end

lpsolver = Options.lpsolver;
abs_tol = Options.abs_tol;

lenP = length(Pn);
tmap = ones(lenP,lenP);

% Vertex enumeration
for is=1:lenP
    [E, R, Pn(is)] = extreme(Pn(is));
    ex{is} = E';
    
    % round almost-zero elements
    ex{is}(find(abs(ex{is}) < 1e-12)) = 0;
    explus{is} = Acell{is}*ex{is} + repmat(fcell{is},1,size(ex{is},2));
    
    % round almost-zero elements
    explus{is}(find(abs(explus{is}) < 1e-12)) = 0;
end

n = dimension(Pn);

for i=1:lenP

    BoxMin{i} = min(ex{i}')';
    BoxMax{i} = max(ex{i}')';
    
    %Pn(i) = set(Pn(i),'bbox',[BoxMin{i} BoxMax{i}]);

    %now extract all other extreme points of the bounding box
    for j=1:2^n
        index=dec2bin(j-1,n);
        for k=1:n
            if(index(k)=='1')
                boxPoint{i}{j}(k)=BoxMax{i}(k);
            else
                boxPoint{i}{j}(k)=BoxMin{i}(k);
            end
        end%for k=1:n
        if(size(boxPoint{i}{j},1)<size(boxPoint{i}{j},2))
            boxPoint{i}{j}= boxPoint{i}{j}';
        end
    end%for j=1:2^(n)
end%Pn


% Initial prune, based on coordinate cuts
% Typically removes 10%
for r = 1:n
    a = eyev(n,r);
    b = 0;
    for prunei = 1:lenP
        iprunes(prunei) = all((a'*explus{prunei} < -1e3*abs_tol));
    end
    for prunej = 1:lenP
        jprunes(prunej) = all((a'*ex{prunej} > 1e3*abs_tol));
    end
    for prunei = 1:lenP
        for prunej = 1:lenP
            if iprunes(prunei)
                if jprunes(prunej)
                    tmap(prunei,prunej) = 0;
                end
            end
        end
    end
end

for i = 1:lenP
    isfulldimP(i) = isfulldim(Pn(i));   
end

vecex = [ex{1:end}];
vecexplus = [explus{1:end}];
indiciesex = cumsum([1 cellfun('prodofsize',ex)/n]);
indiciesexplus = cumsum([1 cellfun('prodofsize',explus)/n]);
lpsolver = Options.lpsolver;

%neighbour = ones(lenP,lenP);

for i=1:lenP

    %check if polytope is mapped onto a full dimensional polytope or onto
    %a facet (this has an impact on the subsequent reachability analysis)
    if(all(abs(eig(Acell{i}))>Options.abs_tol))
        fullMap=1;
    else
        fullMap=0;
    end

    if fullMap,
        lowerCur = min((Acell{i} * [boxPoint{i}{:}] + repmat(fcell{i},1,length(boxPoint{i})))')';
        upperCur = max((Acell{i} * [boxPoint{i}{:}] + repmat(fcell{i},1,length(boxPoint{i})))')';
    end

    for j = find(tmap(i,:))

        if tmap(i,j)
            if (fullMap)
                if any((upperCur < BoxMin{j}) | (lowerCur>BoxMax{j}))
                    tmap(i,j) = 0;
                end
            else
            end
        end

        if tmap(i,j)

            [a,b,f] = separatinghp(ex{j},explus{i},lpsolver,n);

            if (f == 1) % Found one!
                AA = a'*vecexplus;
                BB = a'*vecex;
                iprunes  = zeros(lenP,1);
                for prunei = i:lenP
                    iprunes(prunei) = all((AA(indiciesexplus(prunei):indiciesexplus(prunei+1)-1)-b)>=0);
                end
                jprunes  = zeros(lenP,1);
                for prunej = 1:lenP
                    jprunes(prunej) = all((BB(indiciesex(prunej):indiciesex(prunej+1)-1)-b)<=0);
                end
                jprunes = logical(jprunes);
                if any(jprunes)
                    for prunei = find(iprunes)
                        tmap(prunei,jprunes) = 0;
                    end
                end
            end

        end
    end
end

%-------------------------------------------------------------------
function [a,b,how] = separatinghp(X,Y,lpsolver,nx)

mx = size(X,2);
my = size(Y,2);

A = [X' -ones(mx,1);-Y' ones(my,1)];
b = [-ones(mx,1);-ones(my,1)];
f = zeros(1,nx+1);
[xopt,fval,lambda,exitflag,how]=mpt_solveLPi(f,A,b,[],[],[],lpsolver,0);
a = xopt(1:nx);
b = xopt(end);
how = ~all(abs(xopt<1e-8)) & ~any(xopt==1e9);
