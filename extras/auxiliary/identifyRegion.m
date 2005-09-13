function identfyRegion(P)
%IDENTIFYREGION plots the number of the polytopes into the current figure
%
% identifyregion(P)
%
% -------------------------------------------------------------------------
% INPUT
% -------------------------------------------------------------------------
% P                 - polytope array
%
% see also POLYTOPE/PLOT
%

% Copyright is with the following author(s):
%
% (C) 2004 Frank J. Christophersen, Automatic Control Lab., ETH Zurich,
%          fjc@control.ee.ethz.ch

% -------------------------------------------------------------------------
% Legal note:
%          This program is free software; you can redistribute it and/or
%          modify it under the terms of the GNU General Public
%          License as published by the Free Software Foundation; either
%          version 2.1 of the License, or (at your option) any later 
%          version.
%
%          This program is distributed in the hope that it will be useful,
%          but WITHOUT ANY WARRANTY; without even the implied warranty of
%          MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
%          GNU General Public License for more details.
% 
%          You should have received a copy of the GNU General Public
%          License along with this library; if not, write to the 
%          Free Software Foundation, Inc., 
%          59 Temple Place, Suite 330, 
%          Boston, MA  02111-1307  USA
%
% -------------------------------------------------------------------------


global mptOptions;

if ~isa(P, 'polytope'),
    error('IDENTIFYREGION: input argument MUST be a polytope');
end

hold on;
h = gcf;

n = dimension(P);

for ii=1:length(P)
  [xc, Rc] = chebyball(P(ii));
  
  if n==2
    h2 = text(xc(1), xc(2), num2str(ii));
  elseif n==3
    h2 = text(xc(1), xc(2), xc(3), num2str(ii));
  else
    error('IDENTIFYREGION: only for 2D and 3D polytopes');
  end
  
  set(h2,'Color','w');
  set(h2,'FontSize',7);

end%ii

hold off;
