function masterblock=pwa_def;
%PWA_DEF Definition of various forms of PWA examples
%
% used by mpt_runTests
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% defines various tests cases based on the different PWA systems.
% Creates 'blocks' for each of the tested functions:
%  * mpt_iterativePWA     - minimum time solution
%  * mpt_reducedSwitchPWA - reduced-switch solution
%  * mpt_oneStepCtrlPWA   - one-step solution
%  * mpt_optControlPWA    - CFTOC
%  * mpt_optInfControlPWA - CITOC
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
%                        
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
%

% Copyright is with the following author(s):
%
%(C) 2004 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%              kvasnica@control.ee.ethz.ch

% ---------------------------------------------------------------------------
% Legal note:
%     This file is free software; you can redistribute it and/or modify it
%     under the terms of the GNU General Public License as published by
%     the Free Software Foundation; either version 2, or (at your option)
%     any later version.
%
%     This file is distributed in the hope that it will be useful, but WITHOUT
%     ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
%     or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public
%     License for more details.
%
%     You should have received a copy of the GNU General Public License
%     along with this file; see the file COPYING. If not, write to the Free
%     Software Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
%     02111-1307, USA. 
%
% ---------------------------------------------------------------------------

masterblock = {};

%-----------------------------------------------------------------
func = 'mpt_iterativePWA';
data = {};
options = {};

data{end+1} = 'pwa_sincos';
options{end+1} = 'probStruct.norm = 2;';

data{end+1} = 'pwa_sincos';
options{end+1} = 'probStruct.norm = 1;';

data{end+1} = 'pwa_sincos';
options{end+1} = 'probStruct.norm = Inf;';

data{end+1} = 'pwa2d';
options{end+1} = 'probStruct.norm = 2;';

data{end+1} = 'pwa2d';
options{end+1} = 'probStruct.norm = 1;';

data{end+1} = 'pwa2d';
options{end+1} = 'probStruct.norm = Inf;';

data{end+1} = 'pwa_DI';
options{end+1} = 'probStruct.norm = 2;';

data{end+1} = 'pwa_DI';
options{end+1} = 'probStruct.norm = 1;';

data{end+1} = 'pwa_DI';
options{end+1} = 'probStruct.norm = Inf;';

% data{end+1} = 'pwa_car';
% options{end+1} = 'probStruct.norm = 2;';
% 
% data{end+1} = 'pwa_car';
% options{end+1} = 'probStruct.norm = 1;';
% 
% data{end+1} = 'pwa_car';
% options{end+1} = 'probStruct.norm = Inf;';

% data{end+1} = 'pwa3d';
% options{end+1} = 'probStruct.norm = 2;';
% 
% data{end+1} = 'pwa3d';
% options{end+1} = 'probStruct.norm = 1;';
% 
% data{end+1} = 'pwa3d';
% options{end+1} = 'probStruct.norm = Inf;';

mblock.func = func;
mblock.data = data;
mblock.options = options;
masterblock{end+1} = mblock;
%-----------------------------------------------------------------


%-----------------------------------------------------------------
func = 'mpt_reducedSwitchPWA';
data = {};
options = {};

data{end+1} = 'pwa_sincos';
options{end+1} = 'probStruct.norm = 2; Options.iterative=1;';

data{end+1} = 'pwa_sincos';
options{end+1} = 'probStruct.norm = 1; Options.iterative=1;';

data{end+1} = 'pwa_sincos';
options{end+1} = 'probStruct.norm = Inf; Options.iterative=1;';

data{end+1} = 'pwa2d';
options{end+1} = 'probStruct.norm = 2; Options.iterative=1;';

data{end+1} = 'pwa2d';
options{end+1} = 'probStruct.norm = 1; Options.iterative=1;';

data{end+1} = 'pwa2d';
options{end+1} = 'probStruct.norm = Inf; Options.iterative=1;';

data{end+1} = 'pwa_DI';
options{end+1} = 'probStruct.norm = 2; Options.iterative=1;';

data{end+1} = 'pwa_DI';
options{end+1} = 'probStruct.norm = 1; Options.iterative=1;';

data{end+1} = 'pwa_DI';
options{end+1} = 'probStruct.norm = Inf; Options.iterative=1;';

% data{end+1} = 'pwa_car';
% options{end+1} = 'probStruct.norm = 2; Options.iterative=1;';
% 
% data{end+1} = 'pwa_car';
% options{end+1} = 'probStruct.norm = 1; Options.iterative=1;';
% 
% data{end+1} = 'pwa_car';
% options{end+1} = 'probStruct.norm = Inf; Options.iterative=1;';

% data{end+1} = 'pwa3d';
% options{end+1} = 'probStruct.norm = 2; Options.iterative=1;';
% 
% data{end+1} = 'pwa3d';
% options{end+1} = 'probStruct.norm = 1; Options.iterative=1;';
% 
% data{end+1} = 'pwa3d';
% options{end+1} = 'probStruct.norm = Inf; Options.iterative=1;';

mblock.func = func;
mblock.data = data;
mblock.options = options;
masterblock{end+1} = mblock;
%-----------------------------------------------------------------


% %-----------------------------------------------------------------
% func = 'mpt_iterativePWA_addU';
% data = {};
% options = {};
% 
% data{end+1} = 'pwa_sincos_addU';
% options{end+1} = 'probStruct.norm = 2;';
% 
% data{end+1} = 'pwa_sincos_addU';
% options{end+1} = 'probStruct.norm = 1;';
% 
% data{end+1} = 'pwa_sincos_addU';
% options{end+1} = 'probStruct.norm = Inf;';
% 
% data{end+1} = 'pwa2d_addU';
% options{end+1} = 'probStruct.norm = 2;';
% 
% data{end+1} = 'pwa2d_addU';
% options{end+1} = 'probStruct.norm = 1;';
% 
% data{end+1} = 'pwa2d_addU';
% options{end+1} = 'probStruct.norm = Inf;';
% 
% data{end+1} = 'pwa_DI_addU';
% options{end+1} = 'probStruct.norm = 2;';
% 
% data{end+1} = 'pwa_DI_addU';
% options{end+1} = 'probStruct.norm = 1;';
% 
% data{end+1} = 'pwa_DI_addU';
% options{end+1} = 'probStruct.norm = Inf;';
% 
% mblock.func = func;
% mblock.data = data;
% mblock.options = options;
% masterblock{end+1} = mblock;
% %-----------------------------------------------------------------


%-----------------------------------------------------------------
func = 'mpt_oneStepCtrlPWA';
data = {};
options = {};

data{end+1} = 'pwa_sincos';
options{end+1} = 'probStruct.norm = 1; probStruct.subopt_lev=2; Options.lpsolver=0; probStruct.N=1;';

data{end+1} = 'pwa_sincos';
options{end+1} = 'probStruct.norm = Inf; probStruct.subopt_lev=2; Options.lpsolver=0; probStruct.N=1;';

data{end+1} = 'pwa2d';
options{end+1} = 'probStruct.norm = 1; probStruct.subopt_lev=2; Options.lpsolver=0; probStruct.N=1;';

data{end+1} = 'pwa2d';
options{end+1} = 'probStruct.norm = Inf; probStruct.subopt_lev=2; Options.lpsolver=0; probStruct.N=1;';

data{end+1} = 'pwa_DI';
options{end+1} = 'probStruct.norm = 1; probStruct.subopt_lev=2; Options.lpsolver=0; probStruct.N=1;';

data{end+1} = 'pwa_DI';
options{end+1} = 'probStruct.norm = Inf; probStruct.subopt_lev=2; Options.lpsolver=0; probStruct.N=1;';

% data{end+1} = 'pwa_car';
% options{end+1} = 'probStruct.norm = 1; probStruct.subopt_lev=2; Options.lpsolver=0; probStruct.N=1;';
% 
% data{end+1} = 'pwa_car';
% options{end+1} = 'probStruct.norm = Inf; probStruct.subopt_lev=2; Options.lpsolver=0; probStruct.N=1;';

% data{end+1} = 'pwa3d';
% options{end+1} = 'probStruct.norm = 1; probStruct.subopt_lev=2; Options.lpsolver=0; probStruct.N=1;';
% 
% data{end+1} = 'pwa3d';
% options{end+1} = 'probStruct.norm = Inf; probStruct.subopt_lev=2; Options.lpsolver=0; probStruct.N=1;';


mblock.func = func;
mblock.data = data;
mblock.options = options;
masterblock{end+1} = mblock;
%-----------------------------------------------------------------


%-----------------------------------------------------------------
func = 'mpt_optControlPWA';
data = {};
options = {};

data{end+1} = 'pwa_sincos';
options{end+1} = 'probStruct.norm = 1; probStruct.subopt_lev=0; probStruct.N = 5;';

data{end+1} = 'pwa_sincos';
options{end+1} = 'probStruct.norm = Inf; probStruct.subopt_lev=0; probStruct.N = 5;';

data{end+1} = 'pwa_sincos';
options{end+1} = 'probStruct.norm = 1; probStruct.subopt_lev=0; probStruct.N = 11;';

data{end+1} = 'pwa_sincos';
options{end+1} = 'probStruct.norm = Inf; probStruct.subopt_lev=0; probStruct.N = 11;';

data{end+1} = 'pwa2d';
options{end+1} = 'probStruct.norm = 1; probStruct.subopt_lev=0; probStruct.N = 5;';

data{end+1} = 'pwa2d';
options{end+1} = 'probStruct.norm = Inf; probStruct.subopt_lev=0; probStruct.N = 5;';

data{end+1} = 'pwa_DI';
options{end+1} = 'probStruct.norm = 1; probStruct.subopt_lev=0; probStruct.N = 5;';

data{end+1} = 'pwa_DI';
options{end+1} = 'probStruct.norm = Inf; probStruct.subopt_lev=0; probStruct.N = 5;';

data{end+1} = 'pwa_car';
options{end+1} = 'probStruct.norm = 1; probStruct.subopt_lev=0; probStruct.N = 2;';

data{end+1} = 'pwa_car';
options{end+1} = 'probStruct.norm = Inf; probStruct.subopt_lev=0; probStruct.N = 2;';

% data{end+1} = 'pwa3d';
% options{end+1} = 'probStruct.norm = 1; probStruct.subopt_lev=0; probStruct.N = 3;';
% 
% data{end+1} = 'pwa3d';
% options{end+1} = 'probStruct.norm = Inf; probStruct.subopt_lev=0; probStruct.N = 3;';


mblock.func = func;
mblock.data = data;
mblock.options = options;
masterblock{end+1} = mblock;
%-----------------------------------------------------------------


%-----------------------------------------------------------------
func = 'mpt_optInfControlPWA';
data = {};
options = {};

data{end+1} = 'pwa_sincos';
options{end+1} = 'probStruct.norm = 1; probStruct.subopt_lev=0; probStruct.N = Inf;';

data{end+1} = 'pwa_sincos';
options{end+1} = 'probStruct.norm = Inf; probStruct.subopt_lev=0; probStruct.N = Inf;';

% data{end+1} = 'pwa2d';
% options{end+1} = 'probStruct.norm = 1; probStruct.subopt_lev=0; probStruct.N = Inf;';
% 
% data{end+1} = 'pwa2d';
% options{end+1} = 'probStruct.norm = Inf; probStruct.subopt_lev=0; probStruct.N = Inf;';

data{end+1} = 'pwa_DI';
options{end+1} = 'probStruct.norm = 1; probStruct.subopt_lev=0; probStruct.N = Inf;';

data{end+1} = 'pwa_DI';
options{end+1} = 'probStruct.norm = Inf; probStruct.subopt_lev=0; probStruct.N = Inf;';

mblock.func = func;
mblock.data = data;
mblock.options = options;
masterblock{end+1} = mblock;
%-----------------------------------------------------------------


return

