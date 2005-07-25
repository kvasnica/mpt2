function masterblock=lti2d_def;
%LTI2D_DEF Definition of various forms of a Double Integrator example
%
% used by mpt_runTests
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% defines various tests cases based on the Double Integrator dynamics.
% Creates 'blocks' for each of the tested functions:
%  * mpt_optControl    - CFTOC with various norms and uncertainties
%  * mpt_optInfControl - CITOC
%  * mpt_iterative     - minimum time solution
%  * mpt_oneStepCtrl   - low complexity solution
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
func = 'mpt_optControl';
data = {};
options = {};

data{end+1} = 'Double_Integrator';
options{end+1} = 'probStruct.norm = 2; probStruct.N=5;';

data{end+1} = 'Double_Integrator';
options{end+1} = 'probStruct.norm = 1; probStruct.N=5;';

data{end+1} = 'Double_Integrator';
options{end+1} = 'probStruct.norm = Inf; probStruct.N=5;';

data{end+1} = 'Double_Integrator_addU';
options{end+1} = 'probStruct.norm = 2; probStruct.N = 6;';

data{end+1} = 'Double_Integrator_addU';
options{end+1} = 'probStruct.norm = 1; probStruct.N = 6;';

data{end+1} = 'Double_Integrator_addU';
options{end+1} = 'probStruct.norm = Inf; probStruct.N = 6;';

data{end+1} = 'Double_Integrator_parU';
options{end+1} = 'probStruct.norm = 2; probStruct.N = 6;';

data{end+1} = 'Double_Integrator_parU';
options{end+1} = 'probStruct.norm = 1; probStruct.N = 6;';

data{end+1} = 'Double_Integrator_parU';
options{end+1} = 'probStruct.norm = Inf; probStruct.N = 6;';

mblock.func = func;
mblock.data = data;
mblock.options = options;
masterblock{end+1} = mblock;
%-----------------------------------------------------------------

%-----------------------------------------------------------------
func = 'mpt_optInfControl';
data = {};
options = {};
data{end+1} = 'Double_Integrator';
options{end+1} = 'probStruct.norm = 2; probStruct.N=Inf;';

data{end+1} = 'Double_Integrator_addU';
options{end+1} = 'probStruct.norm = 2; probStruct.N=Inf;';

data{end+1} = 'Double_Integrator_parU';
options{end+1} = 'probStruct.norm = 2; probStruct.N=Inf;';


% this is VERY slows, but works
% data{end+1} = 'Double_Integrator';
% options{end+1} = 'probStruct.norm = 1; probStruct.N=Inf;';
% 
% data{end+1} = 'Double_Integrator';
% options{end+1} = 'probStruct.norm = Inf; probStruct.N=Inf;';

mblock.func = func;
mblock.data = data;
mblock.options = options;
masterblock{end+1} = mblock;
%-----------------------------------------------------------------

%-----------------------------------------------------------------
func = 'mpt_iterative';
data = {};
options = {};
data{end+1} = 'Double_Integrator';
options{end+1} = 'probStruct.norm = 2; probStruct.N=Inf; probStruct.subopt_lev=1;';

data{end+1} = 'Double_Integrator';
options{end+1} = 'probStruct.norm = 1; probStruct.N=Inf; probStruct.subopt_lev=1;';

data{end+1} = 'Double_Integrator';
options{end+1} = 'probStruct.norm = Inf; probStruct.N=Inf; probStruct.subopt_lev=1;';

data{end+1} = 'Double_Integrator_addU';
options{end+1} = 'probStruct.norm = 2; probStruct.N=Inf; probStruct.subopt_lev=1;';

data{end+1} = 'Double_Integrator_addU';
options{end+1} = 'probStruct.norm = 1; probStruct.N=Inf; probStruct.subopt_lev=1;';

data{end+1} = 'Double_Integrator_addU';
options{end+1} = 'probStruct.norm = Inf; probStruct.N=Inf; probStruct.subopt_lev=1;';

% data{end+1} = 'Double_Integrator_parU';
% options{end+1} = 'probStruct.norm = 2; probStruct.N=Inf; probStruct.subopt_lev=1;';
% 
% data{end+1} = 'Double_Integrator_parU';
% options{end+1} = 'probStruct.norm = 1; probStruct.N=Inf; probStruct.subopt_lev=1;';
% 
% data{end+1} = 'Double_Integrator_parU';
% options{end+1} = 'probStruct.norm = Inf; probStruct.N=Inf; probStruct.subopt_lev=1;';

mblock.func = func;
mblock.data = data;
mblock.options = options;
masterblock{end+1} = mblock;
%-----------------------------------------------------------------


%-----------------------------------------------------------------
func = 'mpt_oneStepCtrl';
data = {};
options = {};
data{end+1} = 'Double_Integrator';
options{end+1} = 'probStruct.norm = 2; probStruct.N=1; probStruct.subopt_lev=2; Options.lpsolver=0;';

data{end+1} = 'Double_Integrator';
options{end+1} = 'probStruct.norm = 1; probStruct.N=1; probStruct.subopt_lev=2; Options.lpsolver=0;';

data{end+1} = 'Double_Integrator';
options{end+1} = 'probStruct.norm = Inf; probStruct.N=1; probStruct.subopt_lev=2; Options.lpsolver=0;';

data{end+1} = 'Double_Integrator_addU';
options{end+1} = 'probStruct.norm = 2; probStruct.N=1; probStruct.subopt_lev=2; Options.lpsolver=0;';

data{end+1} = 'Double_Integrator_addU';
options{end+1} = 'probStruct.norm = 1; probStruct.N=1; probStruct.subopt_lev=2; Options.lpsolver=0;';

data{end+1} = 'Double_Integrator_addU';
options{end+1} = 'probStruct.norm = Inf; probStruct.N=1; probStruct.subopt_lev=2; Options.lpsolver=0;';

data{end+1} = 'Double_Integrator';
options{end+1} = 'probStruct.norm = 2; probStruct.N=3; probStruct.subopt_lev=2; Options.lpsolver=0;';

data{end+1} = 'Double_Integrator';
options{end+1} = 'probStruct.norm = 1; probStruct.N=3; probStruct.subopt_lev=2; Options.lpsolver=0;';

data{end+1} = 'Double_Integrator';
options{end+1} = 'probStruct.norm = Inf; probStruct.N=3; probStruct.subopt_lev=2; Options.lpsolver=0;';

data{end+1} = 'Double_Integrator_addU';
options{end+1} = 'probStruct.norm = 2; probStruct.N=3; probStruct.subopt_lev=2; Options.lpsolver=0;';

data{end+1} = 'Double_Integrator_addU';
options{end+1} = 'probStruct.norm = 1; probStruct.N=3; probStruct.subopt_lev=2; Options.lpsolver=0;';

data{end+1} = 'Double_Integrator_addU';
options{end+1} = 'probStruct.norm = Inf; probStruct.N=3; probStruct.subopt_lev=2; Options.lpsolver=0;';


% data{end+1} = 'Double_Integrator_parU';
% options{end+1} = 'probStruct.norm = 2; probStruct.N=1; probStruct.subopt_lev=2; Options.lpsolver=0;';
% 
% data{end+1} = 'Double_Integrator_parU';
% options{end+1} = 'probStruct.norm = 1; probStruct.N=1; probStruct.subopt_lev=2; Options.lpsolver=0;';
% 
% data{end+1} = 'Double_Integrator_parU';
% options{end+1} = 'probStruct.norm = Inf; probStruct.N=1; probStruct.subopt_lev=2; Options.lpsolver=0;';

mblock.func = func;
mblock.data = data;
mblock.options = options;
masterblock{end+1} = mblock;
%-----------------------------------------------------------------
