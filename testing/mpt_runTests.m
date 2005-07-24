function mpt_runTests(systemdeffile,whichblock,modifier,prefix)
%MPT_RUNTESTS Runs various tests of MPT functionality
%
% mpt_runTests(systemdeffile, whichblock, modifier, prefix)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% computes solution for problems defined in 'systemdeffile' and for each
% solution checks several things:
%  * whether the feasible set ctrlStruct.Pfinal is correct
%  * whether there are holes in solution
%  * whether the cost is really optimal
%  * whether stability and feasibility is guaranteed
%
% allows to specify subblocks of the testing file.
%
% USAGE:
%   run all tests as defined in file 'lti2d_def.m'
%     mpt_runTests('lti2d_def')
%
%   run only tests of mpt_optControl routine (i.e. CFTOC for LTI systems)
%     mpt_runTests('lti2d_def','mpt_optControl');
%
%   run all tests with a common modifier, i.e. output regulation
%     mpt_runTests('lti2d_def','all','probStruct.yref=[2;0]; probStruct.Qy=eye(2)');
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% systemdeffile  - name of file which defines which tests to run
% whichblock     - (optional) defines which blocks of systemdeffile to test
% modifier       - (optional) common modifier to apply
% prefix         - (optional) prefix added to the file name of output files
%                        
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% outputs text files with file names of the following form:
%
%   (prefix)_block_HHMM.log   - contains results of the tests
%   (prefix)_block_HHMM.run   - contains all messages displayed during
%                               computation
%

% $Id: mpt_runTests.m,v 1.2 2004/12/10 20:59:03 kvasnica Exp $
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

error(nargchk(1,4,nargin));

eval(sprintf('masterblocks = %s;', systemdeffile));

if nargin>1,
    if ~strcmpi(whichblock,'all'),
        blockfound = 0;
        for ii=1:length(masterblocks),
            if strcmpi(masterblocks{ii}.func,whichblock),
                M{1} = masterblocks{ii};
                masterblocks = M;
                blockfound = 1;
                break
            end
        end
        if ~blockfound,
            error(['no such block: "' whichblock '"!']);
        end
    end
end

if nargin<3,
    modifier = '';
end

gridpoints = 15;

Options = [];
dashline = '-----------------------------------------------------------------';

curtime = clock;
HHMM = sprintf('%02d%02d',curtime(4),curtime(5));
if nargin<4,
    prefix = '';
end

for im = 1:length(masterblocks),
    block = masterblocks{im};
    fname = block.func;
    if isempty(prefix),
        fname = [fname '_' HHMM];
    else
        fname = [prefix '_' fname '_' HHMM];
    end
    fid_run = openlog([fname '.run']);
    fid_err = openlog([fname '.log']);

    curtime = clock;
    writetolog(fid_run,sprintf('Computation started: %02d.%02d.%d %02d:%02d', ...
        curtime(3), curtime(2), curtime(1), curtime(4), curtime(5)));
    writetolog(fid_run,sprintf('System: %s', computer));
    writetolog(fid_run,sprintf('MPT version: %s', mpt_version));
    writetolog(fid_run,'mptOptions:');
    writetolog(fid_run,evalc('mpt_options'));
    writetolog(fid_run,['modifier: ' modifier]);
    writetolog(fid_run,dashline);
    writetolog(fid_run,dashline);
    
    writetolog(fid_err,sprintf('Computation started: %02d.%02d.%d %02d:%02d', ...
        curtime(3), curtime(2), curtime(1), curtime(4), curtime(5)));
    writetolog(fid_err,sprintf('System: %s', computer));
    writetolog(fid_err,sprintf('MPT version: %s', mpt_version));
    writetolog(fid_err,'mptOptions:');
    writetolog(fid_err,evalc('mpt_options'));
    writetolog(fid_err,['modifier: ' modifier]);
    writetolog(fid_err,dashline);
    writetolog(fid_err,dashline);
    
    for is = 1:length(block.data),
        data = block.data{is};
        options = block.options{is};
        [sysStruct, probStruct, Options] = evalsys(data, options, modifier);
        fprintf('\n');
        disp(dashline);
        writetolog(fid_run,dashline);
        writetolog(fid_err,dashline);
        disp([data '; ' options ' ' modifier]);
        T1 = [data '; ' options ' ' modifier];
        writetolog(fid_run,T1);
        writetolog(fid_err,T1);
        ctrlStruct = [];
        try
            writetolog(fid_run,'ctrlStruct=mpt_control(sysStruct,probStruct,Options);');
            T = evalc('ctrlStruct=mpt_control(sysStruct,probStruct,Options);');
            writetolog(fid_run,T);
            solutionerror = iserror(T);
            X0 = mpt_feasibleStates(ctrlStruct,gridpoints);
            disp(sprintf('Regions: %d',length(ctrlStruct.Pn)));
            disp(sprintf('Runtime: %f',ctrlStruct.details.runTime));
            writetolog(fid_err,sprintf('Regions: %d',length(ctrlStruct.Pn)));
            writetolog(fid_err,sprintf('Runtime: %f',ctrlStruct.details.runTime));
            writetolog(fid_run,T);
            if solutionerror,
                writetolog(fid_err,'         solution: errors (see log)');
            else
                writetolog(fid_err,'         solution: ok');
            end
            [anyerrors,errorlines] = iserror(T);
            if anyerrors,
                writetolog(fid_run,errorlines);
            end

            writetolog(fid_run,'checking ctrlStruct.Pfinal');
            [pfinalidentical,T] = checkPfinal(ctrlStruct);
            if pfinalidentical==2,
                writetolog(fid_err,'ctrlStruct.Pfinal not checked (PWA & subopt_lev=0 & finite N)');
                disp('ctrlStruct.Pfinal not checked (PWA & subopt_lev=0 & finite N)');
            elseif pfinalidentical==1,
                writetolog(fid_err,'ctrlStruct.Pfinal: ok');
                disp('ctrlStruct.Pfinal is correct');
            else
                writetolog(fid_err,'ctrlStruct.Pfinal: ERROR!!!'); 
                disp('ctrlStruct.Pfinal is NOT correct!');
            end
            writetolog(fid_run,T);
            
            writetolog(fid_run,'checking for holes in the solution');
            [nholes,maxR] = holesInSolution(ctrlStruct);
            if nholes>0,
                writetolog(fid_err,sprintf('            holes: ERROR: %d holes in solution, largest R: %f',nholes,maxR));
                fprintf('%d holes in solution, largest R: %f\n',nholes,maxR)
            else
                disp('no holes detected');
                writetolog(fid_err,'            holes: ok');
            end
            T = '';
            writetolog(fid_run,'checking cost');
            [costmatch, costoptimal, errtype, Trun] = testCost(ctrlStruct, X0);
            writetolog(fid_run,Trun);
            writetolog(fid_run,'checking feasibility');
            [feasible,stable,anyerrors,stopreason,T,Trun] = testStability(ctrlStruct,X0,gridpoints,1);
            writetolog(fid_run,Trun);
            
            if feasible,
                anyerrors = strvcat(anyerrors,'Feasibility: ok');
                disp('Feasibility: ok');
                writetolog(fid_err,'      Feasibility: ok');
            else
                anyerrors = strvcat(anyerrors, 'Feasibility: error');
                disp('Feasibility: error');
                writetolog(fid_err,'      Feasibility: ERROR!!!');
            end
            whichwords = '';
            if size(stopreason,1)>1,
                whichwords = 'some states';
            else
                %whichwords = 'all states';
                whichwords = '';
            end
            if stable,
                for isr=1:size(stopreason,1),
                    anyerrors = strvcat(anyerrors,['Stability: ' whichwords ' ' stopreason(isr,:)]);
                    disp(['Stability: ' whichwords ' ' stopreason(isr,:)]);
                    if ~isempty(findstr(stopreason(isr,:),'converged to origin (all)')),
                        writetolog(fid_err,'        Stability: ok');
                    else
                        writetolog(fid_err,['        Stability: ' whichwords stopreason(isr,:)]);
                    end
                end
            else
                for isr=1:size(stopreason,1),
                    anyerrors = strvcat(anyerrors,['Stability: error: ' whichwords ' ' stopreason(isr,:)]);
                    disp(['Stability: error: ' whichwords ' ' stopreason(isr,:)]);
                    writetolog(fid_err,['        Stability: ERROR: ' whichwords stopreason(isr,:)]);
                end
            end
            if costmatch,
                if iscell(ctrlStruct.sysStruct.A) & ctrlStruct.probStruct.subopt_lev>0,
                    anyerrors = strvcat(anyerrors,'Open-loop cost identical: PWA system');
                    disp('Open-loop cost identical: PWA system');
                    writetolog(fid_err,'OL cost identical: pwa system');
                elseif ctrlStruct.probStruct.subopt_lev>0,
                    anyerrors = strvcat(anyerrors,'Open-loop cost identical: subopt_lev > 0');
                    disp('Open-loop cost identical: subopt_lev > 0');
                    writetolog(fid_err,'OL cost identical: subopt_lev>0');
                else
                    anyerrors = strvcat(anyerrors,'Open-loop cost identical: ok');
                    disp('Open-loop cost identical: ok');
                    writetolog(fid_err,'OL cost identical: ok');
                end
            else
                anyerrors = strvcat(anyerrors, 'Open-loop cost identical: mismatch!');
                disp('Open-loop cost identical: mismatch!');
                writetolog(fid_err,'OL cost identical: ERROR: MISMATCH!!!');
            end
            if costoptimal,
                if iscell(ctrlStruct.sysStruct.A) & ctrlStruct.probStruct.subopt_lev>0,
                    anyerrors = strvcat(anyerrors,'Open-loop cost optimal: PWA system');
                    writetolog(fid_err,'  OL cost optimal: pwa system');
                elseif ctrlStruct.probStruct.subopt_lev>0,
                    anyerrors = strvcat(anyerrors,'Open-loop cost optimal: subopt_lev > 0');
                    writetolog(fid_err,'  OL cost optimal: subopt_lev>0');
                else
                    anyerrors = strvcat(anyerrors,'Open-loop cost optimal: ok');
                    disp('Open-loop cost optimal: ok');
                    writetolog(fid_err,'  OL cost optimal: ok');
                end
            else
                anyerrors = strvcat(anyerrors, 'Open-loop cost optimal: cost is not optimal!');
                disp('Open-loop cost optimal: cost is not optimal!');
                writetolog(fid_err,'  OL cost optimal: ERROR: cost is NOT OPTIMAL!!!');
            end
            writetolog(fid_run,T);
            [uniqueerrors,b] = unique(anyerrors,'rows');
            writetolog(fid_run, anyerrors(sort(b),:));
        catch
            T = lasterr;
            disp(T);
            writetolog(fid_run,T);
            writetolog(fid_err,T);
        end
        writetolog(fid_run,'.');
        writetolog(fid_err,'.');
    end
    curtime = clock;
    writetolog(fid_run,sprintf('Computation started: %02d.%02d.%d %02d:%02d', ...
        curtime(3), curtime(2), curtime(1), curtime(4), curtime(5)));
    writetolog(fid_err,sprintf('Computation finished: %02d.%02d.%d %02d:%02d', ...
        curtime(3), curtime(2), curtime(1), curtime(4), curtime(5)));
    fclose(fid_run);
    fclose(fid_err);
end
        
        
%-----------------------------------------------------------------
function [pfinalidentical,T] = checkPfinal(ctrlStruct);

sysStruct = ctrlStruct.sysStruct;
probStruct = ctrlStruct.probStruct;

T = '';
pfinalidentical = 0;
if iscell(sysStruct.A) & probStruct.subopt_lev==0 & ~isinf(probStruct.N),
    % special case - PWA system, CFTOC - cannot verify Pfinal
    pfinalidentical = 2;
    return
end

Options.y0bounds =  probStruct.y0bounds;
    
if isinf(probStruct.N) | probStruct.subopt_lev>1,
    % infinite time / minimum time / low complexity solution 
    % they should cover maximum controllable set
    T = evalc('maxCtrlSet = mpt_maxCtrlSet(sysStruct,Options);');
    pfinalidentical = (maxCtrlSet==ctrlStruct.Pfinal);
else
    % CFTOC for linear systems
    Matrices = mpt_constructMatrices(sysStruct, probStruct);
    P=polytope([-Matrices.E Matrices.G],Matrices.W);
    Pfinal = projection(P,1:size(Matrices.E,2));
    pfinalidentical = (Pfinal==ctrlStruct.Pfinal);
end
    


%-----------------------------------------------------------------
function [sysStruct, probStruct,Options] = evalsys(data,options,modifier)

sysStruct = [];
probStruct = [];
Options = [];
T = evalc(data);
T = evalc(options);
T = evalc(modifier);

%-----------------------------------------------------------------
function [nholes,maxR] = holesInSolution(ctrlStruct)
% checks for holes in the solution

if iscell(ctrlStruct.sysStruct.A),
    Pfinal = ctrlStruct.Pfinal;
else
    Pfinal = hull(ctrlStruct.Pfinal);
end
Pn = ctrlStruct.Pn;

holes = Pfinal \ Pn;
if ~isfulldim(holes),
    nholes = 0;
    maxR = 0;
else
    nholes = length(holes);
    [xCheb, Rcheb] = chebyball(holes);
    maxR = max(Rcheb);
end

%-----------------------------------------------------------------
function [costmatch, costoptimal, errtype,Trun] = testCost(ctrlStruct, X0);
% tests if open-loop cost obtained by controller evaluation matches the
% open-loop cost stored in ctrlStruct.Ai, Bi, Ci.

costmatch = 1;
errtype = '';
% if ~iscell(ctrlStruct.sysStruct.A) & ctrlStruct.probStruct.subopt_lev==0,
%     % open-loop cost comparison for LTI systems
%     % compares cost obtained by evaluating performance index
%     Tcost = evalc('iscostok(ctrlStruct, X0);');
%     [costerror, errtypecost] = iserror(Tcost);
%     costmatch = ~costerror;
%     errtype = strvcat(errtype, errtypecost);
% end        

Tcostok = '';
Trun = '';
sysStruct = ctrlStruct.sysStruct;
if iscell(sysStruct.A),
    nx = size(sysStruct.A{1});
else
    nx = size(sysStruct.A);
end

costoptimal = 1;
if ctrlStruct.probStruct.subopt_lev==0 & ...
        ~isfulldim(ctrlStruct.sysStruct.noise) & ~ctrlStruct.probStruct.tracking,
    umax = ctrlStruct.sysStruct.umax(:)';
    umin = ctrlStruct.sysStruct.umin(:)';
    if ~isinf(ctrlStruct.probStruct.N),
        Matrices = mpt_constructMatrices(ctrlStruct.sysStruct, ctrlStruct.probStruct);
    end
    Options.openloop = 1;
    U = [];
    for ii=1:size(X0,1),
        x0 = X0(ii,:)';
        cost = 0; cost2=0;
        try
            Options = [];
            Options.constrviol = 1.005;
            Options.openloop = 1;
            if ctrlStruct.probStruct.tracking,
                Options.reference = [1;zeros(nx-1,1)];
            end
            T=evalc('[X,U,Y,D,cost] = mpt_computeTrajectory(ctrlStruct, x0, [], Options);');
            Trun = strvcat(Trun,T);
            if iserror(T),
                oneline = sprintf('testCost error for state x = %s',mat2str(x0(:)'));
                Trun = strvcat(Trun,oneline);
            end
            Tcostok = strvcat(Tcostok,T);
            Uorig = U;
        catch
            disp('boom');
        end
        
        if isempty(U),
            continue
        end
        
        nu = size(U,2);
        
        for irun=1:5,
            Umod = zeros(size(Uorig));
            for ii=1:size(Uorig,1),
                % perturb the trajectory slightly
                u = Uorig(ii,:);
                umod = u + randn(1,nu)/20/irun;
                if any(umod>umax) | any(umod < umin),
                    umod = u;
                end
                Umod(ii,:) = umod;
            end
            Umodt = Umod';
            Umod2 = Umodt(:);
            if ctrlStruct.probStruct.norm==2 & ~isinf(ctrlStruct.probStruct.N),
                if max(Matrices.G*Umod2 - Matrices.W - Matrices.E*x0) > 0,
                    % perturbed input sequence does not satisfy constraints
                    continue
                end
            end
            Options.manualU = Umod2;
            Options.constrviol = 1;
            cost2 = cost;
            T = 'error';
            try
                if ctrlStruct.probStruct.tracking,
                    Options.reference = [1;zeros(nx-1,1)];
                end
                T=evalc('[X2,U2,Y,D,cost2] = mpt_computeTrajectory(ctrlStruct, x0, [], Options);');
            end
            if iserror(T),
                continue
            end
            if (cost2 - cost) >= -1e8,
                continue
            end
            if ctrlStruct.probStruct.norm~=2,
                if ctrlStruct.probStruct.Tconstraint==2,
                    % test if final state is in Target set (if given
                    if isfulldim(ctrlStruct.probStruct.Tset),
                        if ~isinside(ctrlStruct.probStruct.Tset, X2(size(X2,1),:)'),
                            disp('cost better, but violatest Tset');
                            continue
                        end
                    end
                elseif ctrlStruct.probStruct.Tconstraint==1 & ctrlStruct.probStruct.norm==2,
                    % test if final state is in LQR target set
                    Tset = ctrlStruct.Pn(1);
                    if ~isinside(Tset, X2(end,:)');
                        disp('cost better, but violates LQR Tset');
                        continue
                    end
                end
            end
            if (cost2-cost) < -1e-8,
                line = sprintf('cost not optimal for state %s', mat2str(x0(:)'));
                Trun = strvcat(Trun,line);
                costoptimal = 0;
                break
            end
        end
    end
end
[costerror, errtypecost] = iserror(Tcostok);
costmatch = ~costerror;
errtype = strvcat(errtype, errtypecost);


%-----------------------------------------------------------------
function [feasible,stable,errtype,stopreason,T,Trun] = testStability(ctrlStruct,X0,gridpoints,shouldbestable)
% tests if stability and feasibility is guaranteed

gridpoints = 15;


feaserrwords = {'error', 'warning', 'problem', 'unbounded', ...
        'recomputing', 'constraints exceeded', 'no region', ...
        'mismatch', 'infeasible', 'no pwq', ...
        'no solution', 'failure', 'no success', 'false', ...
        'not symmetrical', 'attention', 'empty',  ...
        'not possible', 'should be', 'failed', ...
        'empty polytope', 'invalid', 'no associated', 'unstable' };

T = '';
Treason = '';
Trun = '';
stopreason = '';

probStruct = ctrlStruct.probStruct;
sysStruct = ctrlStruct.sysStruct;
nx = size(X0,2);

locOpt.display = -1;
%locOpt.fastbreak = 0;
locOpt.openloop = 0;
if isfield(probStruct,'Tset') & isfulldim(probStruct.Tset),
    locOpt.stopInTset = 1;
end
if probStruct.tracking,
    locOpt.reference = [1;zeros(nx-1,1)];
end
for ii=1:size(X0,1),
    x0 = X0(ii,:);
    Tline = evalc('[X,U,Y,D,cost(ii),traj,feasible,dyns,reason]=mpt_computeTrajectory(ctrlStruct,x0,[],locOpt);');
    Trun = strvcat(Trun,Tline);
    if iserror(Tline),
        oneline = sprintf('testStability error for state x = %s',mat2str(x0(:)'));
        Trun = strvcat(Trun,oneline);
    end
    T = strvcat(T, Tline);
    Treason = strvcat(Treason,reason);
end

[notfeasible, errtype] = iserror(T,feaserrwords);
[a,b]=unique(T, 'rows');
T = T(sort(b),:);

[a,b] = unique(errtype, 'rows');
errtype = errtype(sort(b),:);

[a,b,c]=unique(Treason,'rows');
stopreason = '';
for ii=1:size(a,1),
    oneline = deblank(a(ii,:));
    count = findInStrings(Treason,oneline);
    if count==size(Treason,1),
        stopreason = [stopreason,sprintf('%s (all) ',oneline)];
    else
        stopreason = [stopreason,sprintf('%s (%d) ',oneline,count)];
    end
end

feasible = ~notfeasible;
staberrwords = {'not converged'};
notstable = iserror(Treason,staberrwords);
stable = ~notstable;

return

%-----------------------------------------------------------------
function count = findInStrings(T, phrase),

count = 0;
for ii=1:size(T),
    if ~isempty(findstr(T(ii,:),phrase)),
        count = count+1;
    end
end

%-----------------------------------------------------------------
function iscostok(ctrlStruct, X0)
% computes open-loop trajectories

for ii=1:size(X0,1);
    x0 = X0(ii,:)';
    mpt_computeTrajectory(ctrlStruct,x0,[],struct('openloop',1));
end

%-----------------------------------------------------------------
function [res,errortype] = iserror(T,errwords),
% tests for occurence of a given error message in run log

if nargin < 2,
    errwords = {'error', 'warning', 'problem', 'unbounded', ...
        'recomputing', 'constraints exceeded', 'no region', ...
        'reached', 'mismatch', 'infeasible', 'no pwq', ...
        'no solution', 'failure', 'no success', 'false', ...
        'not symmetrical', 'attention', 'empty',  ...
        'not possible', 'should be', 'failed', 'maximum number', ...
        'empty polytope', 'invalid', 'no associated', 'unstable' };
    % 'should not overlap',
end

res = 0;
errortype = '';

Tnew = '';
if size(T,1)==1,
    linebreaks = findstr(T,char(10));
    if ~isempty(linebreaks),
        while 1,
            [line,rest] = strtok(T,char(10));
            Tnew = strvcat(Tnew,line);
            T = rest;
            if isempty(T),
                break
            end
        end
        T = Tnew;
    end
end

for ii=1:size(T,1),
    line = lower(deblank(T(ii,:)));
    for jj=1:length(errwords),
        word = errwords{jj};
        if ~isempty(findstr(line,word)),
            % line contains one of the error words,
            errortype = strvcat(errortype,line);
            res = 1;
        end
    end
end


%-----------------------------------------------------------------
function fid=openlog(fname)

fid = fopen(fname,'w');

if fid<0,
    error(['cannot open ' fname '!']);
end

%-----------------------------------------------------------------
function writetolog(fid, T)

Tnew = '';
if size(T,1)==1,
    while 1,
        [line,rest] = strtok(T,char(10));
        Tnew = strvcat(Tnew,line);
        T = rest;
        if isempty(T),
            break
        end
    end
    T = Tnew;
end    

for ii=1:size(T,1),
    fprintf(fid,'%s\n',deblank(T(ii,:)));
end