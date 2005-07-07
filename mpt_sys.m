function [sysStruct, msg] = mpt_sys(obj, varargin)
% MPT_SYS Converts an object to sysStruct structure
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Converts a given object to MPT's system structure. The following objects can
% be converted:
%   HYS  - HYSDEL source code
%   SS   - State-Space objects of Control Toolbox
%   TF   - Transfer-function objects of Control Toolbox
%   IDSS - State-Space objects of Identification Toolbox
%   MPC  - objects of MPC Toolbox
%
% ---------------------------------------------------------------------------
% USAGE
% ---------------------------------------------------------------------------
% General usage:
%   sysStruct = mpt_sys(object, Ts)
%
% To convert a HYSDEL model, call:
%   sysStruct = mpt_sys('hysdelsource')  - provide name of HYSDEL file in
%                                          apostrophes.
%
% To convert a HYSDEL model without creating an equivalent PWA representation:
%   sysStruct = mpt_sys('hysdelsource', 'nopwa')
%
% To convert from an object, call:
%   sysStruct = mpt_sys(obj)
% e.g.
%   di_ss = ss([1 1; 0 1], [1; 0.5], [1 0], 0);
%   sysStruct = mpt_sys(di_ss, 2)   - discretize the system with sampling time 2
%                                     secs
% 
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% obj          - Input object
% Ts           - Sampling time
%                        
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% sysStruct    - System structure (see 'help mpt_sysStruct' for details)
%


% allowed objects:
%   MPT system structure (sysStruct)
%   MLD structures
%   state space (ss) objects
%   transfer function (tf) objects
%   MPC toolbox objects (mpc)
%   string - name of hysdel source code

global mptOptions
if ~isstruct(mptOptions)
    mpt_error;
end

Options = [];
for ii=length(varargin):-1:1
    if isstruct(varargin{ii})
        Options = varargin{ii};
        break
    end
end

if length(varargin)>0,
    if ischar(varargin{1}),
        if strcmpi(varargin{1}, 'mld') | strcmpi(varargin{1}, 'nopwa'),
            Options.dohys2pwa = 0;
            Options.dummydyns = 1;
        end
    end
end

if ~isfield(Options, 'verbose')
    Options.verbose = mptOptions.verbose;
end
if ~isfield(Options, 'dohys2pwa')
    % if set to true, hysdel model will be transformed to a PWA setup
    Options.dohys2pwa = 1;
end
if ~isfield(Options, 'dummydyns')
    % if true, sets dummy A, B, C, D fields to allow sysStruct with only MLD
    % data (i.e. without hys2pwa conversion)
    Options.dummydyns = 0;
end


if nargin<1
    help mpt_sys
    return
end

sysStruct = [];
nargs = length(varargin);

if isa(obj, 'tf') | isa(obj, 'mpc')
    % input is a transfer function object or an mpc toolbox object
    % convert it to an state-space object
    obj = ss(obj);
end
msg = '';

if isa(obj, 'idss')
    % input is an identification toolbox state-space model
    [sysStruct.A, sysStruct.B, sysStruct.C, sysStruct.D] = ssdata(obj);
    sysStruct.StateName = obj.StateName;
    sysStruct.Ts = obj.Ts;
    itype = 'idss';
    
elseif isa(obj, 'ss')
    % input is a state-space object
    [A, B, C, D, Ts] = ssdata(obj);
    if Ts==0
        % continuous-time model
        if nargs>0,
            for ia = 1:nargs,
                if isa(varargin{ia}, 'double'),
                    Ts = varargin{ia};
                    break
                end
            end
        end
        if Ts==0,
            disp('MPT_SYS: continuous-time model, assuming sampling time Ts=1s');
            Ts = 1;
        end
        % discretize the system
        [A, B, C, D] = ssdata(c2d(obj,Ts));
    end
    sysStruct.A = A;
    sysStruct.B = B;
    sysStruct.C = C;
    sysStruct.D = D;
    sysStruct.Ts = Ts;
    %     if isfield(struct(obj), 'StateName'),
    %         sysStruct.StateName = obj.StateName;
    %     end
    itype = 'ss';
    
elseif isstruct(obj)
    if isfield(obj, 'B') & ~isfield(obj, 'data')
        % this is a sysStruct structure with no MLD data stored inside. we try
        % to convert a PWA model to MLD data
        if ~iscell(obj.B)
            % this is an LTI system, no conversion needed
            sysStruct = obj;
            return
        end
        fprintf('\n');
        disp('======================================================================');
        disp('WARNING: No MLD model available in system structure! Trying to convert');
        disp('         sysStruct to MLD data. This conversion can be terribly');
        disp('         inefficient! You should always opt for HYSDEL model instead.');
        disp('======================================================================');
        fprintf('\n');
        if ~isfield(obj, 'xmax') | ~isfield(obj, 'xmin')
            % xmax and xmin must be defined for pwa2mld translation
            error(sprintf('"%s.xmin" and "%s.xmax" must be defined!', inputname(1), inputname(1)));
        end
        fname = tempname;
        mpt_ss2mld(obj, fname);
        newSysStruct = mpt_sys(fname, struct('verbose', 0, 'dohys2pwa', 0));
        obj.data = newSysStruct.data;
        sysStruct = obj;
        itype = 'sysStruct';
        
    elseif isfield(obj, 'B')
        % MPT system structure
        sysStruct = obj;
        itype = 'sysStruct';
        
    elseif isfield(obj, 'symtable') & isfield(obj, 'Arr')
        % MLD structure generated by HYSDEL
        if Options.dohys2pwa,        
            PWA = hys2pwa(obj);
            sysStruct = mpt_pwa2sys(PWA, obj);
        else
            % create system structure based on MLD structure without creating
            % equivalent PWA transformation
            fprintf('Warning: Simulations will not be available for this type of input.\n');
            S = obj;
            nx = max(S.nx, 1);
            nu = max(S.nu, 1);
            ny = max(S.ny, 1);
            sysStruct.A{1} = eye(nx);
            sysStruct.B{1} = eye(nx, nu);
            sysStruct.C{1} = eye(ny, nx);
            sysStruct.D{1} = zeros(ny, nu);
            sysStruct.guardX{1} = eye(1, nx);
            sysStruct.guardU{1} = zeros(1, nu);
            sysStruct.guardC{1} = mptOptions.infbox;
            sysStruct.xmax = S.xu;
            sysStruct.xmin = S.xl;
            sysStruct.umax = S.uu;
            sysStruct.umin = S.ul;
            sysStruct.data.onlymld = 1;
            sysStruct.data.MLD = S;
            sysStruct.data.hysdel.code = '';
            sysStruct.data.hysdel.code = inputname(1);
            sysStruct.data.SIM.code = '';
            sysStruct.data.SIM.params = {};
        end
        itype = 'mld';
        
    else
        error('MPT_SYS: unknown input structure type!');
    end
    
elseif isa(obj, 'char')
    % string - possible hysdel source filename
    [pathstr, namestr, extstr] = fileparts(obj);
    if isempty(extstr),
        extstr = '.hys';
    end
    if strcmpi(extstr, '.mhys'),
        matrixhysdel = 1;
    else
        matrixhysdel = 0;
    end
    if ~isempty(pathstr)
        fname = [pathstr filesep namestr];
    else
        fname = namestr;
    end
    
    outfile = tempname;
    simfile = [outfile '_sim'];
    
    %====================================================================
    % call hysdel and handle any possible errors
    [status, msg] = callhysdel(fname, outfile, simfile, matrixhysdel);
    if status~=0,
        % there was an error
        if ~matrixhysdel
            % remove additional textual output generated by HYSDEL
            vpos = findstr(msg, 'later version.');
            if ~isempty(vpos)
                msg = msg(vpos+length('later version. '):end);
            end
        end
        fprintf('\n');
        disp(msg);
        fprintf('\n');
        if nargout>1,
            disp('MPT_SYS: HYSDEL produced an error');
            sysStruct = [];
            return
        else
            error('MPT_SYS: HYSDEL produced an error');
        end
    end

    %====================================================================
    % double-check if output files exist
    outfile = [outfile '.m'];
    simfile = [simfile '.m'];
    if ~exist(outfile, 'file')
        fprintf('\nMake sure you have write permissions for the diredtory "%s"\n\n', tempdir);
        error('MPT_SYS: no output generated by HYSDEL.');
    end
    if ~exist(simfile, 'file')
        fprintf('\nMake sure you have write permissions for the diredtory "%s"\n\n', tempdir);
        error('MPT_SYS: no simulator generated by HYSDEL');
    end

    %===========================================================================
    % handle parameters defined in matlab workspace
    parameters = {};
    try
        parameters = sub_readparams(outfile);
    end
    if ~isempty(parameters)
        % check if parameters are defined in global workspace
        params = [];
        try
            params = evalin('base', 'params');
        end
        if ~isempty(params),
            % 'params' structure found in global workspace
            if ~isstruct(params),
                error('MPT_SYS: ''params'' variable is not a structure!');
            end
        
            % check if each parameter is defined in the 'params' structure
            pfields = fields(params);
            for ip1 = 1:length(parameters),
                param_found = 0;
                for ip2 = 1:length(pfields),
                    if strcmp(pfields{ip2}, parameters{ip1}),
                        param_found = 1;
                        break
                    end
                end
                if param_found==0
                    error(sprintf('MPT_SYS: ''params.%s'' is not defined!', parameters{ip1}));
                end
            end
                        
        else
            params = [];
            % 'params' structure not found in global workspace
            for ip = 1:length(parameters)
                try
                    param_value = evalin('base', parameters{ip});
                catch
                    error(sprintf('MPT_SYS: parameter ''%s'' not defined in global workspace!', ...
                        parameters{ip}));
                end
                if ~isa(param_value, 'double')
                    error(sprintf('MPT_SYS: parameter ''%s'' must be a double!', paramater{ip}));
                end
                params = setfield(params, parameters{ip}, param_value);
            end
        end
    end

    %====================================================================
    % load MLD matrices from hysdel output file
    wstatus = warning;
    warning off
    try
        run(outfile);
        SIMCODE = sub_readsimcode(simfile);
    catch
        disp(lasterr);
        error('MPT_SYS: corrupted HYSDEL output, see message above.');
    end
    warning(wstatus);
    if ~exist('S', 'var')
        error('MPT_SYS: corrupted HYSDEL output, see message above.');
    end

    [pathstr, namestr, extstr] = fileparts(fname);
    if isempty(extstr),
        if matrixhysdel,
            extstr = '.mhys';
        else
            extstr = '.hys';
        end
        if isempty(pathstr),
            fname = [namestr extstr];
        else
            fname = [pathstr filesep namestr extstr];
        end
    end
    HYSCODE = readfile(fname);
    
    %====================================================================
    % remove temporary files
    if exist(outfile, 'file')
        delete(outfile);
    end
    if exist(simfile, 'file')
        delete(simfile);
    end

    if Options.dohys2pwa,
        %====================================================================
        % convert MLD model to a PWA form
        PWA = hys2pwa(S,-1);
        
        %====================================================================
        % convert hys2pwa output to sysStruct format
        sysStruct = mpt_pwa2sys(PWA, S);
    else
        if Options.dummydyns,
            nx = S.nx;
            nu = S.nu;
            ny = S.ny;
            sysStruct.A{1} = eye(nx);
            sysStruct.B{1} = zeros(nx, nu);
            sysStruct.C{1} = eye(ny, nx);
            sysStruct.D{1} = zeros(ny, nu);
            sysStruct.guardX{1} = eye(1, nx);
            sysStruct.guardC{1} = 1e6;
            sysStruct.data.onlymld = 1;
        
            % extract state/input constraints from MLD representation
            if isfield(S, 'nxb')
                if S.nxb~=0,
                    sysStruct.xbool = S.nxr+1:S.nx;
                end
            end
            if isfield(S, 'uu') & isfield(S, 'ul'),
                if all(~isinf(S.uu)) & all(~isinf(S.ul)) & S.nu>0,
                    sysStruct.umax = S.uu;
                    sysStruct.umin = S.ul;
                end
            end
            if isfield(S, 'xu') & isfield(S, 'xl'),
                if all(~isinf(S.xu)) & all(~isinf(S.xl)) & nx>0,
                    sysStruct.xmin = S.xl;
                    sysStruct.xmax = S.xu;
                end
            end
        end
        
        sysStruct.data.MLD = S;
    end
    
    %====================================================================
    % write additional info to sysStruct
    sysStruct.data.hysdel.code = HYSCODE;
    sysStruct.data.SIM.code = SIMCODE;
    sysStruct.data.SIM.params = parameters;
    sysStruct.data.hysdel.fromfile = obj;
    itype = 'hysdel';
    
else
    error(sprintf('MPT_SYS: cannot convert inputs of type "%s".', class(obj)));
end

% define default constraints on inputs and outputs
if isstruct(sysStruct)
    %     if isfield(sysStruct, 'B'),
    %         if ~isfield(sysStruct, 'umax') & ~isfield(sysStruct, 'umin'),
    %             if iscell(sysStruct.B),
    %                 nu = size(sysStruct.B{1}, 2);
    %             else
    %                 nu = size(sysStruct.B, 2);
    %             end
    %             sysStruct.umax = repmat(1e3, nu, 1);
    %             sysStruct.umin = repmat(-1e3, nu, 1);
    %             if isfield(sysStruct, 'Uset'),
    %                 if iscell(sysStruct.Uset),
    %                     for ii = 1:length(sysStruct.Uset),
    %                         sysStruct.umax(ii) = max(sysStruct.Uset{ii});
    %                         sysStruct.umin(ii) = min(sysStruct.Uset{ii});
    %                     end
    %                 else
    %                     sysStruct.umax = max(sysStruct.Uset);
    %                     sysStruct.umin = min(sysStruct.Uset);
    %                 end
    %             end
    %         end
    %         sysStruct.umax(find(sysStruct.umax==Inf)) = 1e3;
    %         sysStruct.umin(find(sysStruct.umin==-Inf)) = -1e3;
    %     end
    %     if isfield(sysStruct, 'C')
    %         if ~isfield(sysStruct, 'ymax') & ~isfield(sysStruct, 'ymin'),
    %             if iscell(sysStruct.C),
    %                 ny = size(sysStruct.C{1}, 1);
    %             else
    %                 ny = size(sysStruct.C, 1);
    %             end
    %             sysStruct.ymax = repmat(1e3, ny, 1);
    %             sysStruct.ymin = repmat(-1e3, ny, 1);
    %         end
    %         sysStruct.ymax(find(sysStruct.ymax==Inf)) = 1e3;
    %         sysStruct.ymin(find(sysStruct.ymin==-Inf)) = -1e3;
    %     end
end

if nargs>0,
    for ia = 1:nargs,
        if isa(varargin{ia}, 'double'),
            sysStruct.Ts = varargin{ia};
            break
        end
    end
end

if ~strcmpi(itype, 'sysStruct') & nargin==1,
    return
end

return


if rem(nargs, 2)~=0,
    error('MPT_SYS: object properties must form a pair ''PropertyName'', PropertyValue !');
end


%====================================================================
% handle any additional input arguments (constraints)
nargs = length(varargin);
if rem(nargs, 2)~=0,
    error('MPT_SYS: object properties must form a pair ''PropertyName'', PropertyValue !');
end
for ia = 1:2:nargs
    pname = varargin{ia};
    if ~ischar(pname),
        error('MPT_SYS: ''PropertyName'' must be a string!');
    end
    if strcmp(pname, 'umax') | strcmp(pname, 'umin') | ...
            strcmp(pname, 'ymax') | strcmp(pname, 'ymin') | ...
            strcmp(pname, 'dumax') | strcmp(pname, 'dumin') | ...
            strcmp(pname, 'dymax') | strcmp(pname, 'dymin') | ...
            strcmp(pname, 'noise')
        
        % legal property, set it
        
        par = varargin{ia+1};
        if strcmp(pname, 'noise')
            if ~isa(par, 'polytope')
                error('MPT_SYS: argument following ''noise'' property must be a polytope object!');
            end
        end
        
        if isa(par, 'double')
            par = par(:);
        else
            error(['MPT_SYS: ''' pname ''' must be a double!']);
        end
        sysStruct = setfield(sysStruct, pname, par);
    end
end

if ~isfield(sysStruct, 'umax')
    warning('sysStruct.umax is missing');
end
if ~isfield(sysStruct, 'umin')
    warning('sysStruct.umin is missing');
end
if ~isfield(sysStruct, 'ymax')
    warning('sysStruct.ymax is missing');
end
if ~isfield(sysStruct, 'ymin')
    warning('sysStruct.ymin is missing');
end
    

%-------------------------------------------------------------------------------
function out = readfile(fname)
% reads file into a string

fid = fopen(fname, 'r');
if fid<0
    error(['cannot open "' fname '" for reading! Make sure the file exists and has proper permissions set.']);
end

out = '';
while 1
    tline = fgetl(fid);
    if ischar(tline)
        out = strvcat(out, tline);
    else
        break
    end
end

fclose(fid);


%-------------------------------------------------------------------------------
function params = sub_readparams(outfile)
% detects if the hysdel code uses any symbolical parameters which will be taken
% from global workspace

fid = fopen(outfile, 'r');
if fid<0
    error(['cannot open "' outfile '" for reading! Make sure the file exists and has proper permissions set.']);
end

params = {};

paramstring = 'if ~isfield(params, ';

while 1,
    tline = fgetl(fid);
    if ~ischar(tline), break, end
    if ~isempty(findstr(tline, paramstring)),
        % read parameter name
        apppos = findstr(tline, '''');
        if length(apppos)==2,
            % double check if there are only two apostrophes
            params{end+1} = tline(apppos(1)+1:apppos(2)-1);
        end
    end
end

fclose(fid);

%-------------------------------------------------------------------------------
function simcode=sub_readsimcode(simfile)
% reads the simulator code into a string

simcode = '';

fid = fopen(simfile, 'r');
if fid<0
    error(['cannot open input file "' simfile '" for reading!']);
end

linec = 0;
while 1,
    tline = fgetl(fid);
    if ~ischar(tline), break, end
    linec = linec + 1;
    
    % skip first line (defines function) and comments
    if linec==1, continue, end
    if length(tline)>1,
        if tline(1)=='%',
            continue
        end
    end

    % skip sub-function
    if ~isempty(findstr(tline, 'function within(x, lo, hi, line)'))
        break
    end
    
    %simcode = strvcat(simcode, tline);
    if isempty(findstr(tline, '%')) & ~isempty(tline)
        simcode = [simcode ' ' tline];
        if tline(end)~=';',
            simcode = [simcode ','];
        end
    end
end

fclose(fid);


%-------------------------------------------------------------------------------
function [s,w]=callhysdel(hysfile, outfile, simfile, matrixhysdel)
% calls hysdel binary

if matrixhysdel
    % use MATRIX HYSDEL
    try
        Opt.outfname = outfile;
        w = evalc('mathys(hysfile, simfile, Opt);');
        s = 0;
    catch
        s = 1;
        w = 'Couldn''t execute MATRIX HYSDEL.';
    end
    return
end

global mptOptions

hysdel_bin = '';

if isfield(mptOptions, 'hysdelpath'),
    if ~isempty(mptOptions.hysdelpath),
        hysdel_bin = mptOptions.hysdelpath;
    end
end

if isempty(hysdel_bin),
    comp = computer;
    if ispc,
        hysdel_bin = which('hysdel.exe');
    elseif strcmpi(comp, 'glnx86'),
        hysdel_bin = which('hysdel.linux');
    elseif strcmpi(comp, 'sol2'),
        hysdel_bin = which('hysdel.sol');
    end
end

if isempty(hysdel_bin),
    error('Couldn''t find HYSDEL binary! Run ''mpt_setup'' to set it''s location.');
end

if ispc,
    hysdel_cmd = sprintf('"%s" -i "%s.hys" -m "%s" -s "%s" -5', hysdel_bin, ...
        hysfile, outfile, simfile);
else
    hysdel_cmd = sprintf('%s -i %s.hys -m %s -s %s -5', hysdel_bin, ...
        hysfile, outfile, simfile);
end    

try
    [s,w]=system(hysdel_cmd);
catch
    s = 1;
    w = sprintf('Couldn''t execute "%s" !', hysdel_bin);
end